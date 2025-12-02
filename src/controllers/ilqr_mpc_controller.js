/**
 * ETH-style SE(3) iLQR Model Predictive Controller for quadrotors.
 *
 * Coordinate frames and motor layout (must match physics + EthController):
 * - World frame:  +Z up, +X forward, +Y left.
 * - Body frame:   +X forward, +Y left, +Z up (thrust direction).
 * - Orientation:  unit quaternion mapping body -> world (right handed).
 * - Motor X-configuration:
 *     Motor 0 = Front-Left  (CW)  at (+X, +Y)
 *     Motor 1 = Front-Right (CCW) at (+X, -Y)
 *     Motor 2 = Back-Right  (CW)  at (-X, -Y)
 *     Motor 3 = Back-Left   (CCW) at (-X, +Y)
 *
 * This controller keeps the public MPC interface used across the demos but
 * replaces the unstable finite-difference gradient descent with a full iLQR
 * (iterative Linear Quadratic Regulator) solver using analytic SE(3)
 * linearisation. Rotor mixing and all physical conventions exactly mirror
 * EthController to stay consistent with the simulator.
 */

import { EthController } from '../../drones/stationary/eth_controller.js';

const STATE_SIZE = 13; // [p(3) v(3) q(4) omega(3)]
const CONTROL_SIZE = 4; // [thrust tau_x tau_y tau_z]

const ZERO_VEC3 = () => new THREE.Vector3(0, 0, 0);

const crossMatrix = (v) => ([
  [0, -v.z, v.y],
  [v.z, 0, -v.x],
  [-v.y, v.x, 0],
]);

const matMul = (A, B) => {
  const rows = A.length;
  const cols = B[0].length;
  const res = Array.from({ length: rows }, () => Array(cols).fill(0));
  for (let i = 0; i < rows; i += 1) {
    for (let k = 0; k < B.length; k += 1) {
      const aik = A[i][k];
      for (let j = 0; j < cols; j += 1) {
        res[i][j] += aik * B[k][j];
      }
    }
  }
  return res;
};

const matVecMul = (A, v) => {
  const res = new Array(A.length).fill(0);
  for (let i = 0; i < A.length; i += 1) {
    const row = A[i];
    let sum = 0;
    for (let j = 0; j < row.length; j += 1) sum += row[j] * v[j];
    res[i] = sum;
  }
  return res;
};

const addVecs = (a, b) => a.map((v, i) => v + b[i]);
const subVecs = (a, b) => a.map((v, i) => v - b[i]);
const scaleVec = (v, s) => v.map((x) => x * s);

export class ILQRMPCController extends EthController {
  constructor(params = {}) {
    super(params);

    this.horizonSteps = params.mpcHorizon || 20;
    this.predictionDt = params.predictionDt || 0.05;
    this.maxIterations = params.maxIterations || 12;
    this.lineSearch = [1.0, 0.7, 0.5, 0.3, 0.1, 0.05];

    this.weights = {
      position: params.wPosition || 80.0,
      velocity: params.wVelocity || 10.0,
      tilt: params.wTilt || 0.25,
      yawRate: params.wYawRate || 0.2,
      thrust: params.wThrust || 1e-3,
      torque: params.wTorque || 1e-4,
      smooth: params.wSmooth || 5e-3,
      terminalPosition: params.wTerminalPosition || 160.0,
      terminalVelocity: params.wTerminalVelocity || 20.0,
    };

    this.refVelocityGain = params.refVelocityGain || 1.4;
    this.maxRefVelocity = params.maxRefVelocity || 4.0;
    this.refAccelGain = params.refAccelGain || 2.0;
    this.maxRefAcceleration = params.maxRefAcceleration || 8.0;

    const T_max = params.maxThrust || 4 * (params.maxThrustPerMotor || this.maxThrustPerMotor);
    this.constraints = {
      minThrust: params.minThrust || 0,
      maxThrust: T_max,
      maxTorque: params.maxTorque || 0.02,
      maxYawTorque: params.maxYawTorque || 0.01,
      maxTiltAngle: params.maxTiltAngle
        ? THREE.MathUtils.degToRad(params.maxTiltAngle)
        : this.maxTiltAngle,
    };

    this.previousSolution = null;
    this.predictedTrajectory = [];
    this.lastOptimizationTime = 0;
    this.lastCost = 0;
  }

  reset() {
    super.reset();
    this.previousSolution = null;
    this.predictedTrajectory = [];
    this.lastOptimizationTime = 0;
    this.lastCost = 0;
  }

  computeControl(state, target, dt) {
    const start = performance.now();
    const x0 = this._stateToVector(state);
    const ref = this._buildReference(target, state);

    let controlSeq = this._warmStart();
    let nominal = this._rollout(x0, controlSeq, ref);
    let bestCost = nominal.cost;

    for (let iter = 0; iter < this.maxIterations; iter += 1) {
      const linearization = this._linearizeTrajectory(nominal.states, controlSeq);
      const { kSeq, KSeq } = this._backwardPass(linearization, ref);
      let accepted = false;
      for (const alpha of this.lineSearch) {
        const candidate = this._forwardPass(x0, controlSeq, nominal.states, kSeq, KSeq, alpha, ref);
        if (candidate.cost < bestCost) {
          controlSeq = candidate.controls;
          nominal = candidate;
          bestCost = candidate.cost;
          accepted = true;
          break;
        }
      }
      if (!accepted) break;
      if (iter > 0 && Math.abs(this.lastCost - bestCost) < 1e-4) break;
      this.lastCost = bestCost;
    }

    this.previousSolution = { controls: controlSeq, states: nominal.states, cost: bestCost };
    this.predictedTrajectory = nominal.states.map((x) => this._vectorToState(x));
    this.lastOptimizationTime = performance.now() - start;

    const u0 = controlSeq[0];
    const [thrust, tauX, tauY, tauZ] = this._clampControl(u0);
    const motorCommands = this._mixToMotors(thrust, tauX, tauY, tauZ, state.orientationQuat);

    return {
      motorCommands,
      predictedTrajectory: this.predictedTrajectory,
      optimizationTime: this.lastOptimizationTime,
      cost: bestCost,
    };
  }

  getPredictedTrajectory() {
    return this.predictedTrajectory;
  }

  getMetrics() {
    return {
      optimizationTime: this.lastOptimizationTime,
      horizonSteps: this.horizonSteps,
      predictionTime: this.horizonSteps * this.predictionDt,
      cost: this.lastCost,
    };
  }

  // ---------------------------------------------------------------------------
  // iLQR machinery
  // ---------------------------------------------------------------------------

  _warmStart() {
    const hoverThrust = this.mass * this.gravity;
    if (this.previousSolution?.controls?.length === this.horizonSteps) {
      const shifted = this.previousSolution.controls.slice(1);
      const last = this.previousSolution.controls[this.previousSolution.controls.length - 1];
      shifted.push([...last]);
      return shifted;
    }
    return Array.from({ length: this.horizonSteps }, () => [hoverThrust, 0, 0, 0]);
  }

  _buildReference(target, state) {
    const start = state?.position?.clone() || ZERO_VEC3();
    const posTarget = target?.position?.clone() || ZERO_VEC3();
    const yawTarget = target?.yaw || 0;

    const dir = posTarget.clone().sub(start);
    const desiredVel = dir.multiplyScalar(this.refVelocityGain);
    if (desiredVel.length() > this.maxRefVelocity) {
      desiredVel.setLength(this.maxRefVelocity);
    }

    const refTrajectory = [];
    let pos = start.clone();
    let vel = (state?.velocity?.clone() || ZERO_VEC3());
    for (let i = 0; i <= this.horizonSteps; i += 1) {
      const acc = desiredVel.clone().sub(vel).multiplyScalar(this.refAccelGain);
      if (acc.length() > this.maxRefAcceleration) acc.setLength(this.maxRefAcceleration);
      vel = vel.clone().add(acc.clone().multiplyScalar(this.predictionDt));
      const nextPos = pos.clone().add(vel.clone().multiplyScalar(this.predictionDt));
      refTrajectory.push({
        position: nextPos.clone(),
        velocity: vel.clone(),
        acceleration: acc,
        yaw: yawTarget,
      });
      pos.copy(nextPos);
    }
    return refTrajectory;
  }

  _stateToVector(state) {
    const v = new Array(STATE_SIZE).fill(0);
    v[0] = state.position.x; v[1] = state.position.y; v[2] = state.position.z;
    v[3] = state.velocity.x; v[4] = state.velocity.y; v[5] = state.velocity.z;
    v[6] = state.orientationQuat.x; v[7] = state.orientationQuat.y; v[8] = state.orientationQuat.z; v[9] = state.orientationQuat.w;
    v[10] = state.angularVelocity.x; v[11] = state.angularVelocity.y; v[12] = state.angularVelocity.z;
    return v;
  }

  _vectorToState(vec) {
    return {
      position: new THREE.Vector3(vec[0], vec[1], vec[2]),
      velocity: new THREE.Vector3(vec[3], vec[4], vec[5]),
      orientationQuat: new THREE.Quaternion(vec[6], vec[7], vec[8], vec[9]).normalize(),
      angularVelocity: new THREE.Vector3(vec[10], vec[11], vec[12]),
    };
  }

  _mixToMotors(thrust, tauX, tauY, tauZ) {
    const L = this.L;
    const kF = this.kF;
    const kM = this.kM;
    const T_max = this.maxThrustPerMotor;

    const solveThrusts = (yawTorque) => {
      const S = thrust / kF;
      const Tx = tauX * Math.SQRT2 / (L * kF);
      const Ty = tauY * Math.SQRT2 / (L * kF);
      const Tz = yawTorque / kM;

      const f0 = 0.25 * (S + Tx + Ty - Tz);
      const f1 = 0.25 * (S - Tx + Ty + Tz);
      const f2 = 0.25 * (S - Tx - Ty - Tz);
      const f3 = 0.25 * (S + Tx - Ty + Tz);
      return [f0 * kF, f1 * kF, f2 * kF, f3 * kF];
    };

    let [T0, T1, T2, T3] = solveThrusts(tauZ);
    const anyOverMax = () => T0 > T_max || T1 > T_max || T2 > T_max || T3 > T_max;
    const anyNegative = () => T0 < 0 || T1 < 0 || T2 < 0 || T3 < 0;
    if (anyOverMax() || anyNegative()) {
      [T0, T1, T2, T3] = solveThrusts(0);
      if (anyOverMax()) {
        const scale = T_max / Math.max(T0, T1, T2, T3);
        T0 *= scale; T1 *= scale; T2 *= scale; T3 *= scale;
      }
    }

    T0 = Math.min(Math.max(T0, 0), T_max);
    T1 = Math.min(Math.max(T1, 0), T_max);
    T2 = Math.min(Math.max(T2, 0), T_max);
    T3 = Math.min(Math.max(T3, 0), T_max);

    const omegaRange = this.omegaMax - this.omegaMin;
    const thrustToCmd = (T) => {
      const omega = Math.sqrt(Math.max(T, 0) / kF);
      const u = (omega - this.omegaMin) / omegaRange;
      return THREE.MathUtils.clamp(u, 0, 1);
    };

    return [thrustToCmd(T0), thrustToCmd(T1), thrustToCmd(T2), thrustToCmd(T3)];
  }

  _clampControl(u) {
    const thrust = THREE.MathUtils.clamp(u[0], this.constraints.minThrust, this.constraints.maxThrust);
    const tauX = THREE.MathUtils.clamp(u[1], -this.constraints.maxTorque, this.constraints.maxTorque);
    const tauY = THREE.MathUtils.clamp(u[2], -this.constraints.maxTorque, this.constraints.maxTorque);
    const tauZ = THREE.MathUtils.clamp(u[3], -this.constraints.maxYawTorque, this.constraints.maxYawTorque);
    return [thrust, tauX, tauY, tauZ];
  }

  _rollout(x0, controls, ref) {
    const states = [x0.slice()];
    let cost = 0;
    for (let k = 0; k < controls.length; k += 1) {
      const xk = states[states.length - 1];
      const uk = this._clampControl(controls[k]);
      const refStep = ref?.[k] || null;
      cost += this._stageCost(xk, uk, refStep, k === controls.length - 1, k > 0 ? controls[k - 1] : null);
      const xNext = this._integrate(xk, uk, this.predictionDt);
      states.push(xNext);
    }
    cost += this._terminalCost(states[states.length - 1], ref?.[ref.length - 1]);
    return { states, cost };
  }

  _forwardPass(x0, uNom, xNom, kSeq, KSeq, alpha, ref) {
    const N = uNom.length;
    const states = [x0.slice()];
    const controls = [];
    let cost = 0;
    for (let k = 0; k < N; k += 1) {
      const dx = subVecs(states[k], xNom[k]);
      const du = addVecs(uNom[k], addVecs(scaleVec(kSeq[k], alpha), matVecMul(KSeq[k], dx)));
      const uk = this._clampControl(du);
      controls.push(uk);
      const refStep = ref[k] || null;
      cost += this._stageCost(states[k], uk, refStep, false, k > 0 ? controls[k - 1] : null);
      const xNext = this._integrate(states[k], uk, this.predictionDt);
      states.push(xNext);
    }
    cost += this._terminalCost(states[states.length - 1], ref[ref.length - 1]);
    return { states, controls, cost };
  }

  _linearizeTrajectory(states, controls) {
    const A = [];
    const B = [];
    for (let k = 0; k < controls.length; k += 1) {
      const lin = this._linearize(states[k], controls[k]);
      A.push(lin.A);
      B.push(lin.B);
    }
    return { A, B, states, controls };
  }

  _backwardPass(traj, ref) {
    const N = traj.controls.length;
    const kSeq = new Array(N);
    const KSeq = new Array(N);

    let Vx = this._terminalGrad(traj.states[N], ref[N]);
    let Vxx = this._terminalHess(traj.states[N], ref[N]);

    const reg = 1e-6;

    for (let k = N - 1; k >= 0; k -= 1) {
      const A = traj.A[k];
      const B = traj.B[k];
      const refStep = ref[k];

      const { lx, lu, lxx, luu, lux } = this._costDerivatives(traj.states[k], traj.controls[k], refStep, false, k > 0 ? traj.controls[k - 1] : null);

      const AT_Vx = matVecMul(this._transpose(A), Vx);
      const BT_Vx = matVecMul(this._transpose(B), Vx);

      const Qx = addVecs(lx, AT_Vx);
      const Qu = addVecs(lu, BT_Vx);

      const At_Vxx = matMul(this._transpose(A), Vxx);
      const Bt_Vxx = matMul(this._transpose(B), Vxx);

      const Qxx = this._addMatrices(lxx, matMul(At_Vxx, A));
      const Quu = this._addMatrices(luu, matMul(Bt_Vxx, B));
      const Qux = this._addMatrices(lux, matMul(Bt_Vxx, A));

      for (let i = 0; i < CONTROL_SIZE; i += 1) {
        Quu[i][i] += reg;
      }

      const QuuInv = this._invertSymmetric(Quu);
      const kFeedforward = scaleVec(matVecMul(QuuInv, Qu), -1);
      const KFeedback = matMul(scaleVecMatrix(QuuInv, -1), Qux);

      kSeq[k] = kFeedforward;
      KSeq[k] = KFeedback;

      const Kt_Quu = matMul(this._transpose(KFeedback), Quu);
      const Kt_Quu_K = matMul(Kt_Quu, KFeedback);

      const Vx_new = addVecs(Qx, matVecMul(this._transpose(KFeedback), Qu));
      const Vxx_new = this._addMatrices(Qxx, this._addMatrices(matMul(this._transpose(KFeedback), Qux), Kt_Quu_K));

      Vx = Vx_new;
      Vxx = this._symmetrize(Vxx_new);
    }

    return { kSeq, KSeq };
  }

  _stageCost(x, u, ref, isTerminal = false, uPrev = null) {
    const { position, velocity, orientationQuat, angularVelocity } = this._vectorToState(x);
    const pref = ref?.position || ZERO_VEC3();
    const vref = ref?.velocity || ZERO_VEC3();
    const posErr = position.clone().sub(pref);
    const velErr = velocity.clone().sub(vref);
    const hoverThrust = this.mass * this.gravity;

    const b3 = new THREE.Vector3(0, 0, 1).applyQuaternion(orientationQuat);
    const desiredAccel = ref?.acceleration?.clone() || ZERO_VEC3();
    const b3Ref = desiredAccel.lengthSq() > 1e-6
      ? desiredAccel.clone().add(new THREE.Vector3(0, 0, this.gravity)).normalize()
      : new THREE.Vector3(0, 0, 1);
    const tiltErr = b3.clone().sub(b3Ref);

    let cost = 0;
    cost += this.weights.position * posErr.lengthSq();
    cost += this.weights.velocity * velErr.lengthSq();
    if (!isTerminal) {
      cost += this.weights.tilt * tiltErr.lengthSq();
    }
    cost += this.weights.yawRate * angularVelocity.z * angularVelocity.z;
    if (!isTerminal) {
      cost += this.weights.thrust * (u[0] - hoverThrust) * (u[0] - hoverThrust);
      cost += this.weights.torque * (u[1] * u[1] + u[2] * u[2] + u[3] * u[3]);
      if (uPrev) {
        const du = u.map((val, idx) => val - uPrev[idx]);
        cost += this.weights.smooth * (du[0] * du[0] + du[1] * du[1] + du[2] * du[2] + du[3] * du[3]);
      }
    }
    return cost;
  }

  _terminalCost(x, ref) {
    const { position, velocity } = this._vectorToState(x);
    const pref = ref?.position || ZERO_VEC3();
    const vref = ref?.velocity || ZERO_VEC3();
    const posErr = position.clone().sub(pref);
    const velErr = velocity.clone().sub(vref);
    return this.weights.terminalPosition * posErr.lengthSq()
      + this.weights.terminalVelocity * velErr.lengthSq();
  }

  _costDerivatives(x, u, ref, isTerminal = false, uPrev = null) {
    const n = STATE_SIZE;
    const m = CONTROL_SIZE;
    const lx = new Array(n).fill(0);
    const lu = new Array(m).fill(0);
    const lxx = Array.from({ length: n }, () => new Array(n).fill(0));
    const luu = Array.from({ length: m }, () => new Array(m).fill(0));
    const lux = Array.from({ length: m }, () => new Array(n).fill(0));

    const state = this._vectorToState(x);
    const pref = ref?.position || ZERO_VEC3();
    const vref = ref?.velocity || ZERO_VEC3();

    const posErr = state.position.clone().sub(pref);
    const velErr = state.velocity.clone().sub(vref);

    lx[0] = 2 * this.weights.position * posErr.x;
    lx[1] = 2 * this.weights.position * posErr.y;
    lx[2] = 2 * this.weights.position * posErr.z;
    lxx[0][0] = 2 * this.weights.position;
    lxx[1][1] = 2 * this.weights.position;
    lxx[2][2] = 2 * this.weights.position;

    lx[3] = 2 * this.weights.velocity * velErr.x;
    lx[4] = 2 * this.weights.velocity * velErr.y;
    lx[5] = 2 * this.weights.velocity * velErr.z;
    lxx[3][3] = 2 * this.weights.velocity;
    lxx[4][4] = 2 * this.weights.velocity;
    lxx[5][5] = 2 * this.weights.velocity;

    if (!isTerminal) {
      const b3 = new THREE.Vector3(0, 0, 1).applyQuaternion(state.orientationQuat);
      const desiredAccel = ref?.acceleration?.clone() || ZERO_VEC3();
      const b3Ref = desiredAccel.lengthSq() > 1e-6
        ? desiredAccel.clone().add(new THREE.Vector3(0, 0, this.gravity)).normalize()
        : new THREE.Vector3(0, 0, 1);
      const tiltErr = b3.clone().sub(b3Ref);
      const R = new THREE.Matrix3().setFromMatrix4(new THREE.Matrix4().makeRotationFromQuaternion(state.orientationQuat));
      const dRe3_dqvec = this._dRe3dq(R);
      const tiltGrad = matVecMul(this._transpose(dRe3_dqvec), [tiltErr.x, tiltErr.y, tiltErr.z]);
      lx[6] += 2 * this.weights.tilt * tiltGrad[0];
      lx[7] += 2 * this.weights.tilt * tiltGrad[1];
      lx[8] += 2 * this.weights.tilt * tiltGrad[2];
      const tiltHess = matMul(this._transpose(dRe3_dqvec), dRe3_dqvec);
      lxx[6][6] += 2 * this.weights.tilt * tiltHess[0][0];
      lxx[6][7] += 2 * this.weights.tilt * tiltHess[0][1];
      lxx[6][8] += 2 * this.weights.tilt * tiltHess[0][2];
      lxx[7][6] += 2 * this.weights.tilt * tiltHess[1][0];
      lxx[7][7] += 2 * this.weights.tilt * tiltHess[1][1];
      lxx[7][8] += 2 * this.weights.tilt * tiltHess[1][2];
      lxx[8][6] += 2 * this.weights.tilt * tiltHess[2][0];
      lxx[8][7] += 2 * this.weights.tilt * tiltHess[2][1];
      lxx[8][8] += 2 * this.weights.tilt * tiltHess[2][2];
    }

    lx[10] = 2 * this.weights.yawRate * state.angularVelocity.x;
    lx[11] = 2 * this.weights.yawRate * state.angularVelocity.y;
    lx[12] = 2 * this.weights.yawRate * state.angularVelocity.z;
    lxx[10][10] = 2 * this.weights.yawRate;
    lxx[11][11] = 2 * this.weights.yawRate;
    lxx[12][12] = 2 * this.weights.yawRate;

    if (!isTerminal) {
      const hoverThrust = this.mass * this.gravity;
      const duThrust = u[0] - hoverThrust;
      lu[0] = 2 * this.weights.thrust * duThrust;
      luu[0][0] = 2 * this.weights.thrust;

      lu[1] = 2 * this.weights.torque * u[1];
      lu[2] = 2 * this.weights.torque * u[2];
      lu[3] = 2 * this.weights.torque * u[3];
      luu[1][1] = 2 * this.weights.torque;
      luu[2][2] = 2 * this.weights.torque;
      luu[3][3] = 2 * this.weights.torque;

      if (uPrev) {
        const duPrev = u.map((val, idx) => val - uPrev[idx]);
        for (let i = 0; i < CONTROL_SIZE; i += 1) {
          lu[i] += 2 * this.weights.smooth * duPrev[i];
          luu[i][i] += 2 * this.weights.smooth;
        }
      }
    } else {
      lx[0] *= this.weights.terminalPosition / this.weights.position;
      lx[1] *= this.weights.terminalPosition / this.weights.position;
      lx[2] *= this.weights.terminalPosition / this.weights.position;
      lx[3] *= this.weights.terminalVelocity / this.weights.velocity;
      lx[4] *= this.weights.terminalVelocity / this.weights.velocity;
      lx[5] *= this.weights.terminalVelocity / this.weights.velocity;
      lxx[0][0] = 2 * this.weights.terminalPosition;
      lxx[1][1] = 2 * this.weights.terminalPosition;
      lxx[2][2] = 2 * this.weights.terminalPosition;
      lxx[3][3] = 2 * this.weights.terminalVelocity;
      lxx[4][4] = 2 * this.weights.terminalVelocity;
      lxx[5][5] = 2 * this.weights.terminalVelocity;
    }

    return { lx, lu, lxx, luu, lux };
  }

  _terminalGrad(x, ref) {
    return this._costDerivatives(x, [0, 0, 0, 0], ref, true).lx;
  }

  _terminalHess(x, ref) {
    return this._costDerivatives(x, [0, 0, 0, 0], ref, true).lxx;
  }

  _integrate(x, u, dt) {
    const state = this._vectorToState(x);
    const q = state.orientationQuat.clone().normalize();
    const omega = state.angularVelocity.clone();
    const thrustDir = new THREE.Vector3(0, 0, 1).applyQuaternion(q);

    const pDot = state.velocity.clone();
    const vDot = new THREE.Vector3(0, 0, -this.gravity).add(thrustDir.multiplyScalar(u[0] / this.mass));

    const omegaQuat = new THREE.Quaternion(omega.x, omega.y, omega.z, 0);
    const qDot = new THREE.Quaternion().multiplyQuaternions(omegaQuat, q);
    qDot.x *= 0.5; qDot.y *= 0.5; qDot.z *= 0.5; qDot.w *= 0.5;

    const J = this.inertia;
    const coriolis = omega.clone().cross(new THREE.Vector3(J.x * omega.x, J.y * omega.y, J.z * omega.z));
    const tau = new THREE.Vector3(u[1], u[2], u[3]);
    const omegaDot = new THREE.Vector3(
      (tau.x - coriolis.x) / J.x,
      (tau.y - coriolis.y) / J.y,
      (tau.z - coriolis.z) / J.z,
    );

    const pNext = state.position.clone().add(pDot.multiplyScalar(dt));
    const vNext = state.velocity.clone().add(vDot.multiplyScalar(dt));
    const qNext = q.clone();
    qNext.x += qDot.x * dt; qNext.y += qDot.y * dt; qNext.z += qDot.z * dt; qNext.w += qDot.w * dt;
    qNext.normalize();
    const omegaNext = omega.clone().add(omegaDot.multiplyScalar(dt));

    return [pNext.x, pNext.y, pNext.z, vNext.x, vNext.y, vNext.z, qNext.x, qNext.y, qNext.z, qNext.w, omegaNext.x, omegaNext.y, omegaNext.z];
  }

  _linearize(x, u) {
    const dt = this.predictionDt;
    const state = this._vectorToState(x);
    const q = state.orientationQuat.clone().normalize();
    const omega = state.angularVelocity.clone();
    const R = new THREE.Matrix3().setFromMatrix4(new THREE.Matrix4().makeRotationFromQuaternion(q));
    const Re3 = new THREE.Vector3(0, 0, 1).applyMatrix3(R);

    const Ix = this.inertia.x;
    const Iy = this.inertia.y;
    const Iz = this.inertia.z;

    const A = Array.from({ length: STATE_SIZE }, (_, i) => new Array(STATE_SIZE).fill(0));
    const B = Array.from({ length: STATE_SIZE }, () => new Array(CONTROL_SIZE).fill(0));

    for (let i = 0; i < STATE_SIZE; i += 1) A[i][i] = 1; // identity for Euler discretisation

    A[0][3] = dt; A[1][4] = dt; A[2][5] = dt;

    const dRe3_dqvec = this._dRe3dq(R);
    const thrustCoeff = (u[0] / this.mass) * dt;
    for (let row = 0; row < 3; row += 1) {
      B[row + 3][0] = Re3.getComponent(row) * dt / this.mass;
      A[row + 3][6] += thrustCoeff * dRe3_dqvec[row][0];
      A[row + 3][7] += thrustCoeff * dRe3_dqvec[row][1];
      A[row + 3][8] += thrustCoeff * dRe3_dqvec[row][2];
    }

    const wx = omega.x; const wy = omega.y; const wz = omega.z;
    const Omega = [
      [0, -wx, -wy, -wz],
      [wx, 0, wz, -wy],
      [wy, -wz, 0, wx],
      [wz, wy, -wx, 0],
    ];
    const Aqq = Omega.map((row) => row.map((v) => 0.5 * v * dt));
    for (let i = 0; i < 4; i += 1) {
      for (let j = 0; j < 4; j += 1) {
        A[i + 6][j + 6] += Aqq[i][j];
      }
    }

    const Omega_x = [
      [0, -1, 0, 0],
      [1, 0, 0, 0],
      [0, 0, 0, 1],
      [0, 0, -1, 0],
    ];
    const Omega_y = [
      [0, 0, -1, 0],
      [0, 0, 0, -1],
      [1, 0, 0, 0],
      [0, 1, 0, 0],
    ];
    const Omega_z = [
      [0, 0, 0, -1],
      [0, 0, 1, 0],
      [0, -1, 0, 0],
      [1, 0, 0, 0],
    ];
    const qVec = [q.x, q.y, q.z, q.w];
    const BqOmega = [matVecMul(Omega_x, qVec), matVecMul(Omega_y, qVec), matVecMul(Omega_z, qVec)];
    for (let j = 0; j < 3; j += 1) {
      for (let i = 0; i < 4; i += 1) {
        A[6 + i][10 + j] += 0.5 * dt * BqOmega[j][i];
      }
    }

    A[10][11] += -((Iy - Iz) * wz / Ix) * dt;
    A[10][12] += -((Iy - Iz) * wy / Ix) * dt;
    A[11][10] += -((Iz - Ix) * wz / Iy) * dt;
    A[11][12] += -((Iz - Ix) * wx / Iy) * dt;
    A[12][10] += -((Ix - Iy) * wy / Iz) * dt;
    A[12][11] += -((Ix - Iy) * wx / Iz) * dt;

    B[10][1] = dt / Ix;
    B[11][2] = dt / Iy;
    B[12][3] = dt / Iz;

    return { A, B };
  }

  _dRe3dq(R) {
    const e3 = new THREE.Vector3(0, 0, 1);
    const crossE3 = crossMatrix(e3);
    const Rm = R.elements ? [
      [R.elements[0], R.elements[3], R.elements[6]],
      [R.elements[1], R.elements[4], R.elements[7]],
      [R.elements[2], R.elements[5], R.elements[8]],
    ] : [[R[0][0], R[0][1], R[0][2]], [R[1][0], R[1][1], R[1][2]], [R[2][0], R[2][1], R[2][2]]];
    const Re3Cross = matMul(Rm, crossE3);
    return Re3Cross.map((row) => row.map((v) => -2 * v));
  }

  _transpose(M) {
    return M[0].map((_, i) => M.map((row) => row[i]));
  }

  _addMatrices(A, B) {
    const rows = A.length; const cols = A[0].length;
    const C = Array.from({ length: rows }, () => new Array(cols).fill(0));
    for (let i = 0; i < rows; i += 1) {
      for (let j = 0; j < cols; j += 1) {
        C[i][j] = A[i][j] + B[i][j];
      }
    }
    return C;
  }

  _invertSymmetric(M) {
    const n = M.length;
    const A = M.map((row) => row.slice());
    const I = Array.from({ length: n }, (_, i) => new Array(n).fill(0).map((_, j) => (i === j ? 1 : 0)));
    for (let i = 0; i < n; i += 1) {
      let pivot = A[i][i];
      if (Math.abs(pivot) < 1e-9) pivot = 1e-9;
      const invPivot = 1 / pivot;
      for (let j = 0; j < n; j += 1) { A[i][j] *= invPivot; I[i][j] *= invPivot; }
      for (let k = 0; k < n; k += 1) {
        if (k === i) continue;
        const factor = A[k][i];
        for (let j = 0; j < n; j += 1) {
          A[k][j] -= factor * A[i][j];
          I[k][j] -= factor * I[i][j];
        }
      }
    }
    return I;
  }

  _symmetrize(M) {
    const n = M.length;
    const out = Array.from({ length: n }, () => new Array(n).fill(0));
    for (let i = 0; i < n; i += 1) {
      for (let j = 0; j < n; j += 1) {
        out[i][j] = 0.5 * (M[i][j] + M[j][i]);
      }
    }
    return out;
  }
}

const scaleVecMatrix = (M, s) => M.map((row) => row.map((v) => v * s));
