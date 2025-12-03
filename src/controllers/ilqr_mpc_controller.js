// ILQRMPCController.js
//
// iLQR-based MPC controller for Crazyflie-style quadrotor,
// fully consistent with EthController conventions.
//
// Frames & layout (MUST MATCH physics + EthController):
// - World:  +Z up, +X forward, +Y left
// - Body:   +X forward, +Y left, +Z up (thrust direction)
// - Orientation: quaternion mapping body -> world (THREE.Quaternion)
// - Motors (X configuration):
//     M0 = Front-Left  (CW)  at (+X, +Y)
//     M1 = Front-Right (CCW) at (+X, -Y)
//     M2 = Back-Right  (CW)  at (-X, -Y)
//     M3 = Back-Left   (CCW) at (-X, +Y)
//
// This controller optimises directly over total thrust + body torques
// using iLQR on the same rigid-body model that EthController implicitly uses.
// It then uses the *same* motor mixing as EthController to get per-motor commands.

import { EthController } from '../../drones/stationary/eth_controller.js';

const STATE_SIZE = 13; // [p(3) v(3) q(4) omega(3)]
const CONTROL_SIZE = 4; // [thrust, tau_x, tau_y, tau_z]

const ZERO_VEC3 = () => new THREE.Vector3(0, 0, 0);

// ---------- small linear algebra helpers ----------

const matMul = (A, B) => {
  const rows = A.length;
  const cols = B[0].length;
  const inner = B.length;
  const out = Array.from({ length: rows }, () => new Array(cols).fill(0));
  for (let i = 0; i < rows; i += 1) {
    const Ai = A[i];
    const Oi = out[i];
    for (let k = 0; k < inner; k += 1) {
      const aik = Ai[k];
      const Bk = B[k];
      for (let j = 0; j < cols; j += 1) {
        Oi[j] += aik * Bk[j];
      }
    }
  }
  return out;
};

const matVecMul = (A, v) => {
  const n = A.length;
  const out = new Array(n).fill(0);
  for (let i = 0; i < n; i += 1) {
    const row = A[i];
    let s = 0;
    for (let j = 0; j < row.length; j += 1) s += row[j] * v[j];
    out[i] = s;
  }
  return out;
};

const addVecs = (a, b) => a.map((x, i) => x + b[i]);
const subVecs = (a, b) => a.map((x, i) => x - b[i]);
const scaleVec = (v, s) => v.map((x) => x * s);

const transpose = (M) =>
  M[0].map((_, col) => M.map((row) => row[col]));

const addMatrices = (A, B) => {
  const rows = A.length;
  const cols = A[0].length;
  const C = Array.from({ length: rows }, () => new Array(cols).fill(0));
  for (let i = 0; i < rows; i += 1) {
    for (let j = 0; j < cols; j += 1) {
      C[i][j] = A[i][j] + B[i][j];
    }
  }
  return C;
};

const scaleMatrix = (M, s) =>
  M.map((row) => row.map((v) => v * s));

// Simple symmetric matrix inversion via Gauss–Jordan
const invertSymmetric = (M) => {
  const n = M.length;
  const A = M.map((row) => row.slice());
  const I = Array.from({ length: n }, (_, i) =>
    Array.from({ length: n }, (__ , j) => (i === j ? 1 : 0)),
  );

  for (let i = 0; i < n; i += 1) {
    let pivot = A[i][i];
    if (Math.abs(pivot) < 1e-9) pivot = 1e-9;
    const invP = 1 / pivot;
    for (let j = 0; j < n; j += 1) {
      A[i][j] *= invP;
      I[i][j] *= invP;
    }
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
};

const symmetrize = (M) => {
  const n = M.length;
  const out = Array.from({ length: n }, () => new Array(n).fill(0));
  for (let i = 0; i < n; i += 1) {
    for (let j = 0; j < n; j += 1) {
      out[i][j] = 0.5 * (M[i][j] + M[j][i]);
    }
  }
  return out;
};

// ---------- MPC controller ----------

export class ILQRMPCController extends EthController {
  constructor(params = {}) {
    super(params); // sets mass, inertia, motor params, mixing conventions

    // Horizon & solver settings
    this.horizonSteps = params.mpcHorizon || 40;     // N
    this.predictionDt = params.predictionDt || 0.01; // Δt used inside MPC
    this.maxIterations = params.maxIterations || 15;


    // Line search for iLQR
    this.lineSearch = params.lineSearch || [1.0, 0.7, 0.5, 0.3, 0.1, 0.05];

    // Cost weights (tuned for Crazyflie-ish dynamics)
    this.weights = {
      position: params.wPosition || 40.0,   // stage pos cost
      velocity: params.wVelocity || 10.0,
      attitude: params.wAttitude || 1.5,    // small penalty on roll/pitch
      angVel: params.wAngVel || 0.1,
      thrust: params.wThrust || 1e-3,
      torque: params.wTorque || 5e-4,
      smooth: params.wSmooth || 2e-3,

      terminalPosition: params.wTerminalPosition || 80.0,
      terminalVelocity: params.wTerminalVelocity || 25.0,
    };

    const T_max = params.maxThrust || 4 * (params.maxThrustPerMotor || this.maxThrustPerMotor);
    this.constraints = {
      minThrust: params.minThrust || 0,
      maxThrust: T_max,
      maxTorque: params.maxTorque || 0.02,       // roll/pitch
      maxYawTorque: params.maxYawTorque || 0.01, // yaw
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

  // Main entry point: same interface as EthController.computeControl,
  // but returns extra diagnostics (MPC trajectory, cost, solve time).
  computeControl(state, target, dt) {
    const startTime = performance.now();

    const x0 = this._stateToVector(state);
    const ref = this._buildReference(target); // constant ref over horizon

    // 1) initial control sequence (warm start)
    let uSeq = this._warmStart();
    let nominal = this._rollout(x0, uSeq, ref);
    let bestCost = nominal.cost;

    // 2) iLQR iterations
    for (let iter = 0; iter < this.maxIterations; iter += 1) {
      const { A, B } = this._linearizeTrajectory(nominal.states, uSeq);
      const { kSeq, KSeq } = this._backwardPass(A, B, nominal.states, uSeq, ref);

      let accepted = false;
      for (const alpha of this.lineSearch) {
        const candidate = this._forwardPass(x0, uSeq, nominal.states, kSeq, KSeq, alpha, ref);
        if (candidate.cost < bestCost) {
          uSeq = candidate.controls;
          nominal = candidate;
          bestCost = candidate.cost;
          accepted = true;
          break;
        }
      }

      if (!accepted) {
        // No improving step — stop early
        break;
      }

      if (Math.abs(this.lastCost - bestCost) < 1e-4) break;
      this.lastCost = bestCost;
    }

    // 3) Save solution and build output
    this.previousSolution = { controls: uSeq, states: nominal.states, cost: bestCost };
    this.predictedTrajectory = nominal.states.map((x) => this._vectorToState(x));
    this.lastOptimizationTime = performance.now() - startTime;

    const u0 = uSeq[0];
    const [thrust, tauX, tauY, tauZ] = this._clampControl(u0);
    const motorCommands = this._mixToMotors(thrust, tauX, tauY, tauZ);

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
  // Helpers: state, reference, controls
  // ---------------------------------------------------------------------------

  _stateToVector(state) {
    const v = new Array(STATE_SIZE).fill(0);
    v[0] = state.position.x;
    v[1] = state.position.y;
    v[2] = state.position.z;

    v[3] = state.velocity.x;
    v[4] = state.velocity.y;
    v[5] = state.velocity.z;

    v[6] = state.orientationQuat.x;
    v[7] = state.orientationQuat.y;
    v[8] = state.orientationQuat.z;
    v[9] = state.orientationQuat.w;

    v[10] = state.angularVelocity.x;
    v[11] = state.angularVelocity.y;
    v[12] = state.angularVelocity.z;
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

  // Constant reference over the horizon: just "hover at target"
  _buildReference(target) {
    const pRef = (target?.position || ZERO_VEC3()).clone();
    const N = this.horizonSteps;

    const ref = [];
    for (let k = 0; k <= N; k += 1) {
      ref.push({
        position: pRef,
        velocity: ZERO_VEC3(),
      });
    }
    return ref;
  }

  _warmStart() {
    const hoverThrust = this.mass * this.gravity;
    const N = this.horizonSteps;

    if (this.previousSolution?.controls?.length === N) {
      // Shift previous solution one step forward
      const shifted = this.previousSolution.controls.slice(1);
      shifted.push([...shifted[shifted.length - 1]]);
      return shifted;
    }

    // New sequence: pure hover thrust, zero torques
    return Array.from({ length: N }, () => [hoverThrust, 0, 0, 0]);
  }

  _clampControl(u) {
    const thrust = THREE.MathUtils.clamp(
      u[0],
      this.constraints.minThrust,
      this.constraints.maxThrust,
    );
    const tauX = THREE.MathUtils.clamp(
      u[1],
      -this.constraints.maxTorque,
      this.constraints.maxTorque,
    );
    const tauY = THREE.MathUtils.clamp(
      u[2],
      -this.constraints.maxTorque,
      this.constraints.maxTorque,
    );
    const tauZ = THREE.MathUtils.clamp(
      u[3],
      -this.constraints.maxYawTorque,
      this.constraints.maxYawTorque,
    );
    return [thrust, tauX, tauY, tauZ];
  }

  // EXACTLY the same mixing as EthController.computeControl (copied here)
  _mixToMotors(thrust, tauX, tauY, tauZ) {
    const L = this.L;
    const kF = this.kF;
    const kM = this.kM;
    const T_max = this.maxThrustPerMotor;

    const solveThrusts = (yawTorque) => {
      const S  = thrust / kF;
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
        T0 *= scale;
        T1 *= scale;
        T2 *= scale;
        T3 *= scale;
      }
    }

    T0 = Math.min(Math.max(T0, 0), T_max);
    T1 = Math.min(Math.max(T1, 0), T_max);
    T2 = Math.min(Math.max(T2, 0), T_max);
    T3 = Math.min(Math.max(T3, 0), T_max);

    const omegaRange = this.omegaMax - this.omegaMin;
    const thrustToCmd = (T) => {
      const omega = Math.sqrt(Math.max(T, 0) / this.kF);
      const u = (omega - this.omegaMin) / omegaRange;
      return THREE.MathUtils.clamp(u, 0, 1);
    };

    return [thrustToCmd(T0), thrustToCmd(T1), thrustToCmd(T2), thrustToCmd(T3)];
  }

  // ---------------------------------------------------------------------------
  // Dynamics: same rigid-body model as the simulator
  // ---------------------------------------------------------------------------

  _integrate(x, u, dt) {
    const state = this._vectorToState(x);
    const q = state.orientationQuat.clone().normalize();
    const omega = state.angularVelocity.clone();

    const thrustDir = new THREE.Vector3(0, 0, 1).applyQuaternion(q);

    const pDot = state.velocity.clone();
    const vDot = new THREE.Vector3(0, 0, -this.gravity)
      .add(thrustDir.multiplyScalar(u[0] / this.mass));

    const omegaQuat = new THREE.Quaternion(omega.x, omega.y, omega.z, 0);
    const qDot = new THREE.Quaternion().multiplyQuaternions(omegaQuat, q);
    qDot.x *= 0.5; qDot.y *= 0.5; qDot.z *= 0.5; qDot.w *= 0.5;

    const J = this.inertia;
    const Iomega = new THREE.Vector3(
      J.x * omega.x,
      J.y * omega.y,
      J.z * omega.z,
    );
    const coriolis = omega.clone().cross(Iomega);
    const tau = new THREE.Vector3(u[1], u[2], u[3]);

    const omegaDot = new THREE.Vector3(
      (tau.x - coriolis.x) / J.x,
      (tau.y - coriolis.y) / J.y,
      (tau.z - coriolis.z) / J.z,
    );

    const pNext = state.position.clone().add(pDot.multiplyScalar(dt));
    const vNext = state.velocity.clone().add(vDot.multiplyScalar(dt));
    const qNext = q.clone();
    qNext.x += qDot.x * dt;
    qNext.y += qDot.y * dt;
    qNext.z += qDot.z * dt;
    qNext.w += qDot.w * dt;
    qNext.normalize();
    const omegaNext = omega.clone().add(omegaDot.multiplyScalar(dt));

    return [
      pNext.x, pNext.y, pNext.z,
      vNext.x, vNext.y, vNext.z,
      qNext.x, qNext.y, qNext.z, qNext.w,
      omegaNext.x, omegaNext.y, omegaNext.z,
    ];
  }

  // ---------------------------------------------------------------------------
  // iLQR core (with *numerical* linearisation – the key robustness fix)
  // ---------------------------------------------------------------------------

  _rollout(x0, uSeq, ref) {
    const N = uSeq.length;
    const dt = this.predictionDt;

    const states = [x0.slice()];
    let cost = 0;

    for (let k = 0; k < N; k += 1) {
      const xk = states[states.length - 1];
      const uk = this._clampControl(uSeq[k]);
      const refStep = ref[k] || ref[ref.length - 1];

      cost += this._stageCost(xk, uk, refStep, k, k > 0 ? uSeq[k - 1] : null);

      const xNext = this._integrate(xk, uk, dt);
      states.push(xNext);
    }

    // Terminal cost
    cost += this._terminalCost(states[states.length - 1], ref[ref.length - 1]);

    return { states, controls: uSeq.map((u) => u.slice()), cost };
  }

  _forwardPass(x0, uNom, xNom, kSeq, KSeq, alpha, ref) {
    const N = uNom.length;
    const dt = this.predictionDt;

    const states = [x0.slice()];
    const controls = [];
    let cost = 0;

    for (let k = 0; k < N; k += 1) {
      const xk = states[k];
      const dx = subVecs(xk, xNom[k]);

      // u_k = u_nom_k + α k_k + K_k (x_k - x_nom_k)
      const du_ff = scaleVec(kSeq[k], alpha);
      const du_fb = matVecMul(KSeq[k], dx);
      const uk_unclamped = addVecs(uNom[k], addVecs(du_ff, du_fb));
      const uk = this._clampControl(uk_unclamped);
      controls.push(uk);

      const refStep = ref[k] || ref[ref.length - 1];
      cost += this._stageCost(xk, uk, refStep, k, k > 0 ? controls[k - 1] : null);

      const xNext = this._integrate(xk, uk, dt);
      states.push(xNext);
    }

    cost += this._terminalCost(states[states.length - 1], ref[ref.length - 1]);

    return { states, controls, cost };
  }

  // Numerical linearisation of dynamics along trajectory
  _linearizeTrajectory(states, controls) {
    const N = controls.length;
    const A = new Array(N);
    const B = new Array(N);

    for (let k = 0; k < N; k += 1) {
      const xk = states[k];
      const uk = controls[k];
      const { A: Ak, B: Bk } = this._linearizeFD(xk, uk);
      A[k] = Ak;
      B[k] = Bk;
    }

    return { A, B };
  }

  // Finite-difference Jacobians: A = ∂f/∂x, B = ∂f/∂u
  _linearizeFD(x, u) {
    const n = STATE_SIZE;
    const m = CONTROL_SIZE;
    const dt = this.predictionDt;

    const f0 = this._integrate(x, u, dt);

    const A = Array.from({ length: n }, () => new Array(n).fill(0));
    const B = Array.from({ length: n }, () => new Array(m).fill(0));

    const epsX = 1e-4;
    const epsU = 1e-4;

    // ∂f/∂x
    for (let i = 0; i < n; i += 1) {
      const xp = x.slice();
      const xm = x.slice();
      xp[i] += epsX;
      xm[i] -= epsX;
      const fp = this._integrate(xp, u, dt);
      const fm = this._integrate(xm, u, dt);
      const inv2e = 1 / (2 * epsX);
      for (let j = 0; j < n; j += 1) {
        A[j][i] = (fp[j] - fm[j]) * inv2e;
      }
    }

    // ∂f/∂u
    for (let i = 0; i < m; i += 1) {
      const up = u.slice();
      const um = u.slice();
      up[i] += epsU;
      um[i] -= epsU;
      const fp = this._integrate(x, up, dt);
      const fm = this._integrate(x, um, dt);
      const inv2e = 1 / (2 * epsU);
      for (let j = 0; j < n; j += 1) {
        B[j][i] = (fp[j] - fm[j]) * inv2e;
      }
    }

    return { A, B };
  }

  _backwardPass(A, B, xNom, uNom, ref) {
    const N = uNom.length;
    const kSeq = new Array(N);
    const KSeq = new Array(N);

    // Terminal value function
    let Vx = this._terminalGrad(xNom[N], ref[N]);
    let Vxx = this._terminalHess(xNom[N], ref[N]);

    const reg = 1e-6;

    for (let k = N - 1; k >= 0; k -= 1) {
      const Ak = A[k];
      const Bk = B[k];
      const refStep = ref[k];

      const { lx, lu, lxx, luu, lux } = this._costDerivatives(
        xNom[k],
        uNom[k],
        refStep,
        k === N - 1,
        k > 0 ? uNom[k - 1] : null,
      );

      const AT = transpose(Ak);
      const BT = transpose(Bk);

      const AT_Vx = matVecMul(AT, Vx);
      const BT_Vx = matVecMul(BT, Vx);

      const Qx = addVecs(lx, AT_Vx);
      const Qu = addVecs(lu, BT_Vx);

      const At_Vxx = matMul(AT, Vxx);
      const Bt_Vxx = matMul(BT, Vxx);

      const Qxx = addMatrices(lxx, matMul(At_Vxx, Ak));
      const Quu = addMatrices(luu, matMul(Bt_Vxx, Bk));
      const Qux = addMatrices(lux, matMul(Bt_Vxx, Ak));

      // Regularise Quu
      for (let i = 0; i < CONTROL_SIZE; i += 1) {
        Quu[i][i] += reg;
      }

      const QuuInv = invertSymmetric(Quu);
      const kFeedforward = scaleVec(matVecMul(QuuInv, Qu), -1);
      const KFeedback = matMul(scaleMatrix(QuuInv, -1), Qux);

      kSeq[k] = kFeedforward;
      KSeq[k] = KFeedback;

      // Update value function
      const KT = transpose(KFeedback);
      const KT_Qu = matVecMul(KT, Qu);
      const KT_Quu = matMul(KT, Quu);
      const KT_Quu_K = matMul(KT_Quu, KFeedback);
      const KT_Qux = matMul(KT, Qux);

      const VxNew = addVecs(Qx, KT_Qu);
      const VxxNew = addMatrices(
        Qxx,
        addMatrices(KT_Qux, KT_Quu_K),
      );

      Vx = VxNew;
      Vxx = symmetrize(VxxNew);
    }

    return { kSeq, KSeq };
  }

  // ---------------------------------------------------------------------------
  // Cost function & derivatives (no tricky quaternion tilt derivatives)
  // ---------------------------------------------------------------------------

  _stageCost(x, u, ref, k, uPrev) {
    const { position, velocity, orientationQuat, angularVelocity } = this._vectorToState(x);

    const pref = ref?.position || ZERO_VEC3();

    const posErr = position.clone().sub(pref);
    const velErr = velocity.clone(); // target velocity = 0

    const q = orientationQuat;
    // Penalise roll/pitch (q.x, q.y) softly, leave yaw almost free
    const attErrSq = q.x * q.x + q.y * q.y;

    const omega = angularVelocity;
    const omegaSq = omega.x * omega.x + omega.y * omega.y + omega.z * omega.z;

    const hoverThrust = this.mass * this.gravity;

    let cost = 0;
    cost += this.weights.position * posErr.lengthSq();
    cost += this.weights.velocity * velErr.lengthSq();
    cost += this.weights.attitude * attErrSq;
    cost += this.weights.angVel * omegaSq;

    // Control effort
    const duThrust = u[0] - hoverThrust;
    cost += this.weights.thrust * duThrust * duThrust;
    cost += this.weights.torque * (u[1] * u[1] + u[2] * u[2] + u[3] * u[3]);

    // Smoothing (Δu)
    if (uPrev) {
      const du0 = u[0] - uPrev[0];
      const du1 = u[1] - uPrev[1];
      const du2 = u[2] - uPrev[2];
      const du3 = u[3] - uPrev[3];
      const duSq = du0 * du0 + du1 * du1 + du2 * du2 + du3 * du3;
      cost += this.weights.smooth * duSq;
    }

    return cost;
  }

  _terminalCost(x, ref) {
    const { position, velocity } = this._vectorToState(x);
    const pref = ref?.position || ZERO_VEC3();
    const posErr = position.clone().sub(pref);
    const velErr = velocity.clone();

    let cost = 0;
    cost += this.weights.terminalPosition * posErr.lengthSq();
    cost += this.weights.terminalVelocity * velErr.lengthSq();
    return cost;
  }

  _costDerivatives(x, u, ref, isTerminal, uPrev) {
    const n = STATE_SIZE;
    const m = CONTROL_SIZE;

    const lx = new Array(n).fill(0);
    const lu = new Array(m).fill(0);
    const lxx = Array.from({ length: n }, () => new Array(n).fill(0));
    const luu = Array.from({ length: m }, () => new Array(m).fill(0));
    const lux = Array.from({ length: m }, () => new Array(n).fill(0)); // not used, keep zero

    const { position, velocity, orientationQuat, angularVelocity } = this._vectorToState(x);
    const pref = ref?.position || ZERO_VEC3();

    const posErr = position.clone().sub(pref);
    const velErr = velocity.clone();

    const wPos = isTerminal ? this.weights.terminalPosition : this.weights.position;
    const wVel = isTerminal ? this.weights.terminalVelocity : this.weights.velocity;

    // Position
    lx[0] = 2 * wPos * posErr.x;
    lx[1] = 2 * wPos * posErr.y;
    lx[2] = 2 * wPos * posErr.z;
    lxx[0][0] = 2 * wPos;
    lxx[1][1] = 2 * wPos;
    lxx[2][2] = 2 * wPos;

    // Velocity
    lx[3] = 2 * wVel * velErr.x;
    lx[4] = 2 * wVel * velErr.y;
    lx[5] = 2 * wVel * velErr.z;
    lxx[3][3] = 2 * wVel;
    lxx[4][4] = 2 * wVel;
    lxx[5][5] = 2 * wVel;

    if (!isTerminal) {
      // Attitude (roll/pitch)
      const q = orientationQuat;
      const wAtt = this.weights.attitude;
      lx[6] = 2 * wAtt * q.x;
      lx[7] = 2 * wAtt * q.y;
      lxx[6][6] = 2 * wAtt;
      lxx[7][7] = 2 * wAtt;

      // Angular velocity
      const wOmega = this.weights.angVel;
      const omega = angularVelocity;
      lx[10] = 2 * wOmega * omega.x;
      lx[11] = 2 * wOmega * omega.y;
      lx[12] = 2 * wOmega * omega.z;
      lxx[10][10] = 2 * wOmega;
      lxx[11][11] = 2 * wOmega;
      lxx[12][12] = 2 * wOmega;

      // Control effort
      const hoverThrust = this.mass * this.gravity;
      const duThrust = u[0] - hoverThrust;
      const wT = this.weights.thrust;
      const wTau = this.weights.torque;

      lu[0] = 2 * wT * duThrust;
      luu[0][0] = 2 * wT;

      lu[1] = 2 * wTau * u[1];
      lu[2] = 2 * wTau * u[2];
      lu[3] = 2 * wTau * u[3];
      luu[1][1] = 2 * wTau;
      luu[2][2] = 2 * wTau;
      luu[3][3] = 2 * wTau;

      // Smoothing
      if (uPrev) {
        const wS = this.weights.smooth;
        for (let i = 0; i < m; i += 1) {
          const duPrev = u[i] - uPrev[i];
          lu[i] += 2 * wS * duPrev;
          luu[i][i] += 2 * wS;
        }
      }
    }

    return { lx, lu, lxx, luu, lux };
  }

  _terminalGrad(x, ref) {
    return this._costDerivatives(x, [0, 0, 0, 0], ref, true, null).lx;
  }

  _terminalHess(x, ref) {
    return this._costDerivatives(x, [0, 0, 0, 0], ref, true, null).lxx;
  }
}
