/**
 * Model Predictive Controller (SE(3)) for quadrotor racing.
 *
 * Implements a receding-horizon controller that predicts full rigid-body
 * dynamics (position, velocity, orientation quaternion, angular velocity)
 * using the same conventions as the ETH geometric controller and the
 * physics engine:
 *
 * Frames and sign conventions
 * - World: +Z up.
 * - Body:  +X forward, +Y left, +Z up (thrust direction).
 * - Gravity acts along -Z in world.
 *
 * Motor layout (X-configuration, matches physics & EthController):
 * 0: Front-Left  (CW)  at (+X, +Y)
 * 1: Front-Right (CCW) at (+X, -Y)
 * 2: Back-Right  (CW)  at (-X, -Y)
 * 3: Back-Left   (CCW) at (-X, +Y)
 *
 * The control inputs are collective thrust T (N) along body +Z and body-frame
 * torque τ = [τx, τy, τz]. Allocation to motors uses the exact mixer from
 * EthController to stay consistent with the physics engine.
 */

import { EthController } from '../stationary/eth_controller.js';

/**
 * Full SE(3) state used by the MPC forward simulation.
 */
class MPCState {
  constructor(position, velocity, orientation, angularVelocity) {
    this.position = position.clone();
    this.velocity = velocity.clone();
    this.orientation = orientation.clone(); // body → world quaternion
    this.angularVelocity = angularVelocity.clone(); // body frame
  }

  clone() {
    return new MPCState(this.position, this.velocity, this.orientation, this.angularVelocity);
  }
}

/**
 * Control input: collective thrust + body torques.
 */
class MPCControl {
  constructor(thrust, tauX, tauY, tauZ) {
    this.thrust = thrust;
    this.tau = new THREE.Vector3(tauX, tauY, tauZ);
  }

  clone() {
    return new MPCControl(this.thrust, this.tau.x, this.tau.y, this.tau.z);
  }
}

export class MPCController extends EthController {
  constructor(params) {
    super(params);

    // Trajectory/time bookkeeping (shared with RaceController API)
    this.trajectory = null;
    this.simTime = 0;
    this.gateIndex = 0;

    // Horizon and solver settings
    this.horizonSteps = 15;
    this.predictionDt = 0.05; // seconds per prediction step
    this.maxIterations = 4;

    // Cost weights
    this.weights = {
      position: 10.0,
      velocity: 4.0,
      thrust: 0.1,
      torque: 0.1,
      smooth: 0.05,
      tilt: 0.5,
    };

    // Physical constraints (aligned with EthController clamps)
    this.constraints = {
      minThrust: 0,
      maxThrust: params.maxThrust || 4 * (params.maxThrustPerMotor || this.maxThrustPerMotor),
      maxTorque: 0.02,
      maxTorqueYaw: 0.01,
      maxTiltAngle: params.maxTiltAngle
        ? THREE.MathUtils.degToRad(params.maxTiltAngle)
        : THREE.MathUtils.degToRad(60),
    };

    // Optimization state
    this.previousSolution = null;
    this.predictedTrajectory = [];
    this.lastOptimizationTime = 0;
    this.convergenceHistory = [];
  }

  // --- Public API compatibility helpers ----------------------------------------------------

  setTrajectory(trajectory) {
    this.trajectory = trajectory;
  }

  reset() {
    super.reset();
    this.simTime = 0;
    this.gateIndex = 0;
    this.previousSolution = null;
    this.predictedTrajectory = [];
    this.lastOptimizationTime = 0;
    this.convergenceHistory = [];
  }

  update(dt) {
    this.simTime += dt;
  }

  getTarget() {
    return this._sampleReferenceAtTime(this.simTime) || {
      position: new THREE.Vector3(0, 0, 0.6),
      velocity: new THREE.Vector3(0, 0, 0),
      acceleration: new THREE.Vector3(0, 0, 0),
      yaw: 0,
    };
  }

  updateGateIndex(position, waypoints, gateThreshold = 1.5) {
    if (!waypoints || waypoints.length < 2) return false;
    if (this.gateIndex >= waypoints.length - 1) return false;

    const nextGatePos = waypoints[this.gateIndex + 1];
    if (position.distanceTo(nextGatePos) < gateThreshold) {
      this.gateIndex += 1;
      return true;
    }
    return false;
  }

  getGateIndex() {
    return this.gateIndex;
  }

  // --- MPC core ---------------------------------------------------------------------------

  computeControl(state, target, dt) {
    const start = performance.now();
    this.simTime += dt;

    const refHorizon = this._buildReferenceHorizon(target);
    const solution = this._solveMPC(state, refHorizon);

    // First control of horizon
    const controlAction = solution.controls[0];
    const motorCommands = this._allocateMotorsFromThrustAndTorque(controlAction.thrust, controlAction.tau);

    this.previousSolution = solution;
    this.predictedTrajectory = solution.predictedStates;
    this.lastOptimizationTime = performance.now() - start;

    const desiredOrientation = solution.predictedStates[0]?.orientation.clone()
      || state.orientationQuat?.clone()
      || state.orientation?.clone()
      || new THREE.Quaternion();

    return {
      motorCommands,
      desiredThrust: controlAction.thrust,
      desiredOrientation,
      predictedTrajectory: this.predictedTrajectory,
      optimizationTime: this.lastOptimizationTime,
      cost: solution.cost,
      gateIndex: this.gateIndex,
    };
  }

  _sampleReferenceAtTime(t) {
    if (!this.trajectory) return null;

    if (typeof this.trajectory.getStateAtTime === 'function') {
      const total = this.trajectory.totalDuration || this.trajectory.totalTime || t;
      return this.trajectory.getStateAtTime(total > 0 ? t % total : t);
    }
    if (typeof this.trajectory.sample === 'function') {
      const total = this.trajectory.totalTime || t;
      return this.trajectory.sample(total > 0 ? t % total : t);
    }
    return null;
  }

  _buildReferenceHorizon(fallbackTarget) {
    const horizon = [];
    for (let i = 0; i < this.horizonSteps; i++) {
      const t = this.simTime + i * this.predictionDt;
      const ref = this._sampleReferenceAtTime(t) || fallbackTarget;

      const position = ref?.position?.clone?.() || new THREE.Vector3(0, 0, 0.6);
      const velocity = ref?.velocity?.clone?.() || new THREE.Vector3(0, 0, 0);
      const acceleration = ref?.acceleration?.clone?.() || new THREE.Vector3(0, 0, 0);
      const yaw = ref?.yaw || 0;

      horizon.push({ position, velocity, acceleration, yaw });
    }
    return horizon;
  }

  _solveMPC(currentState, refHorizon) {
    let controls = this._initializeControls();
    let bestCost = Infinity;
    let bestControls = controls;
    let bestStates = [];

    for (let iter = 0; iter < this.maxIterations; iter++) {
      const predictedStates = this._simulateDynamics(currentState, controls);
      const cost = this._evaluateCost(predictedStates, controls, refHorizon);

      if (cost < bestCost) {
        bestCost = cost;
        bestControls = controls.map(c => c.clone());
        bestStates = predictedStates.map(s => s.clone());
      }

      controls = this._updateControls(controls, predictedStates, refHorizon, currentState);
      if (iter > 0 && Math.abs(cost - bestCost) < 1e-3) {
        break;
      }
    }

    this.convergenceHistory.push(bestCost);
    if (this.convergenceHistory.length > 200) this.convergenceHistory.shift();

    return { controls: bestControls, predictedStates: bestStates, cost: bestCost };
  }

  _initializeControls() {
    const controls = [];
    const hoverT = this.mass * this.gravity;
    for (let i = 0; i < this.horizonSteps; i++) {
      if (this.previousSolution && i < this.previousSolution.controls.length - 1) {
        controls.push(this.previousSolution.controls[i + 1].clone());
      } else {
        controls.push(new MPCControl(hoverT, 0, 0, 0));
      }
    }
    return controls;
  }

  _stepDynamics(state, control, dt) {
    const next = state.clone();

    // Rotation matrix body→world
    const R = new THREE.Matrix3().setFromMatrix4(new THREE.Matrix4().makeRotationFromQuaternion(next.orientation));

    // Translational dynamics
    const thrustWorld = new THREE.Vector3(0, 0, 1).applyMatrix3(R).multiplyScalar(control.thrust / this.mass);
    const gravityWorld = new THREE.Vector3(0, 0, -this.gravity);
    const acc = gravityWorld.add(thrustWorld);

    next.velocity.add(acc.multiplyScalar(dt));
    next.position.add(next.velocity.clone().multiplyScalar(dt));

    // Rotational dynamics (diagonal inertia)
    const omega = next.angularVelocity.clone();
    const J = this.inertia;
    const Jomega = new THREE.Vector3(J.x * omega.x, J.y * omega.y, J.z * omega.z);
    const coriolis = omega.clone().cross(Jomega);
    const tauNet = control.tau.clone().sub(coriolis);
    const omegaDot = new THREE.Vector3(tauNet.x / J.x, tauNet.y / J.y, tauNet.z / J.z);

    next.angularVelocity.add(omegaDot.multiplyScalar(dt));

    // Quaternion kinematics: q̇ = 0.5 q ⊗ [0, ω]
    const q = next.orientation.clone();
    const omegaQuat = new THREE.Quaternion(omega.x, omega.y, omega.z, 0);
    const qDot = q.clone().multiply(omegaQuat);
    q.x += 0.5 * qDot.x * dt;
    q.y += 0.5 * qDot.y * dt;
    q.z += 0.5 * qDot.z * dt;
    q.w += 0.5 * qDot.w * dt;
    q.normalize();
    next.orientation.copy(q);

    return next;
  }

  _simulateDynamics(initialState, controls) {
    const states = [];
    let x = new MPCState(
      initialState.position,
      initialState.velocity,
      initialState.orientationQuat || initialState.orientation,
      initialState.angularVelocity,
    );
    states.push(x.clone());

    for (let i = 0; i < controls.length; i++) {
      x = this._stepDynamics(x, controls[i], this.predictionDt);
      states.push(x.clone());
    }
    return states;
  }

  _evaluateCost(predictedStates, controls, refHorizon) {
    let total = 0;
    const hoverThrust = this.mass * this.gravity;

    for (let i = 0; i < this.horizonSteps; i++) {
      const state = predictedStates[i];
      const ref = refHorizon[i];
      const control = controls[i];

      const posError = state.position.clone().sub(ref.position).lengthSq();
      const velError = state.velocity.clone().sub(ref.velocity).lengthSq();
      total += this.weights.position * posError + this.weights.velocity * velError;

      const thrustError = control.thrust - hoverThrust;
      total += this.weights.thrust * thrustError * thrustError;
      total += this.weights.torque * control.tau.lengthSq();

      if (i > 0) {
        const prev = controls[i - 1];
        const dThrust = (control.thrust - prev.thrust) / this.predictionDt;
        const dTau = control.tau.clone().sub(prev.tau).divideScalar(this.predictionDt);
        total += this.weights.smooth * (dThrust * dThrust + dTau.lengthSq());
      }

      // Tilt penalty (angle between body +Z and world +Z)
      const R = new THREE.Matrix3().setFromMatrix4(new THREE.Matrix4().makeRotationFromQuaternion(state.orientation));
      const bodyZ = new THREE.Vector3(0, 0, 1).applyMatrix3(R);
      const cosTilt = THREE.MathUtils.clamp(bodyZ.dot(new THREE.Vector3(0, 0, 1)), -1, 1);
      const tiltAngle = Math.acos(cosTilt);
      total += this.weights.tilt * tiltAngle * tiltAngle;
    }

    return total;
  }

  _clampControl(control) {
    control.thrust = THREE.MathUtils.clamp(control.thrust, this.constraints.minThrust, this.constraints.maxThrust);
    control.tau.x = THREE.MathUtils.clamp(control.tau.x, -this.constraints.maxTorque, this.constraints.maxTorque);
    control.tau.y = THREE.MathUtils.clamp(control.tau.y, -this.constraints.maxTorque, this.constraints.maxTorque);
    control.tau.z = THREE.MathUtils.clamp(control.tau.z, -this.constraints.maxTorqueYaw, this.constraints.maxTorqueYaw);
  }

  _updateControls(controls, predictedStates, refHorizon, initialState) {
    const learningRate = 0.05;
    const epsilon = 0.01;
    const baseCost = this._evaluateCost(predictedStates, controls, refHorizon);

    const newControls = controls.map(c => c.clone());

    for (let i = 0; i < controls.length; i++) {
      const control = newControls[i];

      // Thrust gradient
      {
        const perturbed = controls.map((c, k) => (k === i ? new MPCControl(c.thrust + epsilon, c.tau.x, c.tau.y, c.tau.z) : c));
        const statesPlus = this._simulateDynamics(initialState, perturbed);
        const costPlus = this._evaluateCost(statesPlus, perturbed, refHorizon);
        control.thrust -= learningRate * ((costPlus - baseCost) / epsilon);
      }

      // Roll torque gradient
      {
        const perturbed = controls.map((c, k) => (k === i ? new MPCControl(c.thrust, c.tau.x + epsilon, c.tau.y, c.tau.z) : c));
        const statesPlus = this._simulateDynamics(initialState, perturbed);
        const costPlus = this._evaluateCost(statesPlus, perturbed, refHorizon);
        control.tau.x -= learningRate * ((costPlus - baseCost) / epsilon);
      }

      // Pitch torque gradient
      {
        const perturbed = controls.map((c, k) => (k === i ? new MPCControl(c.thrust, c.tau.x, c.tau.y + epsilon, c.tau.z) : c));
        const statesPlus = this._simulateDynamics(initialState, perturbed);
        const costPlus = this._evaluateCost(statesPlus, perturbed, refHorizon);
        control.tau.y -= learningRate * ((costPlus - baseCost) / epsilon);
      }

      // Yaw torque gradient
      {
        const perturbed = controls.map((c, k) => (k === i ? new MPCControl(c.thrust, c.tau.x, c.tau.y, c.tau.z + epsilon) : c));
        const statesPlus = this._simulateDynamics(initialState, perturbed);
        const costPlus = this._evaluateCost(statesPlus, perturbed, refHorizon);
        control.tau.z -= learningRate * ((costPlus - baseCost) / epsilon);
      }

      this._clampControl(control);
    }

    return newControls;
  }

  // Allocation copied from EthController to keep physics consistency.
  _allocateMotorsFromThrustAndTorque(thrust_des, tau_cmd) {
    const T_max = this.maxThrustPerMotor;
    const L = this.L;
    const kF = this.kF;
    const kM = this.kM;

    thrust_des = THREE.MathUtils.clamp(thrust_des, 0, 4 * T_max);

    const solveThrusts = (yawTorque) => {
      const S = thrust_des / kF;
      const Tx = tau_cmd.x * Math.SQRT2 / (L * kF);
      const Ty = tau_cmd.y * Math.SQRT2 / (L * kF);
      const Tz = yawTorque / kM;

      const f0 = 0.25 * (S + Tx + Ty - Tz);
      const f1 = 0.25 * (S - Tx + Ty + Tz);
      const f2 = 0.25 * (S - Tx - Ty - Tz);
      const f3 = 0.25 * (S + Tx - Ty + Tz);
      return [f0 * kF, f1 * kF, f2 * kF, f3 * kF];
    };

    let [T0, T1, T2, T3] = solveThrusts(tau_cmd.z);

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

  getPredictedTrajectory() {
    return this.predictedTrajectory;
  }

  getMetrics() {
    return {
      optimizationTime: this.lastOptimizationTime,
      horizonSteps: this.horizonSteps,
      predictionTime: this.horizonSteps * this.predictionDt,
      convergenceHistory: this.convergenceHistory,
    };
  }
}
