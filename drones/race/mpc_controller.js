/**
 * Model Predictive Control (MPC) for Drone Racing with SE(3) Dynamics
 * 
 * Full SE(3) predictive controller specialized for torus gate racing:
 * - Predicts full rigid body dynamics: position, velocity, orientation (quaternion), angular velocity
 * - Optimizes control inputs (thrust T, torques τ) over receding horizon
 * - Cost functions specialized for torus gates (centerline tracking, tube containment)
 * - Uses ETH controller's motor allocation for consistent dynamics
 * 
 * State: x = [p, v, q, ω] ∈ ℝ³ × ℝ³ × S³ × ℝ³
 * Control: u = [T, τ] ∈ ℝ⁺ × ℝ³
 * 
 * Based on research from:
 * - ETH Zürich Robotics and Perception Group (SE(3) control)
 * - MIT's Fast Autonomous Flight (aggressive maneuvers)
 * - University of Zürich's Swift drone racing system (gate racing)
 */

import { EthController } from '../stationary/eth_controller.js';

/**
 * MPC State: Full SE(3) rigid body state
 */
class MPCState {
  constructor(p, v, q, omega) {
    this.position = p.clone();          // THREE.Vector3 (world frame)
    this.velocity = v.clone();          // THREE.Vector3 (world frame)
    this.orientation = q.clone();       // THREE.Quaternion (body→world)
    this.angularVelocity = omega.clone(); // THREE.Vector3 (body frame)
  }

  clone() {
    return new MPCState(
      this.position, this.velocity, this.orientation, this.angularVelocity
    );
  }
}

/**
 * MPC Control: Thrust + body frame torques
 * (Same as ETH controller inner control loop)
 */
class MPCControl {
  constructor(T, tauX, tauY, tauZ) {
    this.thrust = T;                  // N (body +Z direction)
    this.tau = new THREE.Vector3(tauX, tauY, tauZ); // N·m (body frame)
  }

  clone() {
    return new MPCControl(this.thrust, this.tau.x, this.tau.y, this.tau.z);
  }
}

export class MPCController extends EthController {
  constructor(params) {
    super(params); // Initialize ETH controller base (includes motor allocation)
    
    // Gate racing state
    this.gates = null;
    this.currentGateIndex = 0;
    this.previousPosition = new THREE.Vector3(0, 0, 0);
    
    // MPC Hyperparameters (tuned for torus racing)
    this.horizonSteps = 20;              // Prediction horizon (N) - longer for gate lookahead
    this.predictionDt = 0.04;            // 40ms prediction steps (800ms total horizon)
    this.maxIterations = 5;              // SQP iterations per control cycle
    
    // Cost function weights (tuned for aggressive gate racing)
    this.weights = {
      position: 8.0,         // Track position error (moderate - gates have tolerance)
      velocity: 3.0,         // Track velocity error (smooth speed control)
      acceleration: 0.2,     // Torque effort (low - allow aggressive maneuvers)
      thrust: 0.1,           // Penalize high thrust (very low - racing needs power)
      tilt: 1.0,             // Penalize extreme tilts (prevent flips)
      jerk: 0.05,            // Smoothness (low - allow quick changes)
    };
    
    // Physical constraints (aggressive racing limits)
    this.constraints = {
      maxThrust: params.maxThrust || (this.mass * this.gravity * 2.5),
      minThrust: params.minThrust || 0.0,
      maxTiltAngle: params.maxTiltAngle ? THREE.MathUtils.degToRad(params.maxTiltAngle) : THREE.MathUtils.degToRad(65),
      maxTorque: 0.02,       // N·m for roll/pitch
      maxTorqueYaw: 0.01,    // N·m for yaw
      maxVelocity: params.maxVelocity || 12.0,
    };
    
    // MPC optimization state
    this.predictedTrajectory = [];
    this.lastOptimizationTime = 0;
    this.convergenceHistory = [];
    
    // Previous solution for warm-starting
    this.previousSolution = null;
    
    console.log('[MPC SE(3)] Initialized with horizon:', this.horizonSteps, 'steps @', this.predictionDt, 's');
    console.log('[MPC SE(3)] Total prediction time:', (this.horizonSteps * this.predictionDt).toFixed(2), 's');
  }

  /**
   * Set gate sequence for racing
   */
  setGates(gates) {
    this.gates = gates;        // array of TorusGate
    this.currentGateIndex = 0;
    console.log('[MPC SE(3)] Gate sequence set with', gates.length, 'gates');
  }

  reset() {
    super.reset(); // Reset ETH controller state (integral terms, etc.)
    
    // Safe reset - only reset if properties exist (may be called from parent constructor)
    if (this.previousPosition) {
      this.currentGateIndex = 0;
      this.previousPosition.set(0, 0, 0);
      this.predictedTrajectory = [];
      this.previousSolution = null;
      this.convergenceHistory = [];
    }
  }

  getGateIndex() {
    return this.currentGateIndex;
  }

  /**
   * Update controller time (called by demo)
   */
  update(dt) {
    // MPC doesn't need explicit time tracking (gate-based, not time-based)
    // But keep method for compatibility with demo loop
  }

  /**
   * Update gate index (public API for demo compatibility)
   * MPC handles this internally via _updateGateIndex in computeControl,
   * but this allows demo to manually advance gates if needed
   */
  updateGateIndex(position, waypoints, threshold) {
    // MPC uses TorusGate geometry, not waypoint distance
    // Just call internal update for consistency
    this._updateGateIndex(position);
  }

  /**
   * Get current target (for HUD/visualization compatibility)
   * Returns the current gate center as the target position
   */
  getTarget() {
    if (!this.gates || this.currentGateIndex >= this.gates.length) {
      return {
        position: new THREE.Vector3(0, 0, 2.5),
        velocity: new THREE.Vector3(0, 0, 0),
        acceleration: new THREE.Vector3(0, 0, 0),
        yaw: 0,
      };
    }
    
    const gate = this.gates[this.currentGateIndex];
    const speed = 5.0; // Match horizon builder
    const velocity = gate.axis.clone().multiplyScalar(speed);
    
    return {
      position: gate.center.clone(),
      velocity: velocity,
      acceleration: new THREE.Vector3(0, 0, 0),
      yaw: Math.atan2(velocity.y, velocity.x),
    };
  }

  /**
   * Update gate index based on drone position
   * Advances when drone passes through gate plane and is within tube
   */
  _updateGateIndex(position) {
    if (!this.gates || this.currentGateIndex >= this.gates.length) return;
    
    const gate = this.gates[this.currentGateIndex];
    
    if (gate.hasPassed(position, this.previousPosition)) {
      this.currentGateIndex++;
      console.log('[MPC SE(3)] Gate', this.currentGateIndex - 1, 'cleared! Next:', this.currentGateIndex);
    }
    
    this.previousPosition.copy(position);
  }

  /**
   * Main MPC control computation
   * Solves SE(3) optimization problem to find best control sequence for gate racing
   */
  computeControl(state, _unusedTarget, dt) {
    const startTime = performance.now();
    
    // Update gate progress tracking
    this._updateGateIndex(state.position);
    
    // Solve MPC optimization problem using Sequential Quadratic Programming (SQP)
    const solution = this._solveMPC(state);
    
    // Extract first control action (receding horizon principle)
    const controlAction = solution.controls[0];
    
    // Convert control action (T, τ) to motor commands via X-config mixer
    const motorCommands = this._controlToMotors(controlAction, state);
    
    // Store solution for warm-starting next iteration
    this.previousSolution = solution;
    this.predictedTrajectory = solution.predictedStates;
    
    const optimizationTime = performance.now() - startTime;
    this.lastOptimizationTime = optimizationTime;
    
    return {
      motorCommands,
      desiredThrust: controlAction.thrust,
      desiredOrientation: solution.predictedStates[0].orientation.clone(),
      predictedTrajectory: this.predictedTrajectory,
      optimizationTime,
      cost: solution.cost,
      gateIndex: this.currentGateIndex,
    };
  }

  /**
   * Build gate-aware reference trajectory over prediction horizon
   * Aims through current and next gate centers along gate axes
   */
  _buildReferenceHorizon() {
    const horizon = [];
    
    for (let i = 0; i < this.horizonSteps; i++) {
      // Lookahead: advance gate index every few steps in horizon
      const gateIdx = Math.min(
        this.currentGateIndex + Math.floor(i / 5),
        (this.gates?.length || 1) - 1
      );
      
      const gate = this.gates ? this.gates[gateIdx] : null;

      // Reference "desired" state: center of torus, aligned with axis
      let positionRef = new THREE.Vector3(0, 0, 2.5);
      let velocityRef = new THREE.Vector3(0, 0, 0);
      let yawRef = 0;

      if (gate) {
        // Aim for gate center
        positionRef = gate.center.clone();
        
        // Velocity: along gate axis direction (through the ring)
        const speed = 5.0; // m/s, tune for race aggressiveness
        velocityRef = gate.axis.clone().multiplyScalar(speed);
        
        // Yaw: align with velocity direction
        yawRef = Math.atan2(velocityRef.y, velocityRef.x);
      }

      horizon.push({
        position: positionRef,
        velocity: velocityRef,
        yaw: yawRef,
        gateIndex: gateIdx,
      });
    }
    
    return horizon;
  }

  /**
   * Solve MPC optimization using iterative Sequential Quadratic Programming
   * Minimizes: sum(cost_stage) over horizon
   * Subject to: SE(3) dynamics, physical limits, gate constraints
   */
  _solveMPC(currentState) {
    const refHorizon = this._buildReferenceHorizon();
    
    // Initialize control sequence (warm-start from previous solution or hover)
    let controls = this._initializeControls();
    
    let bestCost = Infinity;
    let bestControls = controls;
    let bestStates = [];
    
    // Iterative optimization (simplified SQP)
    for (let iter = 0; iter < this.maxIterations; iter++) {
      // Forward simulate with current control sequence
      const predictedStates = this._simulateDynamics(currentState, controls);
      
      // Evaluate cost
      const cost = this._evaluateCost(predictedStates, controls, refHorizon);
      
      if (cost < bestCost) {
        bestCost = cost;
        bestControls = controls.map(c => c.clone());
        bestStates = predictedStates.map(s => s.clone());
      }
      
      // Compute gradients and update controls
      controls = this._updateControls(controls, predictedStates, refHorizon, currentState);
      
      // Early termination if converged
      if (iter > 0 && Math.abs(cost - bestCost) < 1e-2) {
        break;
      }
    }
    
    this.convergenceHistory.push(bestCost);
    if (this.convergenceHistory.length > 100) this.convergenceHistory.shift();
    
    return {
      controls: bestControls,
      predictedStates: bestStates,
      cost: bestCost,
    };
  }

  /**
   * Initialize control sequence (warm-start or hover)
   */
  _initializeControls() {
    const controls = [];
    const hoverT = this.mass * this.gravity;
    
    for (let i = 0; i < this.horizonSteps; i++) {
      if (this.previousSolution && i < this.previousSolution.controls.length - 1) {
        // Warm-start: shift previous solution
        controls.push(this.previousSolution.controls[i + 1].clone());
      } else {
        // Initialize with hover thrust, zero torque
        controls.push(new MPCControl(hoverT, 0, 0, 0));
      }
    }
    
    return controls;
  }

  /**
   * SE(3) dynamics step: full rigid body dynamics with quaternion kinematics
   * Matches physics engine exactly (+Z UP convention)
   */
  _stepDynamics(state, control, dt) {
    const next = state.clone();

    // Rotation matrix from quaternion (body→world)
    const R = new THREE.Matrix3().setFromMatrix4(
      new THREE.Matrix4().makeRotationFromQuaternion(next.orientation)
    );

    // Thrust in body +Z direction, rotate to world frame
    const e3_body = new THREE.Vector3(0, 0, 1);
    const thrustWorld = e3_body.clone().applyMatrix3(R).multiplyScalar(control.thrust / this.mass);
    
    // Gravity acts in world -Z direction
    const gravityWorld = new THREE.Vector3(0, 0, -this.gravity);
    
    // Total acceleration: v̇ = g + R·e₃·T/m
    const acc = gravityWorld.add(thrustWorld);

    // Position & velocity integration (Euler)
    next.velocity.add(acc.multiplyScalar(dt));
    next.position.add(next.velocity.clone().multiplyScalar(dt));

    // Angular velocity dynamics: ω̇ = J⁻¹(τ - ω × Jω)
    const omega = next.angularVelocity.clone();
    const J = this.inertia; // THREE.Vector3 (Jxx, Jyy, Jzz)

    const Jomega = new THREE.Vector3(
      J.x * omega.x,
      J.y * omega.y,
      J.z * omega.z
    );

    const coriolis = omega.clone().cross(Jomega); // ω × Jω
    const tauNet = control.tau.clone().sub(coriolis);

    const omegaDot = new THREE.Vector3(
      tauNet.x / J.x,
      tauNet.y / J.y,
      tauNet.z / J.z
    );

    next.angularVelocity.add(omegaDot.multiplyScalar(dt));

    // Quaternion kinematics: q̇ = 0.5·q ⊗ [0;ω]
    const q = next.orientation.clone();
    const omegaQuat = new THREE.Quaternion(omega.x, omega.y, omega.z, 0);
    const qDot = q.clone().multiply(omegaQuat);
    qDot.x *= 0.5;
    qDot.y *= 0.5;
    qDot.z *= 0.5;
    qDot.w *= 0.5;

    q.x += qDot.x * dt;
    q.y += qDot.y * dt;
    q.z += qDot.z * dt;
    q.w += qDot.w * dt;
    q.normalize();

    next.orientation.copy(q);

    return next;
  }

  /**
   * Forward simulate dynamics with given control sequence
   */
  _simulateDynamics(initialState, controls) {
    const states = [];
    
    // Convert physics state to MPCState if needed
    let x = new MPCState(
      initialState.position,
      initialState.velocity,
      initialState.orientationQuat || initialState.orientation,
      initialState.angularVelocity
    );
    
    states.push(x.clone());
    
    for (let i = 0; i < controls.length; i++) {
      x = this._stepDynamics(x, controls[i], this.predictionDt);
      states.push(x.clone());
    }
    
    return states;
  }

  /**
   * Torus gate-specific cost: centerline tracking + tube containment
   */
  _gateCost(state, gate) {
    if (!gate) return 0;

    const pos = state.position;
    const distToCenterline = gate.distanceToRingCenterline(pos);
    const signedTorusDist = gate.signedDistance(pos);

    // Encourage being inside the tube, close to centerline
    const w_center = 50.0;  // strong - stay on racing line
    const w_tube   = 20.0;  // strong - avoid collision with gate

    const c_center = w_center * distToCenterline * distToCenterline;

    // Penalty only when outside tube (signedDist > 0 = outside)
    const out = Math.max(0, signedTorusDist);
    const c_tube = w_tube * out * out;

    return c_center + c_tube;
  }

  /**
   * Evaluate total cost over horizon with torus gate awareness
   */
  _evaluateCost(predictedStates, controls, refHorizon) {
    let totalCost = 0;
    
    for (let i = 0; i < this.horizonSteps; i++) {
      const state = predictedStates[i];
      const control = controls[i];
      const ref = refHorizon[i];
      
      // 1) Position / velocity tracking (to reference near gate)
      const posError = state.position.clone().sub(ref.position).length();
      const velError = state.velocity.clone().sub(ref.velocity).length();

      totalCost += this.weights.position * posError * posError;
      totalCost += this.weights.velocity * velError * velError;

      // 2) Gate torus-specific cost (centerline + tube containment)
      const gate = this.gates ? this.gates[ref.gateIndex] : null;
      totalCost += this._gateCost(state, gate);

      // 3) Control effort
      const hoverThrust = this.mass * this.gravity;
      const thrustError = (control.thrust - hoverThrust);
      totalCost += this.weights.thrust * thrustError * thrustError;

      // Torque effort (body frame)
      totalCost += this.weights.acceleration * control.tau.lengthSq();

      // 4) Smoothness cost (jerk / torque changes)
      if (i > 0) {
        const prevControl = controls[i - 1];
        const dT = (control.thrust - prevControl.thrust) / this.predictionDt;
        const dTau = control.tau.clone().sub(prevControl.tau).divideScalar(this.predictionDt);
        totalCost += this.weights.jerk * (dT * dT + dTau.lengthSq());
      }

      // 5) Tilt penalty (avoid flipping unnecessarily)
      const R = new THREE.Matrix3().setFromMatrix4(
        new THREE.Matrix4().makeRotationFromQuaternion(state.orientation)
      );
      const bodyZ = new THREE.Vector3(0, 0, 1).applyMatrix3(R); // body +Z in world frame
      const cosTilt = bodyZ.dot(new THREE.Vector3(0, 0, 1)); // dot with world +Z
      const tiltAngle = Math.acos(THREE.MathUtils.clamp(cosTilt, -1, 1));
      totalCost += this.weights.tilt * tiltAngle * tiltAngle;
    }
    
    return totalCost;
  }

  /**
   * Clamp control to physical constraints
   */
  _clampControl(control) {
    // Thrust limits
    control.thrust = THREE.MathUtils.clamp(
      control.thrust,
      this.constraints.minThrust,
      this.constraints.maxThrust
    );

    // Torque limits (body frame)
    control.tau.x = THREE.MathUtils.clamp(control.tau.x, -this.constraints.maxTorque, this.constraints.maxTorque);
    control.tau.y = THREE.MathUtils.clamp(control.tau.y, -this.constraints.maxTorque, this.constraints.maxTorque);
    control.tau.z = THREE.MathUtils.clamp(control.tau.z, -this.constraints.maxTorqueYaw, this.constraints.maxTorqueYaw);
  }

  /**
   * Update controls using gradient descent for SE(3) controls (T, τ)
   * Uses finite differences to compute gradients
   */
  _updateControls(controls, predictedStates, refHorizon, initialState) {
    const learningRate = 0.05;
    const epsilon = 0.01; // For numerical gradient
    
    const newControls = controls.map(c => c.clone());
    const baseCost = this._evaluateCost(predictedStates, controls, refHorizon);
    
    for (let i = 0; i < controls.length; i++) {
      const control = newControls[i];
      
      // Partial derivative w.r.t. thrust
      {
        const perturbed = controls.map((c, k) => 
          k === i ? new MPCControl(c.thrust + epsilon, c.tau.x, c.tau.y, c.tau.z) : c
        );
        const statesPlus = this._simulateDynamics(initialState, perturbed);
        const costPlus = this._evaluateCost(statesPlus, perturbed, refHorizon);
        const grad = (costPlus - baseCost) / epsilon;
        control.thrust -= learningRate * grad;
      }
      
      // Partial derivative w.r.t. tau.x (roll torque)
      {
        const perturbed = controls.map((c, k) => 
          k === i ? new MPCControl(c.thrust, c.tau.x + epsilon, c.tau.y, c.tau.z) : c
        );
        const statesPlus = this._simulateDynamics(initialState, perturbed);
        const costPlus = this._evaluateCost(statesPlus, perturbed, refHorizon);
        const grad = (costPlus - baseCost) / epsilon;
        control.tau.x -= learningRate * grad;
      }
      
      // Partial derivative w.r.t. tau.y (pitch torque)
      {
        const perturbed = controls.map((c, k) => 
          k === i ? new MPCControl(c.thrust, c.tau.x, c.tau.y + epsilon, c.tau.z) : c
        );
        const statesPlus = this._simulateDynamics(initialState, perturbed);
        const costPlus = this._evaluateCost(statesPlus, perturbed, refHorizon);
        const grad = (costPlus - baseCost) / epsilon;
        control.tau.y -= learningRate * grad;
      }
      
      // Partial derivative w.r.t. tau.z (yaw torque)
      {
        const perturbed = controls.map((c, k) => 
          k === i ? new MPCControl(c.thrust, c.tau.x, c.tau.y, c.tau.z + epsilon) : c
        );
        const statesPlus = this._simulateDynamics(initialState, perturbed);
        const costPlus = this._evaluateCost(statesPlus, perturbed, refHorizon);
        const grad = (costPlus - baseCost) / epsilon;
        control.tau.z -= learningRate * grad;
      }
      
      // Apply constraints
      this._clampControl(control);
    }
    
    return newControls;
  }

  /**
   * Convert MPC control (T, τ) to motor commands using ETH-style X-config mixer
   * Same allocation as EthController for consistency with physics
   */
  _controlToMotors(control, state) {
    const T_max = this.maxThrustPerMotor;
    const L = this.L;
    const kF = this.kF;
    const kM = this.kM;

    const tau_cmd = control.tau;
    let thrust_des = control.thrust;

    // Clamp total thrust to physical limits
    thrust_des = THREE.MathUtils.clamp(thrust_des, 0, 4 * T_max);

    // X-configuration motor allocation (Crazyflie 2.x)
    // Motors: 0=FL(CW), 1=FR(CCW), 2=BR(CW), 3=BL(CCW)
    // Solve for individual motor thrusts from total thrust + torques
    const solveThrusts = (yawTorque) => {
      const S  = thrust_des / kF;            // Total thrust in ω² units
      const Tx = tau_cmd.x * Math.SQRT2 / (L * kF);  // Roll in ω² units
      const Ty = tau_cmd.y * Math.SQRT2 / (L * kF);  // Pitch in ω² units
      const Tz = yawTorque / kM;             // Yaw in ω² units

      // X-config allocation matrix inversion
      const f0 = 0.25 * (S + Tx + Ty - Tz);  // FL
      const f1 = 0.25 * (S - Tx + Ty + Tz);  // FR
      const f2 = 0.25 * (S - Tx - Ty - Tz);  // BR
      const f3 = 0.25 * (S + Tx - Ty + Tz);  // BL

      return [f0 * kF, f1 * kF, f2 * kF, f3 * kF]; // Convert back to thrusts
    };

    let [T0, T1, T2, T3] = solveThrusts(tau_cmd.z);

    const anyOverMax = () => T0 > T_max || T1 > T_max || T2 > T_max || T3 > T_max;
    const anyNegative = () => T0 < 0 || T1 < 0 || T2 < 0 || T3 < 0;

    // Handle saturation: drop yaw authority if needed
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

    // Final clamping
    T0 = Math.min(Math.max(T0, 0), T_max);
    T1 = Math.min(Math.max(T1, 0), T_max);
    T2 = Math.min(Math.max(T2, 0), T_max);
    T3 = Math.min(Math.max(T3, 0), T_max);

    // Convert thrust to motor command [0,1]
    const omegaRange = this.omegaMax - this.omegaMin;
    const thrustToCmd = (T) => {
      const omega = Math.sqrt(Math.max(T, 0) / kF);
      const u = (omega - this.omegaMin) / omegaRange;
      return THREE.MathUtils.clamp(u, 0, 1);
    };

    return [thrustToCmd(T0), thrustToCmd(T1), thrustToCmd(T2), thrustToCmd(T3)];
  }

  /**
   * Get predicted trajectory for visualization
   */
  getPredictedTrajectory() {
    return this.predictedTrajectory;
  }

  /**
   * Get performance metrics
   */
  getMetrics() {
    return {
      optimizationTime: this.lastOptimizationTime,
      horizonSteps: this.horizonSteps,
      predictionTime: this.horizonSteps * this.predictionDt,
      convergenceHistory: this.convergenceHistory,
    };
  }
}
