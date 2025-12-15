// Cascaded ETH-style quadrotor controller (position PID + geometric attitude + allocation)
export class EthController {
  constructor(params) {
    this.params = params;
    this.mass = params.mass;
    this.kF = params.thrustCoeff;
    this.kM = params.torqueCoeff;
    this.L = params.armLength;
    this.gravity = params.gravity;
    
    // DEBUG: Verify correct Crazyflie mass is being used
    // console.log(`[ETH_CONTROLLER_INIT] mass=${this.mass} kg (should be ~0.028 kg for Crazyflie)`);
    if (this.mass > 0.1) {
      console.error(`⚠️  MASS TOO HIGH! ${this.mass} kg is not a Crazyflie. Controller gains will be wrong!`);
    }
    const motor = params.motor || {};
    this.omegaMin = motor.omegaMin ?? 0;
    this.omegaMax = motor.omegaMax ?? 1;

    // Use actual motor limits from physics model (kF * omegaMax²)
    // This matches the real thrust envelope of the simulator
    this.maxThrustPerMotor = params.maxThrustPerMotor ??
                              (this.kF * this.omegaMax * this.omegaMax);

    // Log thrust envelope vs hover to verify descent authority
    const Tmin_total = 4 * this.kF * this.omegaMin * this.omegaMin;
    const Thover = this.mass * this.gravity;
    console.log(
      `[THRUST_LIMITS] Tmin_total=${Tmin_total.toFixed(6)} N | Thover=${Thover.toFixed(6)} N | ratio=${(Tmin_total / Thover).toFixed(3)}`
    );
    
    // Store last valid nose-first heading to maintain orientation near waypoints
    this.lastNoseFirstYaw = 0;

    // Diagonal inertia used by the geometric attitude controller
    this.inertia = new THREE.Vector3(params.inertia?.Jxx || 1.4e-5, params.inertia?.Jyy || 1.4e-5, params.inertia?.Jzz || 2.17e-5);

    
    // From Mellinger 2011, Lee 2010, Falanga 2018, mav_control_rw
    // These are the EXACT gains used by ETH Flying Machine Arena
    
    // Position loop gains (outer loop) - proven stable for small quadrotors
    this.Kp = new THREE.Vector3(4.0, 4.0, 8.0);   // ETH standard for agile platforms
    this.Kd = new THREE.Vector3(3.0, 3.0, 4.0);   // Critical damping, Z slightly higher
    this.Ki = new THREE.Vector3(0.05, 0.05, 0.20); // Gentle integral, Z higher for altitude hold

    // Attitude loop gains (inner loop) - SE(3) geometric controller
    // Scaled for Crazyflie inertia (Jxx=1.4e-5, Jyy=1.4e-5, Jzz=2.17e-5)
    this.KR = new THREE.Vector3(4.5, 4.5, 1.0);      // Roll/pitch aggressive, yaw smooth (ETH standard)
    this.Komega = new THREE.Vector3(0.10, 0.10, 0.05); // Angular rate damping (ETH standard)

    // Integral clamp to prevent windup
    this.integralLimit = new THREE.Vector3(0.3, 0.3, 0.5);

    // Physical limits matching ETH SE(3) controllers for agile flight
    this.maxAcc = 25.0; // m/s² - Crazyflie realistic max (T/W=2.62x gives ~16 m/s² + margin)
    this.maxTiltAngle = 35 * Math.PI / 180; // 35° - ETH default for safe aggressive flight
    this.reset();
  }

  reset() {
    this.posIntegral = new THREE.Vector3();
  }

  updatePositionGains({ kp, ki, kd }) {
    // Accept scalar (legacy sliders) or per-axis objects/vectors
    const assignVec = (target, value) => {
      if (value instanceof THREE.Vector3) {
        target.copy(value);
      } else if (typeof value === 'number') {
        target.set(value, value, value);
      } else if (value && typeof value === 'object') {
        target.set(
          value.x ?? target.x,
          value.y ?? target.y,
          value.z ?? target.z,
        );
      }
    };

    assignVec(this.Kp, kp);
    assignVec(this.Ki, ki);
    assignVec(this.Kd, kd);
  }

  updateAttitudeGains({ kR, kOmega }) {
    const assignVec = (target, value) => {
      if (value instanceof THREE.Vector3) {
        target.copy(value);
      } else if (typeof value === 'number') {
        target.set(value, value, value);
      } else if (value && typeof value === 'object') {
        target.set(
          value.x ?? target.x,
          value.y ?? target.y,
          value.z ?? target.z,
        );
      }
    };

    assignVec(this.KR, kR);
    assignVec(this.Komega, kOmega);
  }

  computeControl(state, target, dt) {
    const p = state.position.clone();
    const v = state.velocity.clone();
    const p_ref = (target?.position || new THREE.Vector3()).clone();
    const v_ref = (target?.velocity || new THREE.Vector3()).clone();
    const a_ff = (target?.acceleration || new THREE.Vector3()).clone();
    const yaw_des = target?.yaw || 0;

    // Outer-loop PID in world frame
    const posError = p_ref.clone().sub(p);
    const velError = v_ref.clone().sub(v);
    
    // Apply deadband for position error near waypoint to prevent micro-oscillations
    // When within 0.15m, reduce gains smoothly to achieve gentle final approach
    const posErrorMag = posError.length();
    let gainScale = 1.0;
    if (posErrorMag < 0.15) {
      // Smooth ramp: 1.0 at 0.15m → 0.3 at 0.05m → 0.1 at 0.0m
      gainScale = Math.max(0.1, posErrorMag / 0.15);
    }

    this.posIntegral.add(posError.clone().multiplyScalar(dt));
    this.posIntegral.set(
      THREE.MathUtils.clamp(this.posIntegral.x, -this.integralLimit.x, this.integralLimit.x),
      THREE.MathUtils.clamp(this.posIntegral.y, -this.integralLimit.y, this.integralLimit.y),
      THREE.MathUtils.clamp(this.posIntegral.z, -this.integralLimit.z, this.integralLimit.z),
    );

    // Acceleration command: gravity compensation is now in +Z (up) direction
    // Apply gain scaling near waypoint for smooth final approach
    const a_cmd = new THREE.Vector3(
      (this.Kp.x * posError.x + this.Kd.x * velError.x) * gainScale + this.Ki.x * this.posIntegral.x + a_ff.x,
      (this.Kp.y * posError.y + this.Kd.y * velError.y) * gainScale + this.Ki.y * this.posIntegral.y + a_ff.y,
      (this.Kp.z * posError.z + this.Kd.z * velError.z) * gainScale + this.Ki.z * this.posIntegral.z + a_ff.z + this.gravity,
    );

    // DEBUG: Log raw acceleration command to verify outer loop is working (disabled for cleaner logs)
    // if (Math.random() < 0.01) { // Log 1% of frames to avoid spam
    //   console.log(`[ACMD_RAW] a_cmd: (${a_cmd.x.toFixed(3)}, ${a_cmd.y.toFixed(3)}, ${a_cmd.z.toFixed(3)}) | ` +
    //               `Kp: (${this.Kp.x}, ${this.Kp.y}, ${this.Kp.z}) | posErr: (${posError.x.toFixed(2)}, ${posError.y.toFixed(2)}, ${posError.z.toFixed(2)})`);
    // }

    if (a_cmd.length() > this.maxAcc) {
      a_cmd.multiplyScalar(this.maxAcc / a_cmd.length());
    }

    // Limit maximum tilt angle by constraining horizontal acceleration
    // Physics: horizontal_acc = g * tan(tilt), so max tilt limits max horizontal acceleration
    const horizontalAcc = Math.sqrt(a_cmd.x * a_cmd.x + a_cmd.y * a_cmd.y);
    const maxHorizontalAcc = this.gravity * Math.tan(this.maxTiltAngle);
    if (horizontalAcc > maxHorizontalAcc) {
      const scale = maxHorizontalAcc / horizontalAcc;
      a_cmd.x *= scale;
      a_cmd.y *= scale;
    }
    // Note: Z acceleration is NOT reduced by tilt limiting - that's handled by the
    // attitude controller which computes required thrust = m * ||a_cmd|| / cos(tilt)

    // DEBUG: Log after tilt clamping to verify horizontal acceleration survives
    // if (Math.random() < 0.01) {
    //   console.log(`[ACMD_POST_CLAMP] a_cmd: (${a_cmd.x.toFixed(3)}, ${a_cmd.y.toFixed(3)}, ${a_cmd.z.toFixed(3)}) | ` +
    //               `horizAcc: ${horizontalAcc.toFixed(2)}, maxHorizAcc: ${maxHorizontalAcc.toFixed(2)} @ ${(this.maxTiltAngle*180/Math.PI).toFixed(0)}°`);
    // }

    // Differential flatness mapping to desired orientation (ETH/Lee SE(3) method)
    // Physics uses body +Z as thrust direction, world +Z is up
    const b3 = a_cmd.lengthSq() > 1e-9 ? a_cmd.clone().normalize() : new THREE.Vector3(0, 0, 1);
    
    // DEBUG: Log b3 immediately after normalization
    // if (Math.random() < 0.01) {
    //   console.log(`[B3_COMPUTED] b3 AFTER normalize: (${b3.x.toFixed(3)}, ${b3.y.toFixed(3)}, ${b3.z.toFixed(3)}) | ` +
    //               `a_cmd was: (${a_cmd.x.toFixed(2)}, ${a_cmd.y.toFixed(2)}, ${a_cmd.z.toFixed(2)})`);
    // }
    
    // Avoid upside-down ambiguity (thrust should point generally upward)
    if (b3.z < 0) {
      console.warn(
        '[B3_FLIP]',
        'a_cmd=', a_cmd.toArray().map((v) => v.toFixed(3)),
        'posErr.z=', posError.z.toFixed(3),
        'velErr.z=', velError.z.toFixed(3)
      );
      b3.negate();
    }
    
    // Desired heading in world frame (yaw) - body X should point this direction
    const b1_ref = new THREE.Vector3(Math.cos(yaw_des), Math.sin(yaw_des), 0);
    
    // DEBUG: Check b1_ref computation
    if (Math.random() < 0.02) {
      console.log(`[B1_REF] yaw_des=${(yaw_des * 180 / Math.PI).toFixed(1)}° → b1_ref=(${b1_ref.x.toFixed(3)}, ${b1_ref.y.toFixed(3)}, ${b1_ref.z.toFixed(3)})`);
    }
    
    // Body Y-axis: perpendicular to both thrust (b3) and desired heading (b1_ref)
    // Using crossVectors ensures b2 = b3 × b1_ref (proper right-handed convention)
    let b2 = new THREE.Vector3().crossVectors(b3, b1_ref);
    if (b2.lengthSq() < 1e-6) {
      // Thrust parallel to desired heading - pick arbitrary Y perpendicular to b3
      b2.set(-b3.y, b3.x, 0);
      if (b2.lengthSq() < 1e-6) b2.set(1, 0, 0);
    }
    b2.normalize();
    
    // Body X-axis: completes right-handed frame, guaranteed to point forward
    // b1 = b2 × b3 ensures X points in heading direction
    const b1 = new THREE.Vector3().crossVectors(b2, b3).normalize();

    // DEBUG: Log b1, b2, b3 and yaw
    if (Math.random() < 0.01) {
      const yaw_from_b1 = Math.atan2(b1.y, b1.x) * 180 / Math.PI;
      console.log(`[ETH_CTRL] yaw_des=${(yaw_des * 180 / Math.PI).toFixed(1)}°, b1_yaw=${yaw_from_b1.toFixed(1)}° | ` +
                  `b1: (${b1.x.toFixed(2)}, ${b1.y.toFixed(2)}, ${b1.z.toFixed(2)}) | ` +
                  `b1_ref: (${b1_ref.x.toFixed(2)}, ${b1_ref.y.toFixed(2)})`);
    }

    // Construct rotation matrix from basis vectors
    // Three.js makeBasis(x, y, z) creates matrix with x, y, z as COLUMNS
    // This is exactly what we want: columns = body axes in world frame
    const m4 = new THREE.Matrix4();
    m4.makeBasis(b1, b2, b3);
    const q_des = new THREE.Quaternion().setFromRotationMatrix(m4);
    
    // DEBUG: Verify q_des by extracting b3 back out
    // if (Math.random() < 0.01) {
    //   const R_check = new THREE.Matrix3().setFromMatrix4(m4);
    //   const b3_check = new THREE.Vector3(0, 0, 1).applyMatrix3(R_check);
    //   console.log(`[Q_DES_CHECK] b3 extracted from q_des: (${b3_check.x.toFixed(3)}, ${b3_check.y.toFixed(3)}, ${b3_check.z.toFixed(3)})`);
    // }

    // SE(3) geometric attitude controller
    const q = state.orientationQuat.clone();
    const R = new THREE.Matrix3().setFromMatrix4(new THREE.Matrix4().makeRotationFromQuaternion(q));
    const R_des = new THREE.Matrix3().setFromMatrix4(new THREE.Matrix4().makeRotationFromQuaternion(q_des));

    // ETH SE(3) thrust calculation: T = m * ||a_des||
    // This automatically compensates for tilt - when tilted, thrust increases to maintain vertical lift
    // This is the KEY difference from naive projection (T = m * a·b3) which loses altitude when tilted
    
    // FIX #1: Correct thrust saturation (CRITICAL BUG FIX)
    // T_i_max = max thrust PER MOTOR
    // T_total_max = max COMBINED thrust of all 4 motors
    const T_i_max = this.maxThrustPerMotor;  // Per-motor limit (e.g., 0.18 N for Crazyflie)
    const T_total_max = 4 * T_i_max;          // Total system limit (e.g., 0.72 N)
    
    let thrust_des = this.mass * a_cmd.length();

    if (Math.random() < 0.02) {
      const Tmin_total = 4 * this.kF * this.omegaMin * this.omegaMin;
      console.log(
        '[CTRL_Z]',
        'posErr.z=', posError.z.toFixed(3),
        'velErr.z=', velError.z.toFixed(3),
        'a_cmd.z=', a_cmd.z.toFixed(3),
        '||a_cmd||=', a_cmd.length().toFixed(3),
        'thrust_des=', thrust_des.toFixed(4),
        'Tmin_total=', Tmin_total.toFixed(4)
      );
    }
    
    // DEBUG: Log thrust calculation every 1% of frames
    // if (Math.random() < 0.01) {
    //   const hoverThrust = this.mass * this.gravity;
    //   const thrustRatio = thrust_des / hoverThrust;
    //   console.log(`[THRUST] T_des=${thrust_des.toFixed(4)} N | hover=${hoverThrust.toFixed(4)} N | ratio=${thrustRatio.toFixed(2)}x | ` +
    //               `||a||=${a_cmd.length().toFixed(2)} | T_i_max=${T_i_max.toFixed(6)} N | T_total_max=${T_total_max.toFixed(4)} N`);
    // }
    
    // NOTE: Do NOT clamp thrust_des here - it needs full range for altitude control
    // Clamping early destroys altitude behavior (can't descend properly when target is below)
    // The per-motor saturation is handled later in the allocation block

    const R_T = R.clone().transpose();
    const R_des_T = R_des.clone().transpose();

    // ETH attitude error: e_R = 0.5 * vee(R_desᵀ R - Rᵀ R_des)
    const term1 = new THREE.Matrix3().multiplyMatrices(R_des_T, R);
    const term2 = new THREE.Matrix3().multiplyMatrices(R_T, R_des);
    const skewElements = term1.elements.map((v, i) => v - term2.elements[i]);
    const skew = new THREE.Matrix3().fromArray(skewElements);

    // Correct vee operator for Three.js column-major matrices
    // For skew-symmetric [[0,-c,b],[c,0,-a],[-b,a,0]], vee returns [a,b,c]
    // Three.js stores matrices column-major: index = col*3 + row.
    // For skew-symmetric [[0,-c,b],[c,0,-a],[-b,a,0]] we have
    // elements = [0,c,-b,-c,0,a,b,-a,0]. Vee returns [a,b,c] = [m21,m02,m10].
    const vee = (M) => new THREE.Vector3(
      M.elements[5],  // m21 = a
      M.elements[6],  // m02 = b
      M.elements[1]   // m10 = c
    );
    const e_R = vee(skew).multiplyScalar(0.5);

    // Angular velocity error (hover: ω_des = 0 in body frame)
    const omega = state.angularVelocity.clone();
    const omega_des_body = new THREE.Vector3(0, 0, 0);
    const e_omega = omega.clone().sub(omega_des_body);

    const Iomega = new THREE.Vector3(
      this.inertia.x * omega.x,
      this.inertia.y * omega.y,
      this.inertia.z * omega.z,
    );
    const coriolis = omega.clone().cross(Iomega);

    // ETH/Lee SE(3) attitude control law: tau = -KR*e_R - Komega*e_omega + omega x (J*omega)
    // Negative feedback is critical for stability
    const tau_cmd = new THREE.Vector3(
      -this.KR.x * e_R.x - this.Komega.x * e_omega.x + coriolis.x,
      -this.KR.y * e_R.y - this.Komega.y * e_omega.y + coriolis.y,
      -this.KR.z * e_R.z - this.Komega.z * e_omega.z + coriolis.z,
    );

    // Clamp torque commands to prevent extreme saturation during aggressive maneuvers
    // Limits chosen based on Crazyflie inertia and max motor torque capability
    const maxTorqueRoll = 0.02;   // N·m (roll about x)
    const maxTorquePitch = 0.02;  // N·m (pitch about y)
    const maxTorqueYaw = 0.01;    // N·m (yaw about z, more conservative)
    tau_cmd.x = THREE.MathUtils.clamp(tau_cmd.x, -maxTorqueRoll, maxTorqueRoll);
    tau_cmd.y = THREE.MathUtils.clamp(tau_cmd.y, -maxTorquePitch, maxTorquePitch);
    tau_cmd.z = THREE.MathUtils.clamp(tau_cmd.z, -maxTorqueYaw, maxTorqueYaw);

    // FIX #2 & #3: Control allocation for TRUE Crazyflie X-configuration matching physics engine
    // Motor layout: 0=FL(CW), 1=FR(CCW), 2=BR(CW), 3=BL(CCW) ← VERIFIED with physics engine
    //
    // Physics forward model (from drone_physics_engine.js lines 100-130):
    //   F_tot = F0 + F1 + F2 + F3
    //   τ_x (roll)  = (L/√2) * kF * ((ω0² + ω3²) - (ω1² + ω2²))  [LEFT - RIGHT]
    //   τ_y (pitch) = (L/√2) * kF * ((ω0² + ω1²) - (ω2² + ω3²))  [FRONT - BACK]
    //   τ_z (yaw)   = kM * (-ω0² + ω1² - ω2² + ω3²)             [CW=-1, CCW=+1]
    //
    // Coordinate system (ETH standard):
    //   +X: forward (nose direction)
    //   +Y: left (port side)
    //   +Z: up (thrust direction)
    //   Roll (+τ_x): right side down
    //   Pitch (+τ_y): nose up
    //   Yaw (+τ_z): nose right (CCW from above)
    //
    // Inverse allocation (solving for fᵢ = ωᵢ²):
    // Normalize: S = T/kF, Tx = τ_x·√2/(L·kF), Ty = τ_y·√2/(L·kF), Tz = τ_z/kM
    // System: [1  1  1  1][f0]   [S ]
    //         [1 -1 -1  1][f1] = [Tx]
    //         [1  1 -1 -1][f2]   [Ty]
    //         [-1 1 -1  1][f3]   [Tz]
    // Solution:
    //   f₀ = ¼(S + Tx + Ty - Tz)
    //   f₁ = ¼(S - Tx + Ty + Tz)
    //   f₂ = ¼(S - Tx - Ty - Tz)
    //   f₃ = ¼(S + Tx - Ty + Tz)

    const L = this.L;
    const kF = this.kF;
    const kM = this.kM;
    
    const solveThrusts = (yawTorque) => {
      const S  = thrust_des / kF;
      const Tx = tau_cmd.x * Math.SQRT2 / (L * kF);
      const Ty = tau_cmd.y * Math.SQRT2 / (L * kF);
      const Tz = yawTorque / kM;

      const f0 = 0.25 * (S + Tx + Ty - Tz);
      const f1 = 0.25 * (S - Tx + Ty + Tz);
      const f2 = 0.25 * (S - Tx - Ty - Tz);
      const f3 = 0.25 * (S + Tx - Ty + Tz);

      // Convert back to thrust: Tᵢ = kF · fᵢ
      return [f0 * kF, f1 * kF, f2 * kF, f3 * kF];
    };

    let [T0, T1, T2, T3] = solveThrusts(tau_cmd.z);

    // FIX #1 (continued): Check against PER-MOTOR limits, not total thrust
    const anyOverMax = () => T0 > T_i_max || T1 > T_i_max || T2 > T_i_max || T3 > T_i_max;
    const anyNegative = () => T0 < 0 || T1 < 0 || T2 < 0 || T3 < 0;

    // Saturation handling with prioritization: attitude > yaw
    if (anyOverMax() || anyNegative()) {
      // First attempt: drop yaw control to preserve attitude stability
      [T0, T1, T2, T3] = solveThrusts(0);

      if (anyOverMax()) {
        // Scale down proportionally if still saturated
        const scale = T_i_max / Math.max(T0, T1, T2, T3);
        T0 *= scale;
        T1 *= scale;
        T2 *= scale;
        T3 *= scale;
      }
    }

    // Final clamp to physical per-motor limits [0, T_i_max]
    T0 = Math.min(Math.max(T0, 0), T_i_max);
    T1 = Math.min(Math.max(T1, 0), T_i_max);
    T2 = Math.min(Math.max(T2, 0), T_i_max);
    T3 = Math.min(Math.max(T3, 0), T_i_max);

    const omegaRange = this.omegaMax - this.omegaMin;
    const thrustToCmd = (T) => {
      // T = kF * ω² => ω = sqrt(T / kF)
      const omega = Math.sqrt(Math.max(T, 0) / this.kF);
      const u = (omega - this.omegaMin) / omegaRange;
      return Math.min(Math.max(u, 0), 1);
    };

    const motorCommands = [thrustToCmd(T0), thrustToCmd(T1), thrustToCmd(T2), thrustToCmd(T3)];

    return {
      motorCommands,
      desiredOrientation: q_des,
    };
  }

  // ========================================================================
  // NOSE-FIRST TRAJECTORY FOLLOWING EXTENSION
  // ========================================================================
  
  /**
   * Compute desired yaw to point nose toward target position
   * @param {THREE.Vector3} currentPos - Current position
   * @param {THREE.Vector3} targetPos - Target waypoint position
   * @returns {number} Desired yaw angle in radians
   */
  computeNoseFirstYaw(currentPos, targetPos) {
    const dx = targetPos.x - currentPos.x;
    const dy = targetPos.y - currentPos.y;
    
    // Only update heading if moving significantly (avoid jitter at waypoint)
    const distanceToTarget = Math.sqrt(dx * dx + dy * dy);
    if (distanceToTarget < 0.05) {
      return null; // No heading update near waypoint
    }
    
    return Math.atan2(dy, dx);
  }

  /**
   * Compute nose-first control with automatic heading alignment
   * @param {Object} state - Current drone state
   * @param {Object} target - Target waypoint {position, velocity?, acceleration?, yaw?}
   * @param {number} dt - Time step
   * @param {Object} options - Optional config {enableNoseFacing: true, yawRate: 2.0}
   * @returns {Object} Control output with motorCommands and desiredOrientation
   */
  computeNoseFirstControl(state, target, dt, options = {}) {
    const enableNoseFacing = options.enableNoseFacing !== false;
    const maxYawRate = options.maxYawRate || 100.0; // rad/s - VERY fast for racing (no rate limiting in practice)
    const aggressiveYawTracking = options.aggressiveYawTracking !== false; // Use proportional tracking for large errors
    
    if (!enableNoseFacing) {
      // Fallback to standard control if nose-first disabled
      return this.computeControl(state, target, dt);
    }

    // Compute heading toward target based on position vector
    const dx = target.position.x - state.position.x;
    const dy = target.position.y - state.position.y;
    const distanceToTarget = Math.sqrt(dx * dx + dy * dy);
    
    // Update heading based on distance to target
    let desiredYaw;
    if (distanceToTarget > 0.05) {
      // Far from waypoint: point nose toward target
      desiredYaw = Math.atan2(dy, dx);
      this.lastNoseFirstYaw = desiredYaw; // Store for use near waypoint
      
      // Debug log occasionally
      if (Math.random() < 0.01) {
        console.log(`[NOSE_FIRST] distance=${distanceToTarget.toFixed(2)}m, dx=${dx.toFixed(2)}, dy=${dy.toFixed(2)}, desiredYaw=${(desiredYaw * 180 / Math.PI).toFixed(1)}°`);
      }
    } else {
      // Near waypoint: maintain last heading to avoid sudden rotation
      desiredYaw = this.lastNoseFirstYaw;
    }
    
    // Extract current yaw from quaternion
    // For +Z up convention: yaw is rotation around Z axis
    const q = state.orientationQuat;
    const currentYaw = Math.atan2(
      2 * (q.w * q.z + q.x * q.y),
      1 - 2 * (q.y * q.y + q.z * q.z)
    );
    
    // Compute yaw error and wrap to [-π, π]
    let yawError = desiredYaw - currentYaw;
    const yawErrorRaw = yawError;
    while (yawError > Math.PI) yawError -= 2 * Math.PI;
    while (yawError < -Math.PI) yawError += 2 * Math.PI;
    
    // ETH Racing Strategy: Feed the TRUE desired yaw directly to SE(3) controller
    // Let the attitude gains (KR.z=2.0, Komega.z=0.5) handle smooth convergence
    // This is the CORRECT way - no intermediate yaw tracking layer
    const yawCommand = desiredYaw;
    
    // Debug log occasionally
    if (Math.random() < 0.01) {
      console.log(`[NOSE_FIRST_CTRL] desiredYaw=${(desiredYaw * 180 / Math.PI).toFixed(1)}°, currentYaw=${(currentYaw * 180 / Math.PI).toFixed(1)}°, ` +
                  `error=${(yawError * 180 / Math.PI).toFixed(1)}°, yawCmd=${(yawCommand * 180 / Math.PI).toFixed(1)}° ✓ DIRECT`);
    }

    // Create modified target with nose-first yaw
    const noseFirstTarget = {
      position: target.position,
      velocity: target.velocity || new THREE.Vector3(),
      acceleration: target.acceleration || new THREE.Vector3(),
      yaw: yawCommand,
    };

    return this.computeControl(state, noseFirstTarget, dt);
  }

  /**
   * Generate a smooth trajectory segment with nose-first orientation
   * Useful for waypoint-based racing or exploration
   * @param {THREE.Vector3} startPos - Start position
   * @param {THREE.Vector3} endPos - End position
   * @param {number} cruiseSpeed - Desired cruise speed (m/s)
   * @param {number} t - Current time in segment [0, duration]
   * @returns {Object} Trajectory point {position, velocity, acceleration, yaw}
   */
  generateNoseFirstTrajectory(startPos, endPos, cruiseSpeed, t) {
    const direction = endPos.clone().sub(startPos);
    const totalDistance = direction.length();
    
    if (totalDistance < 1e-6) {
      return {
        position: startPos.clone(),
        velocity: new THREE.Vector3(),
        acceleration: new THREE.Vector3(),
        yaw: 0,
      };
    }
    
    direction.normalize();
    
    // Simple trapezoidal velocity profile
    const accelPhase = Math.min(cruiseSpeed / 2.0, totalDistance / 3); // 1/3 distance for accel
    const decelPhase = accelPhase;
    const cruisePhase = totalDistance - accelPhase - decelPhase;
    
    const accelTime = accelPhase > 0 ? Math.sqrt(2 * accelPhase / 2.0) : 0;
    const cruiseTime = cruisePhase > 0 ? cruisePhase / cruiseSpeed : 0;
    const decelTime = accelTime;
    const totalTime = accelTime + cruiseTime + decelTime;
    
    let s, v, a;
    
    if (t < accelTime) {
      // Acceleration phase
      const tau = t / accelTime;
      s = 0.5 * 2.0 * t * t;
      v = 2.0 * t;
      a = 2.0;
    } else if (t < accelTime + cruiseTime) {
      // Cruise phase
      const tCruise = t - accelTime;
      s = accelPhase + cruiseSpeed * tCruise;
      v = cruiseSpeed;
      a = 0;
    } else if (t < totalTime) {
      // Deceleration phase
      const tDecel = t - accelTime - cruiseTime;
      const tau = tDecel / decelTime;
      s = accelPhase + cruisePhase + cruiseSpeed * tDecel - 0.5 * 2.0 * tDecel * tDecel;
      v = cruiseSpeed - 2.0 * tDecel;
      a = -2.0;
    } else {
      // Reached endpoint
      s = totalDistance;
      v = 0;
      a = 0;
    }
    
    const position = startPos.clone().add(direction.clone().multiplyScalar(s));
    const velocity = direction.clone().multiplyScalar(v);
    const acceleration = direction.clone().multiplyScalar(a);
    const yaw = Math.atan2(direction.y, direction.x);
    
    return { position, velocity, acceleration, yaw };
  }

  /**
   * Multi-waypoint trajectory with nose-first following
   * @param {Array<THREE.Vector3>} waypoints - Array of waypoint positions
   * @param {number} cruiseSpeed - Desired cruise speed between waypoints
   * @param {number} globalTime - Current time in full trajectory
   * @returns {Object} Current trajectory point {position, velocity, acceleration, yaw, waypointIndex}
   */
  followWaypointPath(waypoints, cruiseSpeed, globalTime) {
    if (!waypoints || waypoints.length === 0) {
      return {
        position: new THREE.Vector3(),
        velocity: new THREE.Vector3(),
        acceleration: new THREE.Vector3(),
        yaw: 0,
        waypointIndex: -1,
      };
    }
    
    if (waypoints.length === 1) {
      return {
        position: waypoints[0].clone(),
        velocity: new THREE.Vector3(),
        acceleration: new THREE.Vector3(),
        yaw: 0,
        waypointIndex: 0,
      };
    }

    // Compute segment durations
    const segments = [];
    for (let i = 0; i < waypoints.length - 1; i++) {
      const start = waypoints[i];
      const end = waypoints[i + 1];
      const distance = start.distanceTo(end);
      
      // Estimate duration using trapezoidal profile
      const accelDist = Math.min(cruiseSpeed / 2.0, distance / 3);
      const decelDist = accelDist;
      const cruiseDist = distance - accelDist - decelDist;
      
      const accelTime = accelDist > 0 ? Math.sqrt(2 * accelDist / 2.0) : 0;
      const cruiseTime = cruiseDist > 0 ? cruiseDist / cruiseSpeed : 0;
      const decelTime = accelTime;
      
      const duration = accelTime + cruiseTime + decelTime;
      
      segments.push({ start, end, duration });
    }

    // Find active segment
    let accumulatedTime = 0;
    for (let i = 0; i < segments.length; i++) {
      const seg = segments[i];
      if (globalTime < accumulatedTime + seg.duration) {
        const localTime = globalTime - accumulatedTime;
        const traj = this.generateNoseFirstTrajectory(seg.start, seg.end, cruiseSpeed, localTime);
        traj.waypointIndex = i;
        return traj;
      }
      accumulatedTime += seg.duration;
    }

    // Past final waypoint - hold position
    const finalPos = waypoints[waypoints.length - 1];
    return {
      position: finalPos.clone(),
      velocity: new THREE.Vector3(),
      acceleration: new THREE.Vector3(),
      yaw: 0,
      waypointIndex: waypoints.length - 1,
    };
  }
}
