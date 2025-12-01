// Cascaded ETH-style quadrotor controller (position PID + geometric attitude + allocation)
export class EthController {
  constructor(params) {
    this.params = params;
    this.mass = params.mass;
    this.kF = params.thrustCoeff;
    this.kM = params.torqueCoeff;
    this.L = params.armLength;
    this.gravity = params.gravity;
    const motor = params.motor || {};
    this.omegaMin = motor.omegaMin || 0;
    this.omegaMax = motor.omegaMax || 1;
    this.maxThrustPerMotor = params.maxThrustPerMotor || this.kF * this.omegaMax * this.omegaMax;

    // Diagonal inertia used by the geometric attitude controller
    this.inertia = new THREE.Vector3(params.inertia?.Jxx || 2.4e-5, params.inertia?.Jyy || 2.4e-5, params.inertia?.Jzz || 3.5e-5);

    // Position loop gains (outer loop) - tuned for Crazyflie 32g mass
    // Moderate gains for stable position tracking
    this.Kp = new THREE.Vector3(2.0, 2.0, 4.0);
    this.Kd = new THREE.Vector3(1.5, 1.5, 2.5);
    this.Ki = new THREE.Vector3(0.01, 0.01, 0.02);

    // Attitude loop gains (inner loop) for SE(3) geometric controller
    // Scaled appropriately for Crazyflie's tiny inertia (J ~ 2.4e-5 kg·m²)
    // Rule of thumb: K_R should produce reasonable torques (~ 1e-4 N·m) for unit attitude error
    // Natural frequency target: ω_n ~ 50 rad/s → K_R ~ J * ω_n² ~ 2.4e-5 * 2500 ~ 0.06
    this.KR = new THREE.Vector3(0.06, 0.06, 0.03);
    this.Komega = new THREE.Vector3(0.012, 0.012, 0.006);

    // Integral clamp to avoid windup; scaled for small mass
    this.integralLimit = new THREE.Vector3(0.3, 0.3, 0.5);

    this.maxAcc = 15.0; // m/s² - Crazyflie has high thrust-to-weight ratio
    this.maxTiltAngle = 40 * Math.PI / 180; // 40 degrees - aggressive but safe
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

    this.posIntegral.add(posError.clone().multiplyScalar(dt));
    this.posIntegral.set(
      THREE.MathUtils.clamp(this.posIntegral.x, -this.integralLimit.x, this.integralLimit.x),
      THREE.MathUtils.clamp(this.posIntegral.y, -this.integralLimit.y, this.integralLimit.y),
      THREE.MathUtils.clamp(this.posIntegral.z, -this.integralLimit.z, this.integralLimit.z),
    );

    // Acceleration command: gravity compensation is now in +Z (up) direction
    const a_cmd = new THREE.Vector3(
      this.Kp.x * posError.x + this.Kd.x * velError.x + this.Ki.x * this.posIntegral.x + a_ff.x,
      this.Kp.y * posError.y + this.Kd.y * velError.y + this.Ki.y * this.posIntegral.y + a_ff.y,
      this.Kp.z * posError.z + this.Kd.z * velError.z + this.Ki.z * this.posIntegral.z + a_ff.z + this.gravity,
    );

    // DEBUG: Log raw acceleration command to verify outer loop is working
    if (Math.random() < 0.01) { // Log 1% of frames to avoid spam
      console.log(`[ACMD_RAW] a_cmd: (${a_cmd.x.toFixed(3)}, ${a_cmd.y.toFixed(3)}, ${a_cmd.z.toFixed(3)}) | ` +
                  `Kp: (${this.Kp.x}, ${this.Kp.y}, ${this.Kp.z}) | posErr: (${posError.x.toFixed(2)}, ${posError.y.toFixed(2)})`);
    }

    if (a_cmd.length() > this.maxAcc) {
      a_cmd.multiplyScalar(this.maxAcc / a_cmd.length());
    }

    // Limit maximum tilt angle by constraining horizontal acceleration
    const horizontalAcc = Math.sqrt(a_cmd.x * a_cmd.x + a_cmd.y * a_cmd.y);
    const maxHorizontalAcc = this.gravity * Math.tan(this.maxTiltAngle);
    if (horizontalAcc > maxHorizontalAcc) {
      const scale = maxHorizontalAcc / horizontalAcc;
      a_cmd.x *= scale;
      a_cmd.y *= scale;
    }

    // DEBUG: Log after tilt clamping to verify horizontal acceleration survives
    if (Math.random() < 0.01) {
      console.log(`[ACMD_POST_CLAMP] a_cmd: (${a_cmd.x.toFixed(3)}, ${a_cmd.y.toFixed(3)}, ${a_cmd.z.toFixed(3)}) | ` +
                  `horizAcc: ${horizontalAcc.toFixed(2)}, maxHorizAcc: ${maxHorizontalAcc.toFixed(2)} @ ${(this.maxTiltAngle*180/Math.PI).toFixed(0)}°`);
    }

    // Differential flatness mapping to desired orientation (ETH/Lee SE(3) method)
    // Physics uses body +Z as thrust direction, world +Z is up
    const b3 = a_cmd.lengthSq() > 1e-9 ? a_cmd.clone().normalize() : new THREE.Vector3(0, 0, 1);
    
    // DEBUG: Log b3 immediately after normalization
    if (Math.random() < 0.01) {
      console.log(`[B3_COMPUTED] b3 AFTER normalize: (${b3.x.toFixed(3)}, ${b3.y.toFixed(3)}, ${b3.z.toFixed(3)}) | ` +
                  `a_cmd was: (${a_cmd.x.toFixed(2)}, ${a_cmd.y.toFixed(2)}, ${a_cmd.z.toFixed(2)})`);
    }
    
    // Avoid upside-down ambiguity (thrust should point generally upward)
    if (b3.z < 0) b3.negate();
    
    // Desired heading in world frame (yaw) - body X should point this direction
    const b1_ref = new THREE.Vector3(Math.cos(yaw_des), Math.sin(yaw_des), 0);
    
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

    // DEBUG: Log b1, b2, b3 before matrix construction
    if (Math.random() < 0.01) {
      console.log(`[B1B2B3] b1: (${b1.x.toFixed(2)}, ${b1.y.toFixed(2)}, ${b1.z.toFixed(2)}) | ` +
                  `b2: (${b2.x.toFixed(2)}, ${b2.y.toFixed(2)}, ${b2.z.toFixed(2)}) | ` +
                  `b3: (${b3.x.toFixed(2)}, ${b3.y.toFixed(2)}, ${b3.z.toFixed(2)})`);
    }

    // Construct rotation matrix from basis vectors
    // Three.js makeBasis(x, y, z) creates matrix with x, y, z as COLUMNS
    // This is exactly what we want: columns = body axes in world frame
    const m4 = new THREE.Matrix4();
    m4.makeBasis(b1, b2, b3);
    const q_des = new THREE.Quaternion().setFromRotationMatrix(m4);
    
    // DEBUG: Verify q_des by extracting b3 back out
    if (Math.random() < 0.01) {
      const R_check = new THREE.Matrix3().setFromMatrix4(m4);
      const b3_check = new THREE.Vector3(0, 0, 1).applyMatrix3(R_check);
      console.log(`[Q_DES_CHECK] b3 extracted from q_des: (${b3_check.x.toFixed(3)}, ${b3_check.y.toFixed(3)}, ${b3_check.z.toFixed(3)})`);
    }

    // Required collective thrust (project acceleration onto thrust direction = b3)
    // ETH method: clip to physical limits but let motor allocation handle saturation details
    const T_max = this.maxThrustPerMotor;
    let thrust_des = this.mass * a_cmd.dot(b3);
    thrust_des = THREE.MathUtils.clamp(thrust_des, 0, 4 * T_max);

    // SE(3) geometric attitude controller
    const q = state.orientationQuat.clone();
    const R = new THREE.Matrix3().setFromMatrix4(new THREE.Matrix4().makeRotationFromQuaternion(q));
    const R_des = new THREE.Matrix3().setFromMatrix4(new THREE.Matrix4().makeRotationFromQuaternion(q_des));

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

    // Control allocation for TRUE Crazyflie X-configuration matching physics engine
    // Motor layout: 0=FL(CW), 1=FR(CCW), 2=BR(CW), 3=BL(CCW) ← MUST match physics engine
    //
    // Physics forward model (from drone_physics_engine.js):
    //   F_tot = F0 + F1 + F2 + F3
    //   τ_x (roll)  = (L/√2) * kF * ((ω0² + ω3²) - (ω1² + ω2²))  [LEFT - RIGHT]
    //   τ_y (pitch) = (L/√2) * kF * ((ω0² + ω1²) - (ω2² + ω3²))  [FRONT - BACK]
    //   τ_z (yaw)   = kM * (-ω0² + ω1² - ω2² + ω3²)
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

    const anyOverMax = () => T0 > T_max || T1 > T_max || T2 > T_max || T3 > T_max;
    const anyNegative = () => T0 < 0 || T1 < 0 || T2 < 0 || T3 < 0;

    // Saturation handling with prioritization: attitude > yaw
    if (anyOverMax() || anyNegative()) {
      // First attempt: drop yaw control to preserve attitude stability
      [T0, T1, T2, T3] = solveThrusts(0);

      if (anyOverMax()) {
        // Scale down proportionally if still saturated
        const scale = T_max / Math.max(T0, T1, T2, T3);
        T0 *= scale;
        T1 *= scale;
        T2 *= scale;
        T3 *= scale;
      }
    }

    // Final clamp to physical motor limits [0, T_max]
    T0 = Math.min(Math.max(T0, 0), T_max);
    T1 = Math.min(Math.max(T1, 0), T_max);
    T2 = Math.min(Math.max(T2, 0), T_max);
    T3 = Math.min(Math.max(T3, 0), T_max);

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
}
