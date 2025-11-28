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
    this.inertia = new THREE.Vector3(params.inertia?.Jxx || 0.01, params.inertia?.Jyy || 0.01, params.inertia?.Jzz || 0.02);

    // Gains (ETH Zürich-inspired cascaded control tuned for the demo mass/inertia)
    // Position loop gains (outer loop) – higher Z gains to counter gravity and drag
    this.Kp = new THREE.Vector3(6.0, 6.0, 8.0);
    this.Kd = new THREE.Vector3(4.0, 4.0, 5.0);
    this.Ki = new THREE.Vector3(0.12, 0.12, 0.12);

    // Attitude loop gains (inner loop) for SE(3) geometric controller
    this.KR = new THREE.Vector3(1.0, 1.0, 5.0);
    this.Komega = new THREE.Vector3(0.2, 0.2, 0.3);

    // Integral clamp to avoid windup; tuned to stay within actuator limits
    this.integralLimit = new THREE.Vector3(0.6, 0.6, 0.8);

    this.maxAcc = 20.0;
    this.maxTiltAngle = 60 * Math.PI / 180; // 60 degrees for aggressive racing flight
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

    // Differential flatness mapping to desired orientation
    // Physics uses body +Z as thrust direction, world +Z is up
    const z_b_des = a_cmd.lengthSq() > 1e-9 ? a_cmd.clone().normalize() : new THREE.Vector3(0, 0, 1);
    
    // Desired heading in world frame (yaw) - X-Y plane since Z is up
    const x_c = new THREE.Vector3(Math.cos(yaw_des), Math.sin(yaw_des), 0);
    
    // Body y-axis is perpendicular to both z_b and x_c (points left)
    let y_b_des = z_b_des.clone().cross(x_c);
    if (y_b_des.lengthSq() < 1e-6) {
      // z_b parallel to x_c - pick an arbitrary y_b perpendicular to z_b
      y_b_des = new THREE.Vector3(-z_b_des.z, 0, z_b_des.x);
    }
    y_b_des.normalize();
    
    // Body x-axis completes the right-handed frame
    const x_b_des = y_b_des.clone().cross(z_b_des).normalize();

    const m4 = new THREE.Matrix4();
    m4.makeBasis(x_b_des, y_b_des, z_b_des);
    const q_des = new THREE.Quaternion().setFromRotationMatrix(m4);

    // Required collective thrust (project acceleration onto thrust direction = z-axis)
    let thrust_des = this.mass * a_cmd.dot(z_b_des);
    thrust_des = Math.max(0, Math.min(thrust_des, 4 * this.maxThrustPerMotor));

    // SE(3) geometric attitude controller
    const q = state.orientationQuat.clone();
    const R = new THREE.Matrix3().setFromMatrix4(new THREE.Matrix4().makeRotationFromQuaternion(q));
    const R_des = new THREE.Matrix3().setFromMatrix4(new THREE.Matrix4().makeRotationFromQuaternion(q_des));

    const R_T = R.clone().transpose();
    const R_des_T = R_des.clone().transpose();

    const term1 = new THREE.Matrix3().multiplyMatrices(R_des_T, R);
    const term2 = new THREE.Matrix3().multiplyMatrices(R_T, R_des);
    const skewElements = term1.elements.map((v, i) => v - term2.elements[i]);
    const skew = new THREE.Matrix3().fromArray(skewElements);

    // vee operator for a skew-symmetric matrix stored in column-major order
    const vee = (M) => new THREE.Vector3(M.elements[7], M.elements[2], M.elements[3]);
    const e_R = vee(skew).multiplyScalar(0.5);

    const omega = state.angularVelocity.clone();
    const omega_des = new THREE.Vector3(); // hover
    const omega_des_body = omega_des
      .clone()
      .applyMatrix3(new THREE.Matrix3().multiplyMatrices(R_T, R_des));
    const e_omega = omega.clone().sub(omega_des_body);

    const Iomega = new THREE.Vector3(
      this.inertia.x * omega.x,
      this.inertia.y * omega.y,
      this.inertia.z * omega.z,
    );
    const coriolis = omega.clone().cross(Iomega);

    const tau_cmd = new THREE.Vector3(
      this.KR.x * e_R.x - this.Komega.x * e_omega.x + coriolis.x,
      this.KR.y * e_R.y - this.Komega.y * e_omega.y + coriolis.y,
      this.KR.z * e_R.z - this.Komega.z * e_omega.z + coriolis.z,
    );

    // ETH Zürich control allocation (X-configuration) matching physics engine
    // Motor layout: 0=FL(CW), 1=FR(CCW), 2=BR(CW), 3=BL(CCW)
    // Forward model: τ_x = L*(T1-T3), τ_y = L*(T2-T0), τ_z = (kM/kF)*(-T0+T1-T2+T3)
    const L = this.L;
    const kF = this.kF;
    const kM = this.kM;
    const yawToThrust = kM / kF; // maps thrust imbalance to yaw torque as used in physics engine

    const solveThrusts = (yawTerm) => {
      // Mixer solved so that plugging the resulting T_i back into the physics
      // equations reproduces tau_cmd exactly (including yaw torque scaling).
      const yawAlloc = yawTerm / yawToThrust;

      const T0 = 0.25 * thrust_des - 0.5 * tau_cmd.y / L - 0.25 * yawAlloc; // Front-Left (CW)
      const T1 = 0.25 * thrust_des + 0.5 * tau_cmd.x / L + 0.25 * yawAlloc; // Front-Right (CCW)
      const T2 = 0.25 * thrust_des + 0.5 * tau_cmd.y / L - 0.25 * yawAlloc; // Back-Right (CW)
      const T3 = 0.25 * thrust_des - 0.5 * tau_cmd.x / L + 0.25 * yawAlloc; // Back-Left (CCW)

      return [T0, T1, T2, T3];
    };

    let [T0, T1, T2, T3] = solveThrusts(tau_cmd.z);

    T0 = Math.max(T0, 0);
    T1 = Math.max(T1, 0);
    T2 = Math.max(T2, 0);
    T3 = Math.max(T3, 0);

    const T_max = this.maxThrustPerMotor;
    const anyOverMax = () => T0 > T_max || T1 > T_max || T2 > T_max || T3 > T_max;

    if (anyOverMax()) {
      [T0, T1, T2, T3] = solveThrusts(0); // drop yaw first

      T0 = Math.min(Math.max(T0, 0), T_max);
      T1 = Math.min(Math.max(T1, 0), T_max);
      T2 = Math.min(Math.max(T2, 0), T_max);
      T3 = Math.min(Math.max(T3, 0), T_max);

      if (anyOverMax()) {
        const scale = T_max / Math.max(T0, T1, T2, T3);
        T0 *= scale;
        T1 *= scale;
        T2 *= scale;
        T3 *= scale;
      }
    }

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
