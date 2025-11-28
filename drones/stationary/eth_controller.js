// Cascaded ETH-style quadrotor controller (position PID + attitude PD + allocation)
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

    // Gains
    this.Kp = new THREE.Vector3(4.0, 6.0, 4.0);
    this.Kd = new THREE.Vector3(3.0, 4.5, 3.0);
    this.Ki = new THREE.Vector3(0.5, 0.8, 0.5);
    this.KR = new THREE.Vector3(6.0, 6.0, 3.0);
    this.Komega = new THREE.Vector3(0.25, 0.25, 0.2);

    this.maxAcc = 20.0;
    this.reset();
  }

  reset() {
    this.posIntegral = new THREE.Vector3();
    this.prevOmega = null;
  }

  updatePositionGains({ kp, ki, kd }) {
    if (kp) this.Kp.set(kp, kp, kp);
    if (ki) this.Ki.set(ki, ki, ki);
    if (kd) this.Kd.set(kd, kd, kd);
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
    this.posIntegral.clampLength(-1.5, 1.5);

    const a_cmd = new THREE.Vector3(
      this.Kp.x * posError.x + this.Kd.x * velError.x + this.Ki.x * this.posIntegral.x + a_ff.x,
      this.Kp.y * posError.y + this.Kd.y * velError.y + this.Ki.y * this.posIntegral.y + a_ff.y + this.gravity,
      this.Kp.z * posError.z + this.Kd.z * velError.z + this.Ki.z * this.posIntegral.z + a_ff.z,
    );

    if (a_cmd.length() > this.maxAcc) {
      a_cmd.multiplyScalar(this.maxAcc / a_cmd.length());
    }

    // Differential flatness mapping to desired orientation
    const z_b_des = a_cmd.lengthSq() > 1e-9 ? a_cmd.clone().normalize() : new THREE.Vector3(0, 1, 0);
    const x_c = new THREE.Vector3(Math.cos(yaw_des), 0, Math.sin(yaw_des));

    if (z_b_des.clone().cross(x_c).length() < 1e-6) {
      x_c.set(1, 0, 0);
    }

    const y_b_des = z_b_des.clone().cross(x_c).normalize();
    const x_b_des = y_b_des.clone().cross(z_b_des).normalize();

    const m4 = new THREE.Matrix4();
    m4.makeBasis(x_b_des, y_b_des, z_b_des);
    const q_des = new THREE.Quaternion().setFromRotationMatrix(m4);

    // Required collective thrust
    let thrust_des = this.mass * a_cmd.dot(z_b_des);
    thrust_des = Math.max(0, Math.min(thrust_des, 4 * this.maxThrustPerMotor));

    // Attitude PD
    const q = state.orientationQuat.clone();
    const q_err = q.clone().conjugate().multiply(q_des.clone());
    if (q_err.w < 0) {
      q_err.x *= -1;
      q_err.y *= -1;
      q_err.z *= -1;
      q_err.w *= -1;
    }
    const e_R = new THREE.Vector3(2 * q_err.x, 2 * q_err.y, 2 * q_err.z);

    const omega = state.angularVelocity.clone();
    const e_omega = omega.clone();

    const tau_cmd = new THREE.Vector3(
      -this.KR.x * e_R.x - this.Komega.x * e_omega.x,
      -this.KR.y * e_R.y - this.Komega.y * e_omega.y,
      -this.KR.z * e_R.z - this.Komega.z * e_omega.z,
    );

    // INDI-like correction
    if (this.prevOmega) {
      const omega_dot_meas = omega.clone().sub(this.prevOmega).divideScalar(Math.max(dt, 1e-4));
      const omega_dot_des = new THREE.Vector3(
        tau_cmd.x / this.params.inertia.Jxx,
        tau_cmd.y / this.params.inertia.Jyy,
        tau_cmd.z / this.params.inertia.Jzz,
      );
      const corr = new THREE.Vector3(
        this.params.inertia.Jxx * (omega_dot_des.x - omega_dot_meas.x),
        this.params.inertia.Jyy * (omega_dot_des.y - omega_dot_meas.y),
        this.params.inertia.Jzz * (omega_dot_des.z - omega_dot_meas.z),
      );
      tau_cmd.add(corr);
    }
    this.prevOmega = omega.clone();

    // Control allocation (PLUS configuration)
    // Physics mapping: tau_x = L*(T0-T2), tau_y = L*(T1-T3), tau_z = (kM/kF)*(T0-T1+T2-T3)
    // Inverse allocation for thrusts:
    const L = this.L;
    const kF = this.kF;
    const kM = this.kM;
    const yawFactor = kF / kM;

    let T1 = 0.25 * (thrust_des + tau_cmd.x / L + tau_cmd.z * yawFactor);
    let T2 = 0.25 * (thrust_des + tau_cmd.y / L - tau_cmd.z * yawFactor);
    let T3 = 0.25 * (thrust_des - tau_cmd.x / L + tau_cmd.z * yawFactor);
    let T4 = 0.25 * (thrust_des - tau_cmd.y / L - tau_cmd.z * yawFactor);

    T1 = Math.max(T1, 0);
    T2 = Math.max(T2, 0);
    T3 = Math.max(T3, 0);
    T4 = Math.max(T4, 0);

    const T_max = this.maxThrustPerMotor;
    const anyOverMax = () => T1 > T_max || T2 > T_max || T3 > T_max || T4 > T_max;

    if (anyOverMax()) {
      tau_cmd.z = 0; // drop yaw first
      T1 = 0.25 * (thrust_des + tau_cmd.x / L);
      T2 = 0.25 * (thrust_des - tau_cmd.y / L);
      T3 = 0.25 * (thrust_des - tau_cmd.x / L);
      T4 = 0.25 * (thrust_des + tau_cmd.y / L);

      T1 = Math.min(Math.max(T1, 0), T_max);
      T2 = Math.min(Math.max(T2, 0), T_max);
      T3 = Math.min(Math.max(T3, 0), T_max);
      T4 = Math.min(Math.max(T4, 0), T_max);

      if (anyOverMax()) {
        const scale = T_max / Math.max(T1, T2, T3, T4);
        T1 *= scale;
        T2 *= scale;
        T3 *= scale;
        T4 *= scale;
      }
    }

    const omegaRange = this.omegaMax - this.omegaMin;
    const thrustToCmd = (T) => {
      const omega = Math.sqrt(Math.max(T, 0) / kF);
      const u = (omega - this.omegaMin) / omegaRange;
      return Math.min(Math.max(u, 0), 1);
    };

    const motorCommands = [thrustToCmd(T1), thrustToCmd(T2), thrustToCmd(T3), thrustToCmd(T4)];

    return {
      motorCommands,
      desiredOrientation: q_des,
    };
  }
}
