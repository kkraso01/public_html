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

    // Gains (ETH-style cascaded control)
    // Position loop gains (outer loop)
    this.Kp = new THREE.Vector3(4.0, 6.0, 4.0);
    this.Kd = new THREE.Vector3(3.0, 4.5, 3.0);
    this.Ki = new THREE.Vector3(0.5, 0.8, 0.5);
    
    // Attitude loop gains (inner loop) - must be aggressive enough to track orientation
    this.KR = new THREE.Vector3(8.0, 8.0, 4.0);      // Attitude P: strong response to orientation error
    this.Komega = new THREE.Vector3(0.5, 0.5, 0.3);  // Attitude D: damping on angular velocity

    this.maxAcc = 20.0;
    this.reset();
  }

  reset() {
    this.posIntegral = new THREE.Vector3();
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

    // Acceleration command: gravity compensation is now in +Z (up) direction
    const a_cmd = new THREE.Vector3(
      this.Kp.x * posError.x + this.Kd.x * velError.x + this.Ki.x * this.posIntegral.x + a_ff.x,
      this.Kp.y * posError.y + this.Kd.y * velError.y + this.Ki.y * this.posIntegral.y + a_ff.y,
      this.Kp.z * posError.z + this.Kd.z * velError.z + this.Ki.z * this.posIntegral.z + a_ff.z + this.gravity,
    );

    if (a_cmd.length() > this.maxAcc) {
      a_cmd.multiplyScalar(this.maxAcc / a_cmd.length());
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

    // ETH Zürich control allocation (X-configuration)
    // Motor layout: 0=FL(CW), 1=FR(CCW), 2=BR(CW), 3=BL(CCW)
    // Forward model: τ_x = L*kF*(ω1²-ω3²), τ_y = L*kF*(ω2²-ω0²), τ_z = kM*(-ω0²+ω1²-ω2²+ω3²)
    // With T_i = kF*ω_i², we have: τ_x = L*(T1-T3), τ_y = L*(T2-T0), τ_z = (kM/kF)*(-T0+T1-T2+T3)
    const L = this.L;
    const kF = this.kF;
    const kM = this.kM;

    const solveThrusts = (yawTerm) => {
      // ETH Zürich mixer (from spec):
      // T0 = 0.25*T - 0.5*τ_y/L - 0.5*τ_z/kM   (Front-Left, CW)
      // T1 = 0.25*T + 0.5*τ_x/L + 0.5*τ_z/kM   (Front-Right, CCW)
      // T2 = 0.25*T + 0.5*τ_y/L - 0.5*τ_z/kM   (Back-Right, CW)
      // T3 = 0.25*T - 0.5*τ_x/L + 0.5*τ_z/kM   (Back-Left, CCW)
      // Note: ETH spec assumes τ_z relates to motor thrusts directly via kM, not kM/kF
      
      const T0 = 0.25 * thrust_des - 0.5 * tau_cmd.y / L - 0.5 * yawTerm / kM;
      const T1 = 0.25 * thrust_des + 0.5 * tau_cmd.x / L + 0.5 * yawTerm / kM;
      const T2 = 0.25 * thrust_des + 0.5 * tau_cmd.y / L - 0.5 * yawTerm / kM;
      const T3 = 0.25 * thrust_des - 0.5 * tau_cmd.x / L + 0.5 * yawTerm / kM;
      
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
