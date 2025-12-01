import { MotorModel } from './motor_model.js';
import { CRAZYFLIE_PARAMS } from './params_crazyflie.js';

function clamp01(u) {
  return Math.min(Math.max(u, 0), 1);
}

// Correct quaternion derivative for q_WB (body → world) with ω in body frame:
// q̇ = 0.5 * (q ⊗ ω_quat)
// Note: multiply on the RIGHT, not left, when ω is expressed in body frame
function quatDerivative(q, omega) {
  const omegaQuat = new THREE.Quaternion(omega.x, omega.y, omega.z, 0);
  
  // Correct order: q first, then ω_quat (body-frame angular velocity)
  const qDot = q.clone().multiply(omegaQuat);
  
  qDot.x *= 0.5;
  qDot.y *= 0.5;
  qDot.z *= 0.5;
  qDot.w *= 0.5;
  return qDot;
}

export class DronePhysicsEngine {
  constructor(params = {}) {
    this.params = Object.assign({}, CRAZYFLIE_PARAMS, params);

    // Inertia matrices (diagonal for this simplified model)
    this.I = new THREE.Matrix3().set(
      this.params.inertia.Jxx,
      0,
      0,
      0,
      this.params.inertia.Jyy,
      0,
      0,
      0,
      this.params.inertia.Jzz,
    );
    this.I_inv = new THREE.Matrix3().set(
      1 / this.params.inertia.Jxx,
      0,
      0,
      0,
      1 / this.params.inertia.Jyy,
      0,
      0,
      0,
      1 / this.params.inertia.Jzz,
    );

    this.motorModel = new MotorModel(this.params.motor);

    this.state = {
      p_W: new THREE.Vector3(),
      v_W: new THREE.Vector3(),
      q_WB: new THREE.Quaternion(),
      omega_B: new THREE.Vector3(),
      motorSpeeds: [0, 0, 0, 0],
    };

    this.reset();
  }

  reset(initialState = {}) {
    this.state.p_W.copy(initialState.position || new THREE.Vector3());
    this.state.v_W.copy(initialState.velocity || new THREE.Vector3());
    this.state.q_WB.copy(initialState.orientationQuat || initialState.orientation || new THREE.Quaternion());
    this.state.omega_B.copy(initialState.angularVelocity || new THREE.Vector3());
    this.state.motorSpeeds = [0, 0, 0, 0];
    this.motorModel.reset();
    this.motorCmd = [0, 0, 0, 0];
    
    // Add compatibility aliases for direct state access (non-copying references)
    this.state.position = this.state.p_W;
    this.state.velocity = this.state.v_W;
    this.state.orientation = this.state.q_WB;
    this.state.orientationQuat = this.state.q_WB;
    this.state.angularVelocity = this.state.omega_B;
  }

  setDesiredOrientation() {
    // kept for API compatibility; torque is supplied by external controllers
  }

  applyMotorCommands(u0 = 0, u1 = 0, u2 = 0, u3 = 0) {
    const { omegaMin, omegaMax } = this.params.motor;
    const span = omegaMax - omegaMin;
    this.motorCmd = [u0, u1, u2, u3].map((u) => omegaMin + clamp01(u) * span);
  }

  step(dt) {
    if (dt <= 0) return;

    // 1) Update motor speeds with first-order lag
    this.motorModel.step(dt, this.motorCmd);
    const omegaRotors = this.motorModel.getSpeeds();
    this.state.motorSpeeds = omegaRotors.slice();

    // 2) Compute thrust and torques from rotor speeds (TRUE Crazyflie X configuration)
    //
    // Motor layout (matching real Crazyflie 2.x):
    //   0: Front-Left  (CW)
    //   1: Front-Right (CCW)
    //   2: Back-Right  (CW)
    //   3: Back-Left   (CCW)
    //
    // Body frame: x forward, y left, z up

    const { thrustCoeff: kF, torqueCoeff: kM, armLength: L } = this.params;
    const omegaSq = omegaRotors.map(w => w * w);

    // Total thrust
    const thrustTotal = kF * (omegaSq[0] + omegaSq[1] + omegaSq[2] + omegaSq[3]);
    const F_B = new THREE.Vector3(0, 0, thrustTotal);

    // Correct Crazyflie X-configuration torque model with √2 arm projections
    // X-config arms project at 45° → effective moment arm = L/√2
    // 
    // Roll torque (about +X axis): follows Crazyflie X convention
    // τ_x = (L/√2) · kF · [ (M0 + M3) - (M1 + M2) ] (left minus right)
    const tau_x = (L / Math.sqrt(2)) * kF * ((omegaSq[0] + omegaSq[3]) - (omegaSq[1] + omegaSq[2]));

    // Pitch torque (about +Y axis): τ_y = (L/√2) · kF · [ (M0 + M1) - (M2 + M3) ] (front minus back)
    const tau_y = (L / Math.sqrt(2)) * kF * ((omegaSq[0] + omegaSq[1]) - (omegaSq[2] + omegaSq[3]));

    // Yaw torque: CW = negative, CCW = positive
    const tau_z = kM * (-omegaSq[0] + omegaSq[1] - omegaSq[2] + omegaSq[3]);

    const tau_B = new THREE.Vector3(tau_x, tau_y, tau_z);

    // 3) Aerodynamic drag (world frame)
    const { dragLinear } = this.params;
    const F_D = new THREE.Vector3(
      -dragLinear.x * this.state.v_W.x,
      -dragLinear.y * this.state.v_W.y,
      -dragLinear.z * this.state.v_W.z,
    );

    // 4) Translational dynamics
    // Rotation matrix from body frame to world frame (NO transpose!)
    // q_WB rotates body vectors into world frame
    const R_BW = new THREE.Matrix3().setFromMatrix4(new THREE.Matrix4().makeRotationFromQuaternion(this.state.q_WB));
    const F_W_thrust = F_B.clone().applyMatrix3(R_BW);
    const F_gravity = new THREE.Vector3(0, 0, -this.params.mass * this.params.gravity);  // Gravity in -Z (down)

    const F_net = F_W_thrust.clone().add(F_D).add(F_gravity);
    const v_dot = F_net.clone().multiplyScalar(1 / this.params.mass);
    this.state.v_W.add(v_dot.multiplyScalar(dt));
    this.state.p_W.add(this.state.v_W.clone().multiplyScalar(dt));

    // 5) Rotational dynamics
    const omega_B = this.state.omega_B.clone();
    const Iomega = new THREE.Vector3(
      this.params.inertia.Jxx * omega_B.x,
      this.params.inertia.Jyy * omega_B.y,
      this.params.inertia.Jzz * omega_B.z,
    );
    const coriolis = omega_B.clone().cross(Iomega);

    const omegaDot = new THREE.Vector3(
      (tau_B.x - coriolis.x) / this.params.inertia.Jxx,
      (tau_B.y - coriolis.y) / this.params.inertia.Jyy,
      (tau_B.z - coriolis.z) / this.params.inertia.Jzz
    );

    this.state.omega_B.add(omegaDot.multiplyScalar(dt));

    // Angular damping (air resistance on rotation)
    // Real quadrotors experience rotational drag proportional to angular velocity
    // Crazyflie typical values: ~0.5-2.0 for roll/pitch, ~1.0-3.0 for yaw
    const dragAngular = { x: 1.0, y: 1.0, z: 2.0 }; // Higher yaw damping due to larger drag area
    this.state.omega_B.x *= Math.exp(-dragAngular.x * dt);
    this.state.omega_B.y *= Math.exp(-dragAngular.y * dt);
    this.state.omega_B.z *= Math.exp(-dragAngular.z * dt);

    // Quaternion integration
    const qDot = quatDerivative(this.state.q_WB, this.state.omega_B);
    this.state.q_WB.x += qDot.x * dt;
    this.state.q_WB.y += qDot.y * dt;
    this.state.q_WB.z += qDot.z * dt;
    this.state.q_WB.w += qDot.w * dt;
    this.state.q_WB.normalize();
  }

  getState() {
    const orientationQuat = this.state.q_WB.clone();
    return {
      position: this.state.p_W.clone(),
      velocity: this.state.v_W.clone(),
      orientationQuat,
      orientation: orientationQuat.clone(), // alias for existing demos
      quaternion: orientationQuat.clone(),
      angularVelocity: this.state.omega_B.clone(),
      motorSpeeds: this.state.motorSpeeds.slice(),
    };
  }
}
