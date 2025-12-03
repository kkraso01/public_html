import { EthController } from './eth_controller.js';

export class StationaryController {
  constructor(params = {}) {
    this.params = params;
    this.eth = new EthController(params);
    this.useFOPID = false; // kept for HUD compatibility
    this.useNoseFacing = false; // nose-first mode toggle
    this.lastIntegrationMs = 0; // placeholder for legacy HUD
  }

  reset() {
    this.eth.reset();
  }

  updateGains({ kp, ki, kd }) {
    this.eth.updatePositionGains({ kp, ki, kd });
  }

  updateAttitudeGains({ kR, kOmega }) {
    this.eth.updateAttitudeGains({ kR, kOmega });
  }

  toggleFOPID() {
    // Legacy control panel hook; no-op for cascaded controller
    this.useFOPID = false;
  }

  toggleNoseFacing() {
    this.useNoseFacing = !this.useNoseFacing;
    console.log(`[STATIONARY_CTRL] Nose-first mode: ${this.useNoseFacing ? 'ENABLED' : 'DISABLED'}`);
    return this.useNoseFacing;
  }

  setNoseFacing(enabled) {
    this.useNoseFacing = enabled;
    console.log(`[STATIONARY_CTRL] Nose-first mode: ${this.useNoseFacing ? 'ENABLED' : 'DISABLED'}`);
  }

  compute(state, dt, target = null) {
    // Use provided target or default to hover at origin
    // Hover altitude adjusted for Crazyflie scale (smaller drone, lower default height)
    if (!target) {
      target = {
        position: new THREE.Vector3(0, 0, 0.4),
        velocity: new THREE.Vector3(0, 0, 0),
        acceleration: new THREE.Vector3(0, 0, 0),
        yaw: 0,
      };
    }

    // Debug log occasionally
    if (Math.random() < 0.01) {
      console.log(`[STATIONARY_CTRL.compute] useNoseFacing=${this.useNoseFacing}, target=(${target.position.x.toFixed(2)}, ${target.position.y.toFixed(2)}), state=(${state.position.x.toFixed(2)}, ${state.position.y.toFixed(2)})`);
    }

    // Use nose-first control when enabled (maxYawRate defaults to 10 rad/s in eth controller)
    const result = this.useNoseFacing
      ? this.eth.computeNoseFirstControl(state, target, dt, { enableNoseFacing: true })
      : this.eth.computeControl(state, target, dt);

    // Add meaningful diagnostics for SE(3) controller (not Euler angles!)
    // Extract actual thrust direction from current orientation
    // Body z-axis (0,0,1) in body frame → world frame = third column of rotation matrix
    const m4_actual = new THREE.Matrix4().makeRotationFromQuaternion(state.orientationQuat);
    const b3_actual = new THREE.Vector3(
      m4_actual.elements[8],   // m31 = third column, first row
      m4_actual.elements[9],   // m32 = third column, second row
      m4_actual.elements[10]   // m33 = third column, third row
    );

    // Extract desired thrust direction from desired orientation
    // Body z-axis in desired frame → world frame = third column of R_des
    const m4_desired = new THREE.Matrix4().makeRotationFromQuaternion(result.desiredOrientation);
    const b3_desired = new THREE.Vector3(
      m4_desired.elements[8],   // m31
      m4_desired.elements[9],   // m32
      m4_desired.elements[10]   // m33
    );

    // DEBUG: Log to verify extraction and extract yaw from quaternion
    if (Math.random() < 0.02) {
      const q_des = result.desiredOrientation;
      const desiredYawFromQuat = Math.atan2(
        2 * (q_des.w * q_des.z + q_des.x * q_des.y),
        1 - 2 * (q_des.y * q_des.y + q_des.z * q_des.z)
      );
      
      // Extract actual yaw from drone state
      const q_actual = state.orientationQuat;
      const actualYaw = Math.atan2(
        2 * (q_actual.w * q_actual.z + q_actual.x * q_actual.y),
        1 - 2 * (q_actual.y * q_actual.y + q_actual.z * q_actual.z)
      );
      
      console.log(`[STATIONARY_CTRL] Yaw: actual=${(actualYaw * 180 / Math.PI).toFixed(1)}°, desired=${(desiredYawFromQuat * 180 / Math.PI).toFixed(1)}° | ` +
                  `b3_desired: (${b3_desired.x.toFixed(3)}, ${b3_desired.y.toFixed(3)}, ${b3_desired.z.toFixed(3)})`);
    }

    // Compute meaningful attitude error: angle between thrust vectors
    const thrustDot = b3_actual.dot(b3_desired);
    const thrustErrorDeg = (Math.acos(THREE.MathUtils.clamp(thrustDot, -1, 1)) * 180) / Math.PI;

    // Attach diagnostics to result for HUD/logging
    result.diagnostics = {
      thrustDirection_actual: b3_actual,
      thrustDirection_desired: b3_desired,
      thrustErrorDeg: thrustErrorDeg,
      positionError: target.position.clone().sub(state.position),
    };

    return result;
  }
}
