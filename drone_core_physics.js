// 6-DOF quadrotor dynamics with RK4 integration.
// The design mirrors lightweight research simulators (RotorS / Flightmare style)
// but is intentionally browser-friendly and dependency free beyond Three.js.

import { clamp } from './drone_math.js';

const GRAVITY = new THREE.Vector3(0, -9.81, 0);

export class Quadrotor {
  constructor(params = {}) {
    this.params = Object.assign(
      {
        mass: 1.05, // kg
        inertia: new THREE.Vector3(0.021, 0.021, 0.04), // diag inertia
        armLength: 0.11,
        dragCoeff: 0.15,
        motorTimeConstant: 0.025,
        maxThrust: 28, // N, total across motors
        maxTorque: new THREE.Vector3(1.8, 1.8, 0.9),
      },
      params,
    );

    this.state = {
      position: new THREE.Vector3(),
      velocity: new THREE.Vector3(),
      quaternion: new THREE.Quaternion(),
      omega: new THREE.Vector3(),
    };

    this.motorThrusts = [0, 0, 0, 0];
  }

  reset(initialState = {}) {
    this.state.position.copy(initialState.position || new THREE.Vector3());
    this.state.velocity.copy(initialState.velocity || new THREE.Vector3());
    this.state.quaternion.copy(initialState.quaternion || new THREE.Quaternion());
    this.state.omega.copy(initialState.omega || new THREE.Vector3());
    this.motorThrusts = [0, 0, 0, 0];
  }

  _motorMix(controlInput) {
    if (controlInput.motorThrusts) return controlInput.motorThrusts.slice(0, 4);
    const L = this.params.armLength;
    const k = 0.016; // yaw torque coefficient
    const T = clamp(controlInput.thrust ?? 0, 0, this.params.maxThrust);
    const tau = controlInput.torque || new THREE.Vector3();
    // Simple X configuration mixer
    const m0 = 0.25 * T - tau.y / (2 * L) + tau.z / (4 * k);
    const m1 = 0.25 * T + tau.x / (2 * L) - tau.z / (4 * k);
    const m2 = 0.25 * T + tau.y / (2 * L) + tau.z / (4 * k);
    const m3 = 0.25 * T - tau.x / (2 * L) - tau.z / (4 * k);
    return [m0, m1, m2, m3].map((m) => clamp(m, 0, this.params.maxThrust));
  }

  _derivatives(state, motorThrusts) {
    const mass = this.params.mass;
    const inertia = this.params.inertia;
    const posDot = state.velocity.clone();

    const R = new THREE.Matrix3().setFromMatrix4(new THREE.Matrix4().makeRotationFromQuaternion(state.quaternion));
    const thrustBody = new THREE.Vector3(0, 0, motorThrusts.reduce((a, b) => a + b, 0));
    const thrustWorld = thrustBody.clone().applyMatrix3(R);

    const drag = state.velocity.clone().multiplyScalar(-this.params.dragCoeff);
    const velDot = thrustWorld.clone().multiplyScalar(1 / mass).add(GRAVITY).add(drag);

    const omega = state.omega.clone();
    const inertiaVec = new THREE.Vector3(inertia.x * omega.x, inertia.y * omega.y, inertia.z * omega.z);
    // Torque from motor differentials
    const tau = this._torqueFromMotors(motorThrusts);
    const omegaCrossIomega = omega.clone().cross(inertiaVec);
    const omegaDot = new THREE.Vector3(
      (tau.x - omegaCrossIomega.x) / inertia.x,
      (tau.y - omegaCrossIomega.y) / inertia.y,
      (tau.z - omegaCrossIomega.z) / inertia.z,
    );

    // Quaternion derivative: 0.5 * q ⊗ [0, omega]
    const q = state.quaternion;
    const qDot = new THREE.Quaternion(
      0.5 * (q.w * omega.x + q.y * omega.z - q.z * omega.y),
      0.5 * (q.w * omega.y - q.x * omega.z + q.z * omega.x),
      0.5 * (q.w * omega.z + q.x * omega.y - q.y * omega.x),
      0.5 * (-q.x * omega.x - q.y * omega.y - q.z * omega.z),
    );

    return { posDot, velDot, qDot, omegaDot };
  }

  _torqueFromMotors(thrusts) {
    const L = this.params.armLength;
    const k = 0.016;
    // X configuration mapping back from motor thrusts to torques
    const roll = (thrusts[1] - thrusts[3]) * L;
    const pitch = (thrusts[2] - thrusts[0]) * L;
    const yaw = (thrusts[0] - thrusts[1] + thrusts[2] - thrusts[3]) * k;
    return new THREE.Vector3(roll, pitch, yaw).clamp(
      this.params.maxTorque.clone().multiplyScalar(-1),
      this.params.maxTorque.clone(),
    );
  }

  _rk4Step(controlInput, dt) {
    const motorCmd = this._motorMix(controlInput);

    // Update motors with first-order lag
    const motorActual = this.motorThrusts.map((m, i) => {
      const alpha = dt / Math.max(this.params.motorTimeConstant, 1e-3);
      return m + alpha * (motorCmd[i] - m);
    });

    const s0 = {
      position: this.state.position.clone(),
      velocity: this.state.velocity.clone(),
      quaternion: this.state.quaternion.clone(),
      omega: this.state.omega.clone(),
    };

    const k1 = this._derivatives(s0, motorActual);

    const s1 = {
      position: s0.position.clone().addScaledVector(k1.posDot, dt * 0.5),
      velocity: s0.velocity.clone().addScaledVector(k1.velDot, dt * 0.5),
      quaternion: s0.quaternion.clone().add(new THREE.Quaternion(
        k1.qDot.x * dt * 0.5,
        k1.qDot.y * dt * 0.5,
        k1.qDot.z * dt * 0.5,
        k1.qDot.w * dt * 0.5,
      )).normalize(),
      omega: s0.omega.clone().addScaledVector(k1.omegaDot, dt * 0.5),
    };

    const k2 = this._derivatives(s1, motorActual);

    const s2 = {
      position: s0.position.clone().addScaledVector(k2.posDot, dt * 0.5),
      velocity: s0.velocity.clone().addScaledVector(k2.velDot, dt * 0.5),
      quaternion: s0.quaternion.clone().add(new THREE.Quaternion(
        k2.qDot.x * dt * 0.5,
        k2.qDot.y * dt * 0.5,
        k2.qDot.z * dt * 0.5,
        k2.qDot.w * dt * 0.5,
      )).normalize(),
      omega: s0.omega.clone().addScaledVector(k2.omegaDot, dt * 0.5),
    };

    const k3 = this._derivatives(s2, motorActual);

    const s3 = {
      position: s0.position.clone().addScaledVector(k3.posDot, dt),
      velocity: s0.velocity.clone().addScaledVector(k3.velDot, dt),
      quaternion: s0.quaternion.clone().add(new THREE.Quaternion(
        k3.qDot.x * dt,
        k3.qDot.y * dt,
        k3.qDot.z * dt,
        k3.qDot.w * dt,
      )).normalize(),
      omega: s0.omega.clone().addScaledVector(k3.omegaDot, dt),
    };

    const k4 = this._derivatives(s3, motorActual);

    this.state.position.addScaledVector(
      k1.posDot.clone().add(k2.posDot.clone().multiplyScalar(2)).add(k3.posDot.clone().multiplyScalar(2)).add(k4.posDot),
      dt / 6,
    );
    this.state.velocity.addScaledVector(
      k1.velDot.clone().add(k2.velDot.clone().multiplyScalar(2)).add(k3.velDot.clone().multiplyScalar(2)).add(k4.velDot),
      dt / 6,
    );
    const dq = new THREE.Quaternion(
      (k1.qDot.x + 2 * k2.qDot.x + 2 * k3.qDot.x + k4.qDot.x) * dt / 6,
      (k1.qDot.y + 2 * k2.qDot.y + 2 * k3.qDot.y + k4.qDot.y) * dt / 6,
      (k1.qDot.z + 2 * k2.qDot.z + 2 * k3.qDot.z + k4.qDot.z) * dt / 6,
      (k1.qDot.w + 2 * k2.qDot.w + 2 * k3.qDot.w + k4.qDot.w) * dt / 6,
    );
    this.state.quaternion.add(dq).normalize();
    this.state.omega.addScaledVector(
      k1.omegaDot.clone().add(k2.omegaDot.clone().multiplyScalar(2)).add(k3.omegaDot.clone().multiplyScalar(2)).add(k4.omegaDot),
      dt / 6,
    );

    this.motorThrusts = motorActual;
  }

  step(controlInput, dt) {
    const substeps = Math.max(1, Math.ceil(dt / 0.01));
    const h = dt / substeps;
    for (let i = 0; i < substeps; i++) {
      this._rk4Step(controlInput, h);
    }
  }

  getState() {
    return {
      position: this.state.position.clone(),
      velocity: this.state.velocity.clone(),
      quaternion: this.state.quaternion.clone(),
      omega: this.state.omega.clone(),
    };
  }
}
