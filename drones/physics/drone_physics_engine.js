import { MotorModel } from './motor_model.js';
import { CRAZYFLIE_PARAMS } from './params_crazyflie.js';

const GRAVITY = new THREE.Vector3(0, -9.81, 0);

function quatMultiply(q, r) {
  return new THREE.Quaternion(
    q.w * r.x + q.x * r.w + q.y * r.z - q.z * r.y,
    q.w * r.y - q.x * r.z + q.y * r.w + q.z * r.x,
    q.w * r.z + q.x * r.y - q.y * r.x + q.z * r.w,
    q.w * r.w - q.x * r.x - q.y * r.y - q.z * r.z,
  );
}

export class DronePhysicsEngine {
  constructor(params = {}) {
    this.params = Object.assign({}, CRAZYFLIE_PARAMS, params);
    this.motor = new MotorModel({ rpmMax: this.params.rpmMax, timeConstant: this.params.motorTimeConstant });
    this.state = {
      position: new THREE.Vector3(),
      velocity: new THREE.Vector3(),
      orientation: new THREE.Quaternion(),
      angularVelocity: new THREE.Vector3(),
    };
    this.reset();
  }

  reset(state = {}) {
    this.state.position.copy(state.position || new THREE.Vector3());
    this.state.velocity.copy(state.velocity || new THREE.Vector3());
    this.state.orientation.copy(state.orientation || new THREE.Quaternion());
    this.state.angularVelocity.copy(state.angularVelocity || new THREE.Vector3());
    this.motor.omegas = [0, 0, 0, 0];
    this.motor.commands = [0, 0, 0, 0];
  }

  applyMotorCommands(u1, u2, u3, u4) {
    this.motor.setCommands(u1, u2, u3, u4);
  }

  getState() {
    return {
      position: this.state.position.clone(),
      velocity: this.state.velocity.clone(),
      orientation: this.state.orientation.clone(),
      angularVelocity: this.state.angularVelocity.clone(),
      motorRPM: this.motor.omegas.slice(),
    };
  }

  step(dt) {
    if (dt <= 0) return;
    const omegas = this.motor.step(dt); // rpm
    const thrusts = omegas.map((w) => this.params.kF * w * w);
    const totalThrust = thrusts.reduce((a, b) => a + b, 0);

    const rotationMatrix = new THREE.Matrix3().setFromMatrix4(
      new THREE.Matrix4().makeRotationFromQuaternion(this.state.orientation),
    );
    const thrustWorld = new THREE.Vector3(0, 0, totalThrust).applyMatrix3(rotationMatrix);

    const drag = this.state.velocity.clone().multiplyScalar(-this.params.dragLinear);
    const acc = thrustWorld.clone().multiplyScalar(1 / this.params.mass).add(GRAVITY).add(drag);

    this.state.velocity.add(acc.multiplyScalar(dt));
    this.state.position.add(this.state.velocity.clone().multiplyScalar(dt));

    const tau = this._torqueFromThrusts(thrusts);
    const inertia = this.params.inertia;
    const inertiaVec = new THREE.Vector3(
      inertia.x * this.state.angularVelocity.x,
      inertia.y * this.state.angularVelocity.y,
      inertia.z * this.state.angularVelocity.z,
    );
    const omegaCrossIomega = this.state.angularVelocity.clone().cross(inertiaVec);
    const omegaDot = new THREE.Vector3(
      (tau.x - omegaCrossIomega.x) / inertia.x,
      (tau.y - omegaCrossIomega.y) / inertia.y,
      (tau.z - omegaCrossIomega.z) / inertia.z,
    );

    this.state.angularVelocity.add(omegaDot.multiplyScalar(dt));
    const qDot = quatMultiply(
      this.state.orientation,
      new THREE.Quaternion(
        0.5 * this.state.angularVelocity.x,
        0.5 * this.state.angularVelocity.y,
        0.5 * this.state.angularVelocity.z,
        0,
      ),
    );
    this.state.orientation.x += qDot.x * dt;
    this.state.orientation.y += qDot.y * dt;
    this.state.orientation.z += qDot.z * dt;
    this.state.orientation.w += qDot.w * dt;
    this.state.orientation.normalize();
  }

  _torqueFromThrusts(thrusts) {
    const L = this.params.armLength;
    const yawDir = [1, -1, 1, -1];
    const roll = (thrusts[1] - thrusts[3]) * L;
    const pitch = (thrusts[2] - thrusts[0]) * L;
    const yaw = thrusts.reduce((sum, t, i) => sum + yawDir[i] * t, 0) * (this.params.kM / Math.max(this.params.kF, 1e-9));
    return new THREE.Vector3(roll, pitch, yaw);
  }
}
