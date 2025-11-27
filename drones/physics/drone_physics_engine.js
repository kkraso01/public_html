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
    this.params = Object.assign(
      {
        floorHeight: 0.05,
        ceilingHeight: 5.0,
        minRPM: 0,
        idleRPM: 1800,
        maxRPM: 25000,
        kp_att: 8.0,
        kd_att: 2.5,
        motorForceScale: 1.1,
      },
      CRAZYFLIE_PARAMS,
      params,
    );

    this.params.maxRPM = this.params.maxRPM || this.params.rpmMax || 25000;

    // retune kF so that a 0.5 command (hover) produces approximately mass * g thrust
    const yawRatio = this.params.kM / Math.max(this.params.kF, 1e-9);
    const hoverRPM = (this.params.minRPM || 0) + 0.5 * (this.params.maxRPM - (this.params.minRPM || 0));
    const perMotorHover = (this.params.mass * 9.81) / 4;
    const tunedKF = perMotorHover / (hoverRPM * hoverRPM);
    this.params.kF = tunedKF;
    this.params.kM = tunedKF * yawRatio;
    this.params.hoverRPM = hoverRPM;
    this.params.hoverCommand = (hoverRPM - (this.params.idleRPM || 0)) / (this.params.maxRPM - (this.params.idleRPM || 0));

    this.motor = new MotorModel({ rpmMax: this.params.maxRPM, timeConstant: this.params.motorTimeConstant });
    this.state = {
      position: new THREE.Vector3(),
      velocity: new THREE.Vector3(),
      orientation: new THREE.Quaternion(),
      orientationQuat: new THREE.Quaternion(),
      angularVelocity: new THREE.Vector3(),
      motorRPM: [0, 0, 0, 0],
      totalThrust: 0,
    };
    this.desiredOrientationQuat = new THREE.Quaternion();
    this.reset();
  }

  reset(state = {}) {
    const startPos = state.position || new THREE.Vector3();
    startPos.y = Math.max(startPos.y, (this.params.floorHeight || 0) + 0.2);
    this.state.position.copy(startPos);
    this.state.velocity.copy(state.velocity || new THREE.Vector3());
    const q = state.orientation || state.orientationQuat || new THREE.Quaternion();
    this.state.orientation.copy(q);
    this.state.orientationQuat.copy(q);
    this.state.angularVelocity.copy(state.angularVelocity || new THREE.Vector3());
    this.motor.omegas = [0, 0, 0, 0];
    this.motor.commands = [0, 0, 0, 0];
    this.state.motorRPM = [0, 0, 0, 0];
    this.state.totalThrust = 0;
    this.desiredOrientationQuat.identity();
  }

  applyMotorCommands(u1, u2, u3, u4) {
    const commands = [u1, u2, u3, u4].map((command) => {
      const rpmCommand = THREE.MathUtils.clamp(command, 0, 1);
      const idle = this.params.idleRPM || 0;
      let rpm = idle + rpmCommand * (this.params.maxRPM - idle);
      rpm = Math.max(this.params.minRPM, Math.min(rpm, this.params.maxRPM));
      return rpm;
    });
    this.motor.setCommands(commands[0], commands[1], commands[2], commands[3]);
  }

  getState() {
    return {
      position: this.state.position.clone(),
      velocity: this.state.velocity.clone(),
      orientation: this.state.orientation.clone(),
      orientationQuat: this.state.orientationQuat.clone(),
      angularVelocity: this.state.angularVelocity.clone(),
      motorRPM: this.motor.omegas.slice(),
      totalThrust: this.state.totalThrust,
    };
  }

  step(dt) {
    if (dt <= 0) return;
    const omegas = this.motor.step(dt); // rpm
    this.state.motorRPM = omegas.slice();
    const thrusts = omegas.map((w) => this.params.kF * w * w * (this.params.motorForceScale || 1));
    const totalThrust = thrusts.reduce((a, b) => a + b, 0);
    
    const rotationMatrix = new THREE.Matrix3().setFromMatrix4(
      new THREE.Matrix4().makeRotationFromQuaternion(this.state.orientation),
    );
    const thrustWorld = new THREE.Vector3(0, 0, totalThrust).applyMatrix3(rotationMatrix);

    if (this.state.position.y < 0.25) {
      const h = Math.max(this.state.position.y, 0.01);
      const groundEffect = 0.2 / (h * h);
      thrustWorld.y += groundEffect;
      this.state.totalThrust = totalThrust + groundEffect;
    } else {
      this.state.totalThrust = totalThrust;
    }

    const drag = this.state.velocity.clone().multiplyScalar(-this.params.dragLinear);
    const acc = thrustWorld.clone().multiplyScalar(1 / this.params.mass).add(GRAVITY).add(drag);

    this.state.velocity.add(acc.multiplyScalar(dt));
    this.state.position.add(this.state.velocity.clone().multiplyScalar(dt));

    if (this.state.position.y < this.params.floorHeight) {
      this.state.position.y = this.params.floorHeight;
      if (this.state.velocity.y < 0) this.state.velocity.y = 0;
    }
    if (this.state.position.y > this.params.ceilingHeight) {
      this.state.position.y = this.params.ceilingHeight;
      if (this.state.velocity.y > 0) this.state.velocity.y = 0;
    }

    const worldMinX = -20;
    const worldMaxX = 20;
    const worldMinZ = -20;
    const worldMaxZ = 20;

    this.state.position.x = Math.max(worldMinX, Math.min(worldMaxX, this.state.position.x));
    this.state.position.z = Math.max(worldMinZ, Math.min(worldMaxZ, this.state.position.z));

    const tau = this._torqueFromThrusts(thrusts);
    const desiredQuat = this.desiredOrientationQuat || new THREE.Quaternion();
    const currentQuat = this.state.orientationQuat || this.state.orientation;
    const qError = desiredQuat.clone().multiply(currentQuat.clone().invert());
    if (qError.w < 0) {
      qError.x *= -1;
      qError.y *= -1;
      qError.z *= -1;
      qError.w *= -1;
    }
    const errorAxis = new THREE.Vector3(qError.x, qError.y, qError.z).multiplyScalar(2.0);
    const attTorque = errorAxis
      .multiplyScalar(this.params.kp_att)
      .add(this.state.angularVelocity.clone().multiplyScalar(-this.params.kd_att));
    const torque = tau.add(attTorque);
    const inertia = this.params.inertia;
    const inertiaVec = new THREE.Vector3(
      inertia.x * this.state.angularVelocity.x,
      inertia.y * this.state.angularVelocity.y,
      inertia.z * this.state.angularVelocity.z,
    );
    const omegaCrossIomega = this.state.angularVelocity.clone().cross(inertiaVec);
    const omegaDot = new THREE.Vector3(
      (torque.x - omegaCrossIomega.x) / inertia.x,
      (torque.y - omegaCrossIomega.y) / inertia.y,
      (torque.z - omegaCrossIomega.z) / inertia.z,
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
    this.state.orientationQuat.copy(this.state.orientation);
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
