// Geometric SE(3)-style controller for quadrotor trajectory tracking.
// This controller follows the structure popularized by Lee et al., providing thrust and body torques
// from desired acceleration / yaw commands.

import { clamp } from '../utils/drone_math.js';

export class GeometricController {
  constructor(params = {}) {
    this.params = Object.assign(
      {
        mass: 1.05,
        inertia: new THREE.Vector3(0.021, 0.021, 0.04),
        kpPos: new THREE.Vector3(6, 6, 8),
        kdPos: new THREE.Vector3(4.5, 4.5, 5),
        kR: new THREE.Vector3(6, 6, 4),
        kOmega: new THREE.Vector3(0.25, 0.25, 0.2),
        maxThrust: 28,
        maxOmega: 8.0, // rad/s
      },
      params,
    );
  }

  computeControl(state, desired) {
    const pos = state.position;
    const vel = state.velocity;
    const q = state.quaternion;
    const omega = state.omega;

    const ePos = desired.position.clone().sub(pos);
    const eVel = desired.velocity.clone().sub(vel);

    // Desired acceleration from outer loop
    const accCmd = desired.acceleration
      .clone()
      .add(new THREE.Vector3(
        this.params.kpPos.x * ePos.x + this.params.kdPos.x * eVel.x,
        this.params.kpPos.y * ePos.y + this.params.kdPos.y * eVel.y,
        this.params.kpPos.z * ePos.z + this.params.kdPos.z * eVel.z,
      ))
      .add(new THREE.Vector3(0, 9.81, 0)); // gravity compensation

    // Desired heading and body axes
    const zbDes = accCmd.clone().normalize();
    const yaw = desired.yaw || 0;
    const xc = new THREE.Vector3(Math.cos(yaw), 0, Math.sin(yaw));
    const ybDes = zbDes.clone().cross(xc).normalize();
    const xbDes = ybDes.clone().cross(zbDes).normalize();

    const Rdes = new THREE.Matrix3().set(
      xbDes.x, ybDes.x, zbDes.x,
      xbDes.y, ybDes.y, zbDes.y,
      xbDes.z, ybDes.z, zbDes.z,
    );

    const R = new THREE.Matrix3().setFromMatrix4(new THREE.Matrix4().makeRotationFromQuaternion(q));
    const Rt = new THREE.Matrix3().copy(R).transpose();
    const Rerr = new THREE.Matrix3().multiplyMatrices(Rt, Rdes);
    const eR = new THREE.Vector3(
      Rerr.elements[7] - Rerr.elements[5],
      Rerr.elements[2] - Rerr.elements[6],
      Rerr.elements[3] - Rerr.elements[1],
    ).multiplyScalar(0.5);

    // Desired angular velocity set to zero; could be extended with yaw rate feedforward
    const eOmega = omega.clone();

    const torque = new THREE.Vector3(
      -this.params.kR.x * eR.x - this.params.kOmega.x * eOmega.x,
      -this.params.kR.y * eR.y - this.params.kOmega.y * eOmega.y,
      -this.params.kR.z * eR.z - this.params.kOmega.z * eOmega.z,
    );

    const basisX = new THREE.Vector3();
    const basisY = new THREE.Vector3();
    const basisZ = new THREE.Vector3();
    R.extractBasis(basisX, basisY, basisZ);
    const thrust = clamp(accCmd.dot(basisZ) * this.params.mass, 0, this.params.maxThrust);

    return { thrust, torque };
  }
}

// Lightweight altitude controller useful for hovering in the cave demo.
export class AltitudeHoldController {
  constructor(params = {}) {
    this.params = Object.assign(
      {
        mass: 1.05,
        kp: 6,
        kd: 4,
        maxThrust: 28,
      },
      params,
    );
  }

  computeThrust(currentZ, currentVz, desiredZ, desiredVz = 0) {
    const e = desiredZ - currentZ;
    const ev = desiredVz - currentVz;
    const acc = this.params.kp * e + this.params.kd * ev + 9.81;
    return clamp(acc * this.params.mass, 0, this.params.maxThrust);
  }
}
