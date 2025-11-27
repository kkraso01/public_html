import { DronePhysicsEngine } from '../physics/drone_physics_engine.js';

function clamp(x, min, max) {
  return Math.min(Math.max(x, min), max);
}

function binomial(a, k) {
  if (k === 0) return 1;
  let coeff = 1;
  for (let i = 0; i < k; i++) {
    coeff *= (a - i) / (i + 1);
  }
  return coeff;
}

class FractionalIntegrator {
  constructor(alpha, dt) {
    this.alpha = alpha; // fractional order 0 < α < 1
    this.dt = dt;
    this.history = [];
  }

  update(error) {
    this.history.unshift(error);
    let sum = 0;
    const a = this.alpha;

    for (let k = 0; k < this.history.length; k++) {
      const coeff = Math.pow(-1, k) * binomial(a, k);
      sum += coeff * this.history[k];
      if (k > 120) break; // performance safeguard
    }
    return sum * Math.pow(this.dt, -a);
  }
}

export class StationaryController {
  constructor(params = {}) {
    this.params = Object.assign({
      kp: 4.0,
      ki: 1.4,
      kd: 2.2,
      yaw: 0,
      fractionalAlpha: 0.8,
      dt: 1 / 240,
    }, params);

    this.integral = new THREE.Vector3();
    this.integrators = {
      x: new FractionalIntegrator(this.params.fractionalAlpha, this.params.dt),
      y: new FractionalIntegrator(this.params.fractionalAlpha, this.params.dt),
      z: new FractionalIntegrator(this.params.fractionalAlpha, this.params.dt),
    };
    this.useFOPID = true;
    this.fractionalDisabled = false;
    this.lastIntegrationMs = 0;
  }

  updateGains({ kp, ki, kd }) {
    if (typeof kp === 'number') this.params.kp = kp;
    if (typeof ki === 'number') this.params.ki = ki;
    if (typeof kd === 'number') this.params.kd = kd;
  }

  toggleFOPID(enabled) {
    this.useFOPID = enabled && !this.fractionalDisabled;
  }

  _desiredOrientationFromThrust(thrustVec, yaw) {
    const zBody = thrustVec.lengthSq() > 1e-6 ? thrustVec.clone().normalize() : new THREE.Vector3(0, 1, 0);
    const xC = new THREE.Vector3(Math.sin(yaw), 0, Math.cos(yaw));
    const yBody = zBody.clone().cross(xC).normalize();
    if (yBody.lengthSq() < 1e-6) return new THREE.Quaternion();
    const xBody = yBody.clone().cross(zBody).normalize();
    const m = new THREE.Matrix4();
    m.makeBasis(xBody, yBody, zBody);
    const q = new THREE.Quaternion();
    q.setFromRotationMatrix(m);
    return q;
  }

  _fractionalIntegral(error) {
    const start = performance.now();
    const integral = new THREE.Vector3(
      this.integrators.x.update(error.x),
      this.integrators.y.update(error.y),
      this.integrators.z.update(error.z),
    );
    this.lastIntegrationMs = performance.now() - start;
    if (this.lastIntegrationMs > 4) {
      this.fractionalDisabled = true;
      this.useFOPID = false;
      console.warn('[FOPID] Disabled: browser too slow, using classical PID.');
    }
    return integral;
  }

  _classicalIntegral(error, dt) {
    this.integral.add(error.clone().multiplyScalar(dt));
    this.integral.clampLength(-1.5, 1.5);
    return this.integral.clone();
  }

  compute(state, target, dt) {
    const params = this.params;
    const posErr = target.position.clone().sub(state.position);
    const velErr = target.velocity ? target.velocity.clone().sub(state.velocity) : state.velocity.clone().multiplyScalar(-1);

    let integral;
    if (this.useFOPID && !this.fractionalDisabled) {
      const fractional = this._fractionalIntegral(posErr);
      const fractionalTerm = fractional.x + fractional.y + fractional.z;
      if (!Number.isFinite(fractionalTerm)) {
        this.useFOPID = false; // fall back to PID
        this.fractionalDisabled = true;
        integral = this._classicalIntegral(posErr, dt);
      } else {
        integral = fractional;
      }
    } else {
      integral = this._classicalIntegral(posErr, dt);
    }

    const desiredAcc = new THREE.Vector3(
      params.kp * posErr.x + params.ki * integral.x + params.kd * velErr.x,
      params.kp * posErr.y + params.ki * integral.y + params.kd * velErr.y,
      params.kp * posErr.z + params.ki * integral.z + params.kd * velErr.z,
    );
    desiredAcc.clampLength(0, 6.5);

    const mass = target.mass || 1.05;
    const thrustVector = desiredAcc.add(new THREE.Vector3(0, 9.81, 0)).multiplyScalar(mass);
    thrustVector.y = Math.max(thrustVector.y, 0);

    const desiredOrientation = this._desiredOrientationFromThrust(thrustVector, params.yaw);
    const kp_att = state.kp_att || (state.params?.kp_att ?? DronePhysicsEngine?.prototype?.params?.kp_att ?? 6.0);
    const kd_att = state.kd_att || (state.params?.kd_att ?? DronePhysicsEngine?.prototype?.params?.kd_att ?? 2.0);
    const currentQuat = state.orientationQuat || state.orientation;
    const qError = desiredOrientation.clone().multiply(currentQuat.clone().invert());
    const errorAxis = new THREE.Vector3(qError.x, qError.y, qError.z).multiplyScalar(2.0);
    const torque = errorAxis.multiplyScalar(kp_att).add(state.angularVelocity.clone().multiplyScalar(-kd_att));

    const paramsPhys = state.params || target.params || DronePhysicsEngine?.prototype?.params || {};
    const kF = paramsPhys.kF || 1e-3;
    const kM = paramsPhys.kM || kF * 0.02;
    const L = paramsPhys.armLength || 0.046;
    const kYaw = kM / Math.max(kF, 1e-9);

    const thrustMag = clamp(thrustVector.length(), 0, (paramsPhys.mass || mass) * 25);
    const m0 = 0.25 * thrustMag - torque.y / (2 * L) + torque.z / (4 * kYaw);
    const m1 = 0.25 * thrustMag + torque.x / (2 * L) - torque.z / (4 * kYaw);
    const m2 = 0.25 * thrustMag + torque.y / (2 * L) + torque.z / (4 * kYaw);
    const m3 = 0.25 * thrustMag - torque.x / (2 * L) - torque.z / (4 * kYaw);
    const thrusts = [m0, m1, m2, m3].map((m) => clamp(m, 0, thrustMag));
    const rpms = thrusts.map((t) => Math.sqrt(Math.max(t, 0) / Math.max(kF, 1e-9)));
    const commands = rpms.map((r) => {
      const minRPM = paramsPhys.minRPM || 0;
      const maxRPM = paramsPhys.maxRPM || paramsPhys.rpmMax || 25000;
      const clamped = clamp(r, minRPM, maxRPM);
      return clamp((clamped - minRPM) / (maxRPM - minRPM), 0, 1);
    });

    return { commands, desiredOrientation };
  }
}

export { FractionalIntegrator, binomial };
