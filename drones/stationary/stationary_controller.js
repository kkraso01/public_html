import { EthController } from './eth_controller.js';

export class StationaryController {
  constructor(params = {}) {
    this.params = params;
    this.eth = new EthController(params);
    this.useFOPID = false; // kept for HUD compatibility
    this.lastIntegrationMs = 0; // placeholder for legacy HUD
  }

  reset() {
    this.eth.reset();
  }

  updateGains({ kp, ki, kd }) {
    this.eth.updatePositionGains({ kp, ki, kd });
  }

  toggleFOPID() {
    // Legacy control panel hook; no-op for cascaded controller
    this.useFOPID = false;
  }

  compute(state, dt) {
    // Fixed hover target slightly above the origin
    const target = {
      position: new THREE.Vector3(0, 0.6, 0),
      velocity: new THREE.Vector3(0, 0, 0),
      acceleration: new THREE.Vector3(0, 0, 0),
      yaw: 0,
    };

    return this.eth.computeControl(state, target, dt);
  }
}
