// Race-specific controller that extends ETH controller with trajectory following
import { EthController } from '../stationary/eth_controller.js';

export class RaceController extends EthController {
  constructor(params) {
    super(params);
    this.trajectory = null;
    this.simTime = 0;
    this.gateIndex = 0;
  }

  // Wrapper to match StationaryController API
  updateGains({ kp, ki, kd }) {
    this.updatePositionGains({ kp, ki, kd });
  }

  setTrajectory(trajectory) {
    this.trajectory = trajectory;
  }

  reset() {
    super.reset();
    this.simTime = 0;
    this.gateIndex = 0;
  }

  update(dt) {
    // Advance simulation time
    this.simTime += dt;
  }

  // Sample the target from trajectory based on current simulation time
  getTarget() {
    if (!this.trajectory) {
      return {
        position: new THREE.Vector3(0, 0, 0.6),
        velocity: new THREE.Vector3(0, 0, 0),
        acceleration: new THREE.Vector3(0, 0, 0),
        yaw: 0,
      };
    }

    // Loop the trajectory - handle both ReferenceTrajectory and TimeOptimalTrajectory
    let target;
    if (typeof this.trajectory.getStateAtTime === 'function') {
      target = this.trajectory.getStateAtTime(this.simTime % (this.trajectory.totalDuration || this.trajectory.totalTime));
    } else if (typeof this.trajectory.sample === 'function') {
      target = this.trajectory.sample(this.simTime % this.trajectory.totalTime);
    } else {
      return {
        position: new THREE.Vector3(0, 0, 0.6),
        velocity: new THREE.Vector3(0, 0, 0),
        acceleration: new THREE.Vector3(0, 0, 0),
        yaw: 0,
      };
    }
    return {
      position: target.position,
      velocity: target.velocity || new THREE.Vector3(0, 0, 0),
      acceleration: target.acceleration || new THREE.Vector3(0, 0, 0),
      yaw: target.yaw || 0,
    };
  }

  // Check if drone passed through a gate
  updateGateIndex(position, waypoints, gateThreshold = 1.5) {
    if (this.gateIndex >= waypoints.length - 1) {
      return false; // Completed
    }

    const nextGatePos = waypoints[this.gateIndex + 1];
    if (position.distanceTo(nextGatePos) < gateThreshold) {
      this.gateIndex++;
      return true;
    }
    return false;
  }

  getGateIndex() {
    return this.gateIndex;
  }
}
