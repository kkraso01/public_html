import { DroneSLAM } from './slam-core.js';
import { DroneDetectionPipeline } from './detection-pipeline.js';
import { TelemetryBus } from './telemetry-bus.js';

export class DroneLabController {
  constructor({ gridSize, resolution, confidenceThreshold } = {}) {
    this.bus = new TelemetryBus();
    this.slam = new DroneSLAM({ gridSize, resolution });
    this.detector = new DroneDetectionPipeline({ confidenceThreshold });
  }

  on(event, handler) {
    return this.bus.on(event, handler);
  }

  attachDetector(fn) {
    this.detector.attachDetector(fn);
  }

  attachTracker(fn) {
    this.detector.attachTracker(fn);
  }

  step({ odometry = { dx: 0, dy: 0, dtheta: 0 }, rangeReadings = [], fov }) {
    const pose = this.slam.integrateOdometry(odometry);
    const map = this.slam.integrateObservation({ rangeReadings, fov });
    this.bus.emit('pose', pose);
    this.bus.emit('map', map);
  }

  async processFrame(frame) {
    const { detections, tracks } = await this.detector.processFrame(frame);
    this.bus.emit('detections', detections);
    this.bus.emit('tracks', tracks);
    return { detections, tracks };
  }
}
