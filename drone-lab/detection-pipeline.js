export class DroneDetectionPipeline {
  constructor({ detector = null, tracker = null, confidenceThreshold = 0.4 } = {}) {
    this.detector = detector;
    this.tracker = tracker;
    this.confidenceThreshold = confidenceThreshold;
  }

  async processFrame(frame) {
    const detections = this.detector ? await this.detector(frame) : [];
    const filtered = detections.filter((d) => d.confidence === undefined || d.confidence >= this.confidenceThreshold);
    const tracks = this.tracker ? this.tracker(filtered) : filtered;
    return { detections: filtered, tracks };
  }

  attachDetector(detectorFn) {
    this.detector = detectorFn;
  }

  attachTracker(trackerFn) {
    this.tracker = trackerFn;
  }
}
