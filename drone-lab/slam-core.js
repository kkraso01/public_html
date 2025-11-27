export class DroneSLAM {
  constructor({ gridSize = 50, resolution = 0.1, processNoise = 0.01 } = {}) {
    this.gridSize = gridSize;
    this.resolution = resolution; // meters per cell
    this.processNoise = processNoise;
    this.pose = { x: 0, y: 0, theta: 0 };
    this.occupancy = this.#createGrid(gridSize);
  }

  reset({ pose = { x: 0, y: 0, theta: 0 } } = {}) {
    this.pose = { ...pose };
    this.occupancy = this.#createGrid(this.gridSize);
  }

  integrateOdometry({ dx = 0, dy = 0, dtheta = 0 }) {
    this.pose.x += dx + this.#noise();
    this.pose.y += dy + this.#noise();
    this.pose.theta += dtheta + this.#noise();
    return { ...this.pose };
  }

  integrateObservation({ rangeReadings = [], fov = Math.PI / 2 }) {
    if (!rangeReadings.length) return this.occupancy;

    const angleIncrement = rangeReadings.length > 1 ? fov / (rangeReadings.length - 1) : 0;
    rangeReadings.forEach((r, idx) => {
      const angle = this.pose.theta - fov / 2 + idx * angleIncrement;
      const hit = this.#polarToGrid({ r, angle });
      if (hit) this.#markOccupied(hit.x, hit.y);
    });
    return this.occupancy;
  }

  #polarToGrid({ r, angle }) {
    const xWorld = this.pose.x + r * Math.cos(angle);
    const yWorld = this.pose.y + r * Math.sin(angle);
    const xIdx = Math.round(xWorld / this.resolution + this.gridSize / 2);
    const yIdx = Math.round(yWorld / this.resolution + this.gridSize / 2);
    if (this.#inBounds(xIdx, yIdx)) return { x: xIdx, y: yIdx };
    return null;
  }

  #markOccupied(x, y) {
    const current = this.occupancy[y][x];
    const clamped = Math.min(1, current + 0.25);
    this.occupancy[y][x] = clamped;
  }

  #createGrid(size) {
    return Array.from({ length: size }, () => Array.from({ length: size }, () => 0));
  }

  #inBounds(x, y) {
    return x >= 0 && y >= 0 && x < this.gridSize && y < this.gridSize;
  }

  #noise() {
    return (Math.random() - 0.5) * this.processNoise;
  }
}
