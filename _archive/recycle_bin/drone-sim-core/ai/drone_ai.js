import { AmmoLidar } from '../sensors/lidar.js';
import { clamp } from '../utils/drone_math.js';

class OccupancyGrid {
  constructor(resolution = 0.35, size = 96) {
    this.resolution = resolution;
    this.size = size;
    this.origin = new THREE.Vector2(-((size * resolution) / 2), -((size * resolution) / 2));
    this.grid = new Float32Array(size * size).fill(0);
  }

  indexFromWorld(pos) {
    const x = Math.floor((pos.x - this.origin.x) / this.resolution);
    const y = Math.floor((pos.z - this.origin.y) / this.resolution);
    return { x, y, idx: y * this.size + x };
  }

  isInside(x, y) {
    return x >= 0 && x < this.size && y >= 0 && y < this.size;
  }

  updateCell(x, y, delta) {
    if (!this.isInside(x, y)) return;
    const idx = y * this.size + x;
    this.grid[idx] = clamp(this.grid[idx] + delta, -5, 5);
  }

  value(x, y) {
    if (!this.isInside(x, y)) return 0;
    return this.grid[y * this.size + x];
  }

  frontierCells() {
    const cells = [];
    for (let y = 1; y < this.size - 1; y++) {
      for (let x = 1; x < this.size - 1; x++) {
        if (this.value(x, y) < -0.2) {
          const unknownNbr =
            Math.abs(this.value(x + 1, y)) < 0.01 ||
            Math.abs(this.value(x - 1, y)) < 0.01 ||
            Math.abs(this.value(x, y + 1)) < 0.01 ||
            Math.abs(this.value(x, y - 1)) < 0.01;
          if (unknownNbr) cells.push({ x, y });
        }
      }
    }
    return cells;
  }
}

export class DroneAI {
  constructor(ammoWorld) {
    this.grid = new OccupancyGrid();
    this.lidar = new AmmoLidar(ammoWorld, { rays: 60, maxRange: 22, verticalSpread: 0.3 });
    this.frontiersDirty = true;
    this.onReplan = () => {};
  }

  setOnReplan(cb) {
    this.onReplan = cb || (() => {});
  }

  onEnvironmentChanged() {
    this.frontiersDirty = true;
    this.onReplan();
  }

  scanAndUpdate(pose) {
    const hits = this.lidar.scan(pose);
    for (const h of hits) {
      const cell = this.grid.indexFromWorld(h.point);
      if (this.grid.isInside(cell.x, cell.y)) this.grid.updateCell(cell.x, cell.y, 1.4);

      const steps = Math.ceil(h.distance / this.grid.resolution);
      const dir2d = new THREE.Vector2(h.direction.x, h.direction.z).normalize();
      for (let i = 1; i < steps; i++) {
        const pt = new THREE.Vector2(pose.position.x, pose.position.z).add(dir2d.clone().multiplyScalar(i * this.grid.resolution));
        const idx = this.grid.indexFromWorld(new THREE.Vector3(pt.x, 0, pt.y));
        this.grid.updateCell(idx.x, idx.y, -0.6);
      }
    }
    return hits;
  }

  pickFrontier(position) {
    const frontiers = this.grid.frontierCells();
    if (!frontiers.length) return null;
    frontiers.sort((a, b) => {
      const aw = new THREE.Vector2(a.x * this.grid.resolution + this.grid.origin.x, a.y * this.grid.resolution + this.grid.origin.y);
      const bw = new THREE.Vector2(b.x * this.grid.resolution + this.grid.origin.x, b.y * this.grid.resolution + this.grid.origin.y);
      return aw.distanceToSquared(new THREE.Vector2(position.x, position.z)) -
        bw.distanceToSquared(new THREE.Vector2(position.x, position.z));
    });
    return frontiers[0];
  }
}
