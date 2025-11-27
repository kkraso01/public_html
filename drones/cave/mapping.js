function clamp(x, min, max) {
  return Math.min(Math.max(x, min), max);
}

export class OccupancyGrid {
  constructor(resolution = 0.5, size = 80) {
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

  inside(x, y) {
    return x >= 0 && x < this.size && y >= 0 && y < this.size;
  }

  updateCell(x, y, delta) {
    if (!this.inside(x, y)) return;
    const idx = y * this.size + x;
    this.grid[idx] = clamp(this.grid[idx] + delta, -4, 4);
  }

  value(x, y) {
    if (!this.inside(x, y)) return 0;
    return this.grid[y * this.size + x];
  }

  frontierCells() {
    const cells = [];
    for (let y = 1; y < this.size - 1; y++) {
      for (let x = 1; x < this.size - 1; x++) {
        if (this.value(x, y) < -0.1) {
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

export function updateGridFromLidar(grid, pose, hits) {
  hits.forEach((hit) => {
    const worldPoint = hit.point;
    const idxHit = grid.indexFromWorld(worldPoint);
    grid.updateCell(idxHit.x, idxHit.y, 0.8);

    const steps = Math.ceil(hit.distance / grid.resolution);
    for (let i = 0; i < steps; i++) {
      const p = pose.position.clone().add(hit.direction.clone().multiplyScalar(i * grid.resolution));
      const idx = grid.indexFromWorld(p);
      grid.updateCell(idx.x, idx.y, -0.3);
    }
  });
}

export function chooseFrontierTarget(grid, currentPos) {
  const cells = grid.frontierCells();
  if (cells.length === 0) return null;
  let best = null;
  let bestDist = Infinity;
  cells.forEach((c) => {
    const world = new THREE.Vector3(
      grid.origin.x + c.x * grid.resolution,
      currentPos.y,
      grid.origin.y + c.y * grid.resolution,
    );
    const d = world.distanceTo(currentPos);
    if (d < bestDist) {
      bestDist = d;
      best = world;
    }
  });
  return best;
}
