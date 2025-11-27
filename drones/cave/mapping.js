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
        if (this.value(x, y) < -0.05) {
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

function worldFromCell(grid, cell) {
  return new THREE.Vector3(
    grid.origin.x + cell.x * grid.resolution,
    0,
    grid.origin.y + cell.y * grid.resolution,
  );
}

export function clusterFrontiers(grid) {
  const cells = grid.frontierCells();
  const visited = new Set();
  const clusters = [];
  const key = (c) => `${c.x},${c.y}`;

  const neighbors = (c) => [
    { x: c.x + 1, y: c.y },
    { x: c.x - 1, y: c.y },
    { x: c.x, y: c.y + 1 },
    { x: c.x, y: c.y - 1 },
  ];

  for (const cell of cells) {
    const k = key(cell);
    if (visited.has(k)) continue;
    const queue = [cell];
    const cluster = [];
    visited.add(k);
    while (queue.length) {
      const cur = queue.pop();
      cluster.push(cur);
      neighbors(cur).forEach((n) => {
        const nk = key(n);
        if (!visited.has(nk) && grid.inside(n.x, n.y) && grid.value(n.x, n.y) < -0.05) {
          visited.add(nk);
          queue.push(n);
        }
      });
    }
    const centroid = cluster.reduce((acc, c) => acc.add(worldFromCell(grid, c)), new THREE.Vector3()).multiplyScalar(1 / cluster.length);
    clusters.push({ cells: cluster, centroid });
  }

  return clusters;
}

export function chooseFrontierTarget(grid, currentPos) {
  const clusters = clusterFrontiers(grid);
  if (!clusters.length) return null;

  const current2d = new THREE.Vector2(currentPos.x, currentPos.z);
  const nearest = clusters.reduce((best, c) => {
    const dist = new THREE.Vector2(c.centroid.x, c.centroid.z).distanceTo(current2d);
    if (!best || dist < best.dist) return { cluster: c, dist };
    return best;
  }, null);
  const largest = clusters.reduce((best, c) => (!best || c.cells.length > best.cells.length ? { cluster: c } : best), null);

  const targetCluster = nearest?.dist < 4 ? nearest.cluster : largest.cluster;
  const targetCell = targetCluster.cells[0];
  const worldTarget = worldFromCell(grid, targetCell);
  worldTarget.y = currentPos.y;
  return { point: worldTarget, clusterCount: clusters.length, targetSize: targetCluster.cells.length };
}

export function planPath(grid, start, goal) {
  const startCell = grid.indexFromWorld(start);
  const goalCell = grid.indexFromWorld(goal);
  if (!grid.inside(startCell.x, startCell.y) || !grid.inside(goalCell.x, goalCell.y)) return [];

  const open = new Set([`${startCell.x},${startCell.y}`]);
  const cameFrom = new Map();
  const gScore = new Map([[`${startCell.x},${startCell.y}`, 0]]);
  const fScore = new Map([[`${startCell.x},${startCell.y}`, 0]]);
  const neighborDirs = [
    { x: 1, y: 0 },
    { x: -1, y: 0 },
    { x: 0, y: 1 },
    { x: 0, y: -1 },
  ];

  const h = (c) => Math.abs(c.x - goalCell.x) + Math.abs(c.y - goalCell.y);

  const score = (c) => fScore.get(`${c.x},${c.y}`) ?? Infinity;
  const popLowest = () => {
    let best = null;
    for (const key of open) {
      const [x, y] = key.split(',').map((n) => parseInt(n, 10));
      const s = score({ x, y });
      if (!best || s < best.score) best = { key, score: s };
    }
    if (!best) return null;
    open.delete(best.key);
    const [bx, by] = best.key.split(',').map((n) => parseInt(n, 10));
    return { x: bx, y: by };
  };

  const occupied = (c) => grid.value(c.x, c.y) > 0.6;

  const reconstruct = (endCell) => {
    const path = [endCell];
    let curKey = `${endCell.x},${endCell.y}`;
    while (cameFrom.has(curKey)) {
      const prev = cameFrom.get(curKey);
      path.unshift(prev);
      curKey = `${prev.x},${prev.y}`;
    }
    return path.map((c) => {
      const wp = worldFromCell(grid, c);
      wp.y = start.y;
      return wp;
    });
  };

  while (open.size) {
    const current = popLowest();
    if (!current) break;
    if (current.x === goalCell.x && current.y === goalCell.y) {
      return reconstruct(current);
    }

    neighborDirs.forEach((d) => {
      const n = { x: current.x + d.x, y: current.y + d.y };
      if (!grid.inside(n.x, n.y) || occupied(n)) return;
      const tentativeG = (gScore.get(`${current.x},${current.y}`) ?? Infinity) + 1 + Math.max(grid.value(n.x, n.y), 0);
      const nk = `${n.x},${n.y}`;
      if (tentativeG < (gScore.get(nk) ?? Infinity)) {
        cameFrom.set(nk, current);
        gScore.set(nk, tentativeG);
        fScore.set(nk, tentativeG + h(n));
        open.add(nk);
      }
    });
  }

  return [];
}
