function clamp(x, min, max) {
  return Math.min(Math.max(x, min), max);
}

// 3D Voxel Grid with Z-up convention
export class OccupancyGrid3D {
  constructor(resolution = 0.5, sizeXY = 80, sizeZ = 20) {
    this.resolution = resolution;
    this.sizeX = sizeXY;
    this.sizeY = sizeXY;
    this.sizeZ = sizeZ;
    // Z-up: origin in 3D space (X, Y, Z)
    this.origin = new THREE.Vector3(
      -((sizeXY * resolution) / 2), 
      -((sizeXY * resolution) / 2),
      0 // Z starts at 0 (ground level)
    );
    this.grid = new Float32Array(sizeXY * sizeXY * sizeZ).fill(0);
  }

  indexFromWorld(pos) {
    const x = Math.floor((pos.x - this.origin.x) / this.resolution);
    const y = Math.floor((pos.y - this.origin.y) / this.resolution);
    const z = Math.floor((pos.z - this.origin.z) / this.resolution);
    return { 
      x, y, z, 
      idx: z * (this.sizeX * this.sizeY) + y * this.sizeX + x 
    };
  }

  inside(x, y, z) {
    return x >= 0 && x < this.sizeX && y >= 0 && y < this.sizeY && z >= 0 && z < this.sizeZ;
  }

  updateCell(x, y, z, delta) {
    if (!this.inside(x, y, z)) return;
    const idx = z * (this.sizeX * this.sizeY) + y * this.sizeX + x;
    this.grid[idx] = clamp(this.grid[idx] + delta, -4, 4);
  }

  value(x, y, z) {
    if (!this.inside(x, y, z)) return 0;
    return this.grid[z * (this.sizeX * this.sizeY) + y * this.sizeX + x];
  }

  // Get 2D slice at given Z height for frontier detection
  getSlice(zWorld) {
    const z = Math.floor((zWorld - this.origin.z) / this.resolution);
    if (z < 0 || z >= this.sizeZ) return null;
    return { z, grid: this };
  }

  frontierCells(zSlice = null) {
    const cells = [];
    const zLevel = zSlice !== null ? zSlice : Math.floor(this.sizeZ / 4); // Default to lower level
    
    for (let y = 1; y < this.sizeY - 1; y++) {
      for (let x = 1; x < this.sizeX - 1; x++) {
        if (this.value(x, y, zLevel) < -0.05) {
          // Check horizontal neighbors for unknown space
          const unknownNbr =
            Math.abs(this.value(x + 1, y, zLevel)) < 0.01 ||
            Math.abs(this.value(x - 1, y, zLevel)) < 0.01 ||
            Math.abs(this.value(x, y + 1, zLevel)) < 0.01 ||
            Math.abs(this.value(x, y - 1, zLevel)) < 0.01;
          if (unknownNbr) cells.push({ x, y, z: zLevel });
        }
      }
    }
    return cells;
  }

  // Get occupied voxels for visualization
  getOccupiedVoxels(threshold = 0.6) {
    const voxels = [];
    for (let z = 0; z < this.sizeZ; z++) {
      for (let y = 0; y < this.sizeY; y++) {
        for (let x = 0; x < this.sizeX; x++) {
          if (this.value(x, y, z) > threshold) {
            voxels.push({
              x, y, z,
              world: this.worldFromCell({ x, y, z })
            });
          }
        }
      }
    }
    return voxels;
  }

  worldFromCell(cell) {
    return new THREE.Vector3(
      this.origin.x + cell.x * this.resolution,
      this.origin.y + cell.y * this.resolution,
      this.origin.z + cell.z * this.resolution
    );
  }

  getStats() {
    let free = 0, occupied = 0, unknown = 0;
    for (let i = 0; i < this.grid.length; i++) {
      if (this.grid[i] < -0.05) free++;
      else if (this.grid[i] > 0.6) occupied++;
      else unknown++;
    }
    return { free, occupied, unknown, total: this.grid.length };
  }
}

// Legacy 2D grid for backward compatibility
export class OccupancyGrid {
  constructor(resolution = 0.5, size = 80) {
    this.resolution = resolution;
    this.size = size;
    this.origin = new THREE.Vector2(-((size * resolution) / 2), -((size * resolution) / 2));
    this.grid = new Float32Array(size * size).fill(0);
  }

  indexFromWorld(pos) {
    const x = Math.floor((pos.x - this.origin.x) / this.resolution);
    const y = Math.floor((pos.y - this.origin.y) / this.resolution);
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

// Particle Filter for 6DOF pose estimation (X, Y, Z, roll, pitch, yaw)
export class ParticleFilter {
  constructor(numParticles = 100) {
    this.numParticles = numParticles;
    this.particles = [];
    this.weights = new Float32Array(numParticles);
    this.estimatedPose = {
      position: new THREE.Vector3(0, 0, 1.4),
      orientation: new THREE.Quaternion(),
      yaw: 0
    };
  }

  initialize(initialPose, positionNoise = 0.5, yawNoise = 0.3) {
    this.particles = [];
    for (let i = 0; i < this.numParticles; i++) {
      this.particles.push({
        position: new THREE.Vector3(
          initialPose.position.x + (Math.random() - 0.5) * positionNoise,
          initialPose.position.y + (Math.random() - 0.5) * positionNoise,
          initialPose.position.z + (Math.random() - 0.5) * positionNoise * 0.5
        ),
        yaw: initialPose.yaw + (Math.random() - 0.5) * yawNoise,
        orientation: initialPose.orientation.clone()
      });
      this.weights[i] = 1.0 / this.numParticles;
    }
  }

  // Motion model: predict particle poses based on velocity
  predict(velocity, angularVel, dt, processNoise = 0.05) {
    this.particles.forEach(p => {
      // Add motion with noise (Z-up convention)
      p.position.x += velocity.x * dt + (Math.random() - 0.5) * processNoise;
      p.position.y += velocity.y * dt + (Math.random() - 0.5) * processNoise;
      p.position.z += velocity.z * dt + (Math.random() - 0.5) * processNoise * 0.5;
      p.yaw += angularVel.z * dt + (Math.random() - 0.5) * processNoise * 0.5;
      
      // Update quaternion from yaw
      p.orientation.setFromAxisAngle(new THREE.Vector3(0, 0, 1), p.yaw);
    });
  }

  // Measurement model: weight particles based on LIDAR likelihood
  update(lidarHits, caveSim) {
    if (!lidarHits || lidarHits.length === 0) return;
    
    let sumWeights = 0;
    this.particles.forEach((p, i) => {
      let logLikelihood = 0;
      let validMeasurements = 0;

      // Sample a subset of LIDAR rays for efficiency
      const sampleStep = Math.max(1, Math.floor(lidarHits.length / 20));
      for (let j = 0; j < lidarHits.length; j += sampleStep) {
        const hit = lidarHits[j];
        
        // Transform ray direction to particle frame
        const rotatedDir = hit.direction.clone().applyQuaternion(p.orientation);
        
        // Simulate expected measurement from this particle pose
        const expectedHit = caveSim.raycast(p.position, rotatedDir, 18);
        
        if (expectedHit && hit.distance < 17) {
          // Likelihood based on distance match (Gaussian)
          const error = Math.abs(expectedHit.distance - hit.distance);
          const sigma = 0.3; // measurement noise
          logLikelihood += -0.5 * (error * error) / (sigma * sigma);
          validMeasurements++;
        }
      }

      // Weight is exp(log-likelihood), with min weight to avoid degeneracy
      this.weights[i] = validMeasurements > 0 ? Math.exp(logLikelihood) : 0.001;
      sumWeights += this.weights[i];
    });

    // Normalize weights
    if (sumWeights > 0) {
      for (let i = 0; i < this.numParticles; i++) {
        this.weights[i] /= sumWeights;
      }
    }

    // Update estimated pose (weighted average)
    this.estimatedPose.position.set(0, 0, 0);
    let sinYaw = 0, cosYaw = 0;
    
    this.particles.forEach((p, i) => {
      const w = this.weights[i];
      this.estimatedPose.position.addScaledVector(p.position, w);
      sinYaw += Math.sin(p.yaw) * w;
      cosYaw += Math.cos(p.yaw) * w;
    });
    
    this.estimatedPose.yaw = Math.atan2(sinYaw, cosYaw);
    this.estimatedPose.orientation.setFromAxisAngle(new THREE.Vector3(0, 0, 1), this.estimatedPose.yaw);
  }

  // Resample particles (low variance resampling)
  resample() {
    const newParticles = [];
    const step = 1.0 / this.numParticles;
    let u = Math.random() * step;
    let c = this.weights[0];
    let i = 0;

    for (let j = 0; j < this.numParticles; j++) {
      while (u > c && i < this.numParticles - 1) {
        i++;
        c += this.weights[i];
      }
      newParticles.push({
        position: this.particles[i].position.clone(),
        yaw: this.particles[i].yaw,
        orientation: this.particles[i].orientation.clone()
      });
      u += step;
    }

    this.particles = newParticles;
    this.weights.fill(1.0 / this.numParticles);
  }

  // Check if resampling is needed (effective sample size)
  needsResampling() {
    let sumSq = 0;
    for (let i = 0; i < this.numParticles; i++) {
      sumSq += this.weights[i] * this.weights[i];
    }
    const nEff = 1.0 / sumSq;
    return nEff < this.numParticles / 2;
  }
}

// 3D LIDAR mapping update with raytracing
export function updateGrid3DFromLidar(grid, pose, hits) {
  let occupiedCount = 0;
  let freeCount = 0;
  
  hits.forEach((hit) => {
    // Only process actual hits (not max-range misses)
    if (!hit.hit) return;
    
    const worldPoint = hit.point;
    const idxHit = grid.indexFromWorld(worldPoint);
    
    // Mark hit point as occupied (stronger update for obstacles)
    if (grid.inside(idxHit.x, idxHit.y, idxHit.z)) {
      grid.updateCell(idxHit.x, idxHit.y, idxHit.z, 1.2); // Stronger occupied signal
      occupiedCount++;
    }

    // Raytrace from sensor position to hit point, marking free space
    const steps = Math.ceil(hit.distance / grid.resolution);
    for (let i = 1; i < steps; i++) { // Start from 1 to skip sensor position
      const p = pose.position.clone().add(hit.direction.clone().multiplyScalar(i * grid.resolution));
      const idx = grid.indexFromWorld(p);
      if (grid.inside(idx.x, idx.y, idx.z)) {
        grid.updateCell(idx.x, idx.y, idx.z, -0.4); // Stronger free signal
        freeCount++;
      }
    }
  });
  
  // Debug every 200 updates
  if (!updateGrid3DFromLidar._counter) updateGrid3DFromLidar._counter = 0;
  updateGrid3DFromLidar._counter++;
  if (updateGrid3DFromLidar._counter % 200 === 0) {
    const stats = grid.getStats();
    console.log(`[GRID] Update #${updateGrid3DFromLidar._counter}: ${occupiedCount} occupied, ${freeCount} free cells marked. Grid stats: free=${stats.free}, occupied=${stats.occupied}, unknown=${stats.unknown}`);
  }
}

// Legacy 2D update
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
  if (grid.worldFromCell) {
    // 3D grid has built-in method
    return grid.worldFromCell(cell);
  }
  // 2D grid fallback
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
  const is3D = cells.length > 0 && cells[0].z !== undefined;
  const key = (c) => is3D ? `${c.x},${c.y},${c.z}` : `${c.x},${c.y}`;

  const neighbors = (c) => {
    if (is3D) {
      return [
        { x: c.x + 1, y: c.y, z: c.z },
        { x: c.x - 1, y: c.y, z: c.z },
        { x: c.x, y: c.y + 1, z: c.z },
        { x: c.x, y: c.y - 1, z: c.z },
      ];
    }
    return [
      { x: c.x + 1, y: c.y },
      { x: c.x - 1, y: c.y },
      { x: c.x, y: c.y + 1 },
      { x: c.x, y: c.y - 1 },
    ];
  };

  const checkFree = (n) => {
    if (is3D) {
      return grid.inside(n.x, n.y, n.z) && grid.value(n.x, n.y, n.z) < -0.05;
    }
    return grid.inside(n.x, n.y) && grid.value(n.x, n.y) < -0.05;
  };

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
        if (!visited.has(nk) && checkFree(n)) {
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
  
  // If no frontiers found, search for nearest unknown space directly
  if (!clusters.length) {
    const unknownTarget = findNearestUnknown(grid, currentPos);
    if (unknownTarget) {
      // Ensure safe altitude
      unknownTarget.z = Math.max(unknownTarget.z, currentPos.z, 1.2);
      return { point: unknownTarget, clusterCount: 0, targetSize: 1 };
    }
    return null;
  }

  const current2d = new THREE.Vector2(currentPos.x, currentPos.y);
  
  // Score clusters by combination of size and distance (prefer large + far)
  const scoredClusters = clusters.map(c => {
    const dist = new THREE.Vector2(c.centroid.x, c.centroid.y).distanceTo(current2d);
    // Score = size * distance_factor (prefer larger clusters further away)
    // But avoid extremely close clusters (< 1m) to prevent hovering
    const distFactor = dist < 1.0 ? 0.1 : Math.min(dist / 3.0, 2.0);
    const score = c.cells.length * distFactor;
    return { cluster: c, dist, score };
  });
  
  // Pick highest scoring cluster
  const best = scoredClusters.reduce((best, c) => 
    (!best || c.score > best.score) ? c : best
  , null);
  
  const targetCluster = best.cluster;
  // Pick cell in cluster furthest from current position for more exploration
  const targetCell = targetCluster.cells.reduce((best, cell) => {
    const cellWorld = worldFromCell(grid, cell);
    const dist = new THREE.Vector2(cellWorld.x, cellWorld.y).distanceTo(current2d);
    return (!best || dist > best.dist) ? { cell, dist } : best;
  }, null).cell;
  
  const worldTarget = worldFromCell(grid, targetCell);
  // Allow full vertical exploration - frontier cells already have Z from 3D grid
  worldTarget.z = Math.max(worldTarget.z, 1.2); // Only enforce minimum safety altitude
  return { point: worldTarget, clusterCount: clusters.length, targetSize: targetCluster.cells.length };
}

// Find nearest unknown cell when no frontiers exist
function findNearestUnknown(grid, currentPos) {
  const currentCell = grid.indexFromWorld(currentPos);
  const zLevel = currentCell.z;
  const is3D = grid.sizeZ !== undefined;
  
  let bestDist = Infinity;
  let bestCell = null;
  
  // Search in expanding rings from current position
  const maxSearchRadius = 20; // cells
  
  for (let radius = 5; radius < maxSearchRadius; radius++) {
    for (let dy = -radius; dy <= radius; dy++) {
      for (let dx = -radius; dx <= radius; dx++) {
        const x = currentCell.x + dx;
        const y = currentCell.y + dy;
        
        // Check if on current ring boundary
        if (Math.abs(dx) !== radius && Math.abs(dy) !== radius) continue;
        
        if (is3D) {
          if (!grid.inside(x, y, zLevel)) continue;
          const value = grid.value(x, y, zLevel);
          
          // Unknown cell (not observed)
          if (Math.abs(value) < 0.01) {
            const cellWorld = grid.worldFromCell({ x, y, z: zLevel });
            const dist = new THREE.Vector2(cellWorld.x, cellWorld.y).distanceTo(
              new THREE.Vector2(currentPos.x, currentPos.y)
            );
            
            if (dist < bestDist) {
              bestDist = dist;
              bestCell = { x, y, z: zLevel };
            }
          }
        } else {
          if (!grid.inside(x, y)) continue;
          const value = grid.value(x, y);
          
          if (Math.abs(value) < 0.01) {
            const cellWorld = worldFromCell(grid, { x, y });
            const dist = new THREE.Vector2(cellWorld.x, cellWorld.y).distanceTo(
              new THREE.Vector2(currentPos.x, currentPos.y)
            );
            
            if (dist < bestDist) {
              bestDist = dist;
              bestCell = { x, y };
            }
          }
        }
      }
    }
    
    // Found unknown space at this radius, return it
    if (bestCell) {
      const worldTarget = worldFromCell(grid, bestCell);
      worldTarget.z = Math.max(currentPos.z, 1.2); // Preserve altitude with minimum safety
      return worldTarget;
    }
  }
  
  return null;
}

export function planPath(grid, start, goal, options = {}) {
  // Try with safety inflation first, fall back to no inflation if it fails
  const useInflation = options.useInflation !== false;
  
  const path = _planPathInternal(grid, start, goal, useInflation);
  
  // If path planning failed and we were using inflation, retry without it
  if (path.length === 0 && useInflation) {
    return _planPathInternal(grid, start, goal, false);
  }
  
  return path;
}

function _planPathInternal(grid, start, goal, useInflation = true) {
  const startCell = grid.indexFromWorld(start);
  const goalCell = grid.indexFromWorld(goal);
  
  // Determine Z level for path planning (use start altitude)
  const zLevel = startCell.z;
  
  // Check if 2D grid (legacy) or 3D grid
  const is3D = grid.sizeZ !== undefined;
  
  if (is3D) {
    if (!grid.inside(startCell.x, startCell.y, zLevel) || !grid.inside(goalCell.x, goalCell.y, zLevel)) return [];
  } else {
    if (!grid.inside(startCell.x, startCell.y) || !grid.inside(goalCell.x, goalCell.y)) return [];
  }

  const open = new Set([`${startCell.x},${startCell.y}`]);
  const cameFrom = new Map();
  const gScore = new Map([[`${startCell.x},${startCell.y}`, 0]]);
  const fScore = new Map([[`${startCell.x},${startCell.y}`, 0]]);
  const neighborDirs = [
    // Cardinal directions (cost 1.0)
    { x: 1, y: 0, cost: 1.0 },
    { x: -1, y: 0, cost: 1.0 },
    { x: 0, y: 1, cost: 1.0 },
    { x: 0, y: -1, cost: 1.0 },
    // Diagonal directions (cost sqrt(2) ≈ 1.414)
    { x: 1, y: 1, cost: 1.414 },
    { x: 1, y: -1, cost: 1.414 },
    { x: -1, y: 1, cost: 1.414 },
    { x: -1, y: -1, cost: 1.414 },
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

  const occupied = (c) => {
    if (is3D) {
      // Check if cell is occupied
      const centerOcc = grid.value(c.x, c.y, zLevel);
      if (centerOcc > 0.6) return true;
      
      // Optional safety inflation
      if (useInflation) {
        // Light safety inflation: only check immediate cardinal neighbors (not diagonals)
        // This gives ~0.25m clearance on each side with 0.5m resolution
        const cardinalNeighbors = [
          {dx: 1, dy: 0}, {dx: -1, dy: 0}, {dx: 0, dy: 1}, {dx: 0, dy: -1}
        ];
        
        for (const {dx, dy} of cardinalNeighbors) {
          if (grid.inside(c.x + dx, c.y + dy, zLevel)) {
            const neighborOcc = grid.value(c.x + dx, c.y + dy, zLevel);
            if (neighborOcc > 0.7) return true; // Higher threshold for inflation
          }
        }
      }
      return false;
    }
    
    // 2D grid: same logic
    const centerOcc = grid.value(c.x, c.y);
    if (centerOcc > 0.6) return true;
    
    // Optional safety inflation
    if (useInflation) {
      const cardinalNeighbors = [
        {dx: 1, dy: 0}, {dx: -1, dy: 0}, {dx: 0, dy: 1}, {dx: 0, dy: -1}
      ];
      
      for (const {dx, dy} of cardinalNeighbors) {
        if (grid.inside(c.x + dx, c.y + dy)) {
          const neighborOcc = grid.value(c.x + dx, c.y + dy);
          if (neighborOcc > 0.7) return true;
        }
      }
    }
    return false;
  };

  const reconstruct = (endCell) => {
    const path = [endCell];
    let curKey = `${endCell.x},${endCell.y}`;
    while (cameFrom.has(curKey)) {
      const prev = cameFrom.get(curKey);
      path.unshift(prev);
      curKey = `${prev.x},${prev.y}`;
    }
    const waypoints = path.map((c, idx) => {
      const cellWithZ = is3D ? { x: c.x, y: c.y, z: zLevel } : c;
      const wp = worldFromCell(grid, cellWithZ);
      // Gradually interpolate altitude from start to goal
      const progress = idx / (path.length - 1 || 1);
      wp.z = start.z + (goal.z - start.z) * progress;
      return wp;
    });
    
    // Simplify path: remove redundant waypoints on straight lines
    const simplified = [];
    if (waypoints.length > 0) {
      simplified.push(waypoints[0]); // Always keep start
      
      for (let i = 1; i < waypoints.length - 1; i++) {
        const prev = waypoints[i - 1];
        const curr = waypoints[i];
        const next = waypoints[i + 1];
        
        // Calculate vectors
        const v1x = curr.x - prev.x;
        const v1y = curr.y - prev.y;
        const v2x = next.x - curr.x;
        const v2y = next.y - curr.y;
        
        // Normalize
        const len1 = Math.sqrt(v1x * v1x + v1y * v1y);
        const len2 = Math.sqrt(v2x * v2x + v2y * v2y);
        
        if (len1 > 0.001 && len2 > 0.001) {
          const dot = (v1x * v2x + v1y * v2y) / (len1 * len2);
          
          // Keep waypoint if there's a direction change (angle > 15 degrees)
          // cos(15°) ≈ 0.966
          if (dot < 0.966) {
            simplified.push(curr);
          }
          // Otherwise skip this waypoint (it's on a straight line)
        } else {
          simplified.push(curr); // Keep if vectors are too small
        }
      }
      
      if (waypoints.length > 1) {
        simplified.push(waypoints[waypoints.length - 1]); // Always keep goal
      }
    }
    
    // Add yaw to each simplified waypoint
    for (let i = 0; i < simplified.length; i++) {
      if (i < simplified.length - 1) {
        // Point towards next waypoint
        const dx = simplified[i + 1].x - simplified[i].x;
        const dy = simplified[i + 1].y - simplified[i].y;
        simplified[i].yaw = Math.atan2(dy, dx);
      } else {
        // Last waypoint: point towards goal
        const dx = goal.x - simplified[i].x;
        const dy = goal.y - simplified[i].y;
        simplified[i].yaw = Math.atan2(dy, dx);
      }
    }
    
    return simplified;
  };

  while (open.size) {
    const current = popLowest();
    if (!current) break;
    if (current.x === goalCell.x && current.y === goalCell.y) {
      return reconstruct(current);
    }

    neighborDirs.forEach((d) => {
      const n = { x: current.x + d.x, y: current.y + d.y };
      const insideCheck = is3D ? grid.inside(n.x, n.y, zLevel) : grid.inside(n.x, n.y);
      if (!insideCheck || occupied(n)) return;
      
      const currentOccupancy = is3D ? grid.value(n.x, n.y, zLevel) : grid.value(n.x, n.y);
      // Use actual movement cost (diagonal = 1.414, cardinal = 1.0)
      const moveCost = d.cost ?? 1.0;
      const tentativeG = (gScore.get(`${current.x},${current.y}`) ?? Infinity) + moveCost + Math.max(currentOccupancy, 0);
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
