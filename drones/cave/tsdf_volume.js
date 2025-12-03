// tsdf_volume.js - Truncated Signed Distance Field (TSDF) volume for 3D mapping
// More accurate surface representation than binary occupancy grids

import * as THREE from 'https://cdn.jsdelivr.net/npm/three@0.170.0/build/three.module.js';

/**
 * TSDF Volume - stores signed distance to nearest surface and confidence weights
 * Uses same coordinate convention as OccupancyGrid3D (+Z up)
 */
export class TSDFVolume {
  constructor(resolution = 0.3, sizeXY = 80, sizeZ = 20, truncation = 0.6) {
    this.resolution = resolution;
    this.sizeX = sizeXY;
    this.sizeY = sizeXY;
    this.sizeZ = sizeZ;
    // Z-up convention: origin in 3D space (X, Y, Z)
    this.origin = new THREE.Vector3(
      -((sizeXY * resolution) / 2),
      -((sizeXY * resolution) / 2),
      0 // Z starts at 0 (ground level)
    );
    this.truncation = truncation; // mu - truncation distance

    const voxels = sizeXY * sizeXY * sizeZ;
    this.tsdf = new Float32Array(voxels).fill(1);     // 1 = far in front of surface
    this.weights = new Float32Array(voxels).fill(0);  // 0 weight initially
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
    return x >= 0 && x < this.sizeX &&
           y >= 0 && y < this.sizeY &&
           z >= 0 && z < this.sizeZ;
  }

  worldFromCell({ x, y, z }) {
    return new THREE.Vector3(
      this.origin.x + (x + 0.5) * this.resolution,
      this.origin.y + (y + 0.5) * this.resolution,
      this.origin.z + (z + 0.5) * this.resolution,
    );
  }

  /**
   * Integrate a LiDAR scan into the TSDF volume
   * @param {object} pose - {position: Vector3, orientation: Quaternion}
   * @param {Array} hits - LiDAR hit array from lidar.scan()
   */
  integrateScan(pose, hits) {
    const mu = this.truncation;

    hits.forEach(hit => {
      if (!hit.hit) return; // Skip rays that didn't hit anything

      const sensorPos = pose.position;
      const dir = hit.direction.clone().normalize();
      const depth = hit.distance;

      // Integrate voxels in [depth - mu, depth + mu]
      const start = Math.max(0, depth - mu);
      const end   = depth + mu;
      const step = this.resolution * 0.7; // slightly less than voxel size for coverage

      for (let d = start; d <= end; d += step) {
        const p = sensorPos.clone().addScaledVector(dir, d);
        const { x, y, z, idx } = this.indexFromWorld(p);
        if (!this.inside(x, y, z)) continue;

        // Signed distance: positive in front of surface, negative behind
        const sdf = depth - d; // convention: >0 in front, <0 behind
        if (sdf <= -mu) continue;  // too far behind surface
        if (sdf >= mu) continue;   // too far in front (already empty)

        // Normalize to [-1, 1] range
        const tsdfMeas = THREE.MathUtils.clamp(sdf / mu, -1, 1);
        const wOld = this.weights[idx];
        const wNew = wOld + 1;

        // Weighted average fusion
        this.tsdf[idx] = (wOld * this.tsdf[idx] + tsdfMeas) / wNew;
        this.weights[idx] = wNew;
      }
    });
  }

  /**
   * Quick surface extraction for visualization as point cloud
   * Returns points near zero-crossing (surface)
   * @param {number} threshold - TSDF value threshold for surface detection
   * @returns {Array<Vector3>} - Surface points in world coordinates
   */
  getSurfacePoints(threshold = 0.1) {
    const pts = [];
    for (let z = 1; z < this.sizeZ - 1; z++) {
      for (let y = 1; y < this.sizeY - 1; y++) {
        for (let x = 1; x < this.sizeX - 1; x++) {
          const idx = z * (this.sizeX * this.sizeY) + y * this.sizeX + x;
          const w = this.weights[idx];
          if (w < 1) continue; // Skip unobserved voxels
          
          const v = this.tsdf[idx];
          // Surface is where TSDF is near zero
          if (Math.abs(v) < threshold) {
            pts.push(this.worldFromCell({ x, y, z }));
          }
        }
      }
    }
    return pts;
  }

  /**
   * Get statistics about the TSDF volume
   * @returns {object} - {surface, front, behind, unknown, total}
   */
  getStats() {
    let surface = 0;  // Near zero-crossing
    let front = 0;    // Positive TSDF (in front of surface)
    let behind = 0;   // Negative TSDF (behind surface)
    let unknown = 0;  // Not yet observed
    
    for (let i = 0; i < this.tsdf.length; i++) {
      if (this.weights[i] < 0.1) {
        unknown++;
      } else {
        const v = this.tsdf[i];
        if (Math.abs(v) < 0.1) {
          surface++;
        } else if (v > 0) {
          front++;
        } else {
          behind++;
        }
      }
    }
    
    return { surface, front, behind, unknown, total: this.tsdf.length };
  }

  /**
   * Get gradient of TSDF at a point (for normal estimation or collision avoidance)
   * @param {Vector3} pos - World position
   * @returns {Vector3|null} - Gradient vector or null if outside volume
   */
  getGradient(pos) {
    const { x, y, z } = this.indexFromWorld(pos);
    if (!this.inside(x, y, z)) return null;
    
    // Check neighbors exist
    if (x <= 0 || x >= this.sizeX - 1 || 
        y <= 0 || y >= this.sizeY - 1 || 
        z <= 0 || z >= this.sizeZ - 1) return null;
    
    const getVal = (ix, iy, iz) => {
      const idx = iz * (this.sizeX * this.sizeY) + iy * this.sizeX + ix;
      return this.tsdf[idx];
    };
    
    // Central differences
    const dx = (getVal(x+1, y, z) - getVal(x-1, y, z)) / (2 * this.resolution);
    const dy = (getVal(x, y+1, z) - getVal(x, y-1, z)) / (2 * this.resolution);
    const dz = (getVal(x, y, z+1) - getVal(x, y, z-1)) / (2 * this.resolution);
    
    return new THREE.Vector3(dx, dy, dz);
  }
}
