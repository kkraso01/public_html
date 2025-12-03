// nbv.js - Next-Best-View exploration planner
// Actively picks viewpoints that maximize expected information gain

import * as THREE from 'https://cdn.jsdelivr.net/npm/three@0.170.0/build/three.module.js';

/**
 * Estimate information gain from a candidate pose by simulating LiDAR observation
 * @param {OccupancyGrid3D} grid - 3D occupancy grid
 * @param {object} pose - {position: Vector3, orientation: Quaternion}
 * @param {object} options - {maxRange, numRays}
 * @returns {number} - Information gain score (unknown voxels that would be observed)
 */
function estimateInfoGain(grid, pose, options = {}) {
  const maxRange = options.maxRange ?? 12.0;
  const numRays = options.numRays ?? 64;
  const res = grid.resolution;

  let gain = 0;
  const dir = new THREE.Vector3();

  // Simple horizontal fan + a few pitched rays
  for (let i = 0; i < numRays; i++) {
    const angle = (i / numRays) * Math.PI * 2;
    const pitch = (i % 8) < 4 ? 0 : (i % 2 === 0 ? 0.2 : -0.2); // small up/down variation

    // Base direction in world frame: yaw=angle, pitch
    dir.set(Math.cos(angle), Math.sin(angle), 0).normalize();
    if (pitch !== 0) {
      const pitchQ = new THREE.Quaternion().setFromAxisAngle(
        new THREE.Vector3(0, 1, 0), // rotate around Y for pitch
        pitch
      );
      dir.applyQuaternion(pitchQ);
    }

    // Rotate by candidate orientation (for realism)
    dir.applyQuaternion(pose.orientation);

    // March along ray in grid
    const steps = Math.floor(maxRange / res);
    let seenUnknownOnThisRay = false;

    for (let s = 1; s <= steps; s++) {
      const p = pose.position.clone().addScaledVector(dir, s * res);
      const idx = grid.indexFromWorld(p);
      if (!grid.inside(idx.x, idx.y, idx.z)) break;

      const v = grid.value(idx.x, idx.y, idx.z);
      if (v > 0.6) {
        // hit occupied → ray stops
        break;
      }
      if (Math.abs(v) < 0.01) {
        // still unknown → information gain
        gain += 1;
        seenUnknownOnThisRay = true;
        break; // We count only first unknown per ray (cheap heuristic)
      }
    }
  }

  return gain;
}

/**
 * Compute a Next-Best-View target pose that maximizes information gain
 * @param {OccupancyGrid3D} grid - 3D occupancy grid
 * @param {object} slamPose - {position: Vector3, orientation: Quaternion}
 * @param {object} options - {numCandidates, radiusMin, radiusMax, minZ, maxZ, maxRange, travelWeight}
 * @returns {object|null} - {pose, score, gain, travelCost} or null if no good candidates
 */
export function computeNextBestView(grid, slamPose, options = {}) {
  const numCandidates = options.numCandidates ?? 16;
  const radiusMin = options.radiusMin ?? 3.0;
  const radiusMax = options.radiusMax ?? 7.0;
  const minZ = options.minZ ?? 1.2;
  const maxZ = options.maxZ ?? 6.5; // Increased to allow high viewpoints (cave is 8m tall)

  const currentPos = slamPose.position;
  let best = null;

  for (let i = 0; i < numCandidates; i++) {
    const angle = (i / numCandidates) * Math.PI * 2;
    const radius = radiusMin + Math.random() * (radiusMax - radiusMin);

    const candidatePos = new THREE.Vector3(
      currentPos.x + Math.cos(angle) * radius,
      currentPos.y + Math.sin(angle) * radius,
      THREE.MathUtils.clamp(currentPos.z + (Math.random() - 0.5) * 0.8, minZ, maxZ)
    );

    // Face roughly towards current position (good for seeing new stuff)
    const yaw = Math.atan2(currentPos.y - candidatePos.y, currentPos.x - candidatePos.x);
    const q = new THREE.Quaternion().setFromAxisAngle(
      new THREE.Vector3(0, 0, 1),
      yaw
    );

    const pose = { position: candidatePos, orientation: q };

    const gain = estimateInfoGain(grid, pose, { maxRange: options.maxRange ?? 12.0 });

    // Simple score: information gain minus small travel penalty
    const travelCost = candidatePos.distanceTo(currentPos);
    const score = gain - travelCost * (options.travelWeight ?? 0.1);

    if (!best || score > best.score) {
      best = { pose, score, gain, travelCost };
    }
  }

  return best; // or null if none
}
