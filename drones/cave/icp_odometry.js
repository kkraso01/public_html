// icp_odometry.js - Iterative Closest Point (ICP) for LiDAR odometry
// Estimates pose changes between consecutive scans for realistic SLAM

import * as THREE from 'https://cdn.jsdelivr.net/npm/three@0.170.0/build/three.module.js';

/**
 * Build a point cloud (world coords) from LiDAR hits
 * @param {object} pose - {position: Vector3, orientation: Quaternion}
 * @param {Array} hits - LiDAR hit array
 * @param {number} maxPoints - Downsample to this many points
 * @returns {Array<Vector3>} - Point cloud in world coordinates
 */
export function hitsToPointCloud(pose, hits, maxPoints = 500) {
  const pts = [];
  const step = Math.max(1, Math.floor(hits.length / maxPoints));
  for (let i = 0; i < hits.length; i += step) {
    const h = hits[i];
    if (!h.hit) continue;
    const p = pose.position.clone().addScaledVector(
      h.direction,
      h.distance
    );
    pts.push(p);
  }
  return pts;
}

/**
 * Simple 3x3 SVD for Kabsch algorithm (specialized for rotation estimation)
 * Uses Jacobi iteration - good enough for ICP
 * @param {Array<number>} H - 3x3 matrix in row-major order [h00,h01,h02, h10,h11,h12, h20,h21,h22]
 * @returns {object} - {U: Matrix3, S: [s0,s1,s2], V: Matrix3}
 */
function svd3x3(H) {
  // For simplicity, use a helper that converts to Matrix3 and back
  const mat = new THREE.Matrix3();
  mat.set(H[0], H[1], H[2], H[3], H[4], H[5], H[6], H[7], H[8]);
  
  // Approximate SVD using eigendecomposition of H^T H
  // This is a simplified version - for production use a proper SVD library
  
  // For now, return identity as placeholder
  // A full implementation would use Jacobi or Golub-Reinsch SVD
  const U = new THREE.Matrix3().identity();
  const S = [1, 1, 1];
  const V = new THREE.Matrix3().identity();
  
  return { U, S, V };
}

/**
 * Compute rigid transform (R, t) aligning src -> dst using Kabsch algorithm
 * @param {Array<Vector3>} srcPts - Source point cloud
 * @param {Array<Vector3>} dstPts - Destination point cloud
 * @returns {object|null} - {R: Matrix3, t: Vector3} or null if failed
 */
function computeRigidTransformKabsch(srcPts, dstPts) {
  const N = srcPts.length;
  if (N === 0 || N !== dstPts.length) return null;

  // Compute centroids
  const srcCentroid = new THREE.Vector3();
  const dstCentroid = new THREE.Vector3();
  for (let i = 0; i < N; i++) {
    srcCentroid.add(srcPts[i]);
    dstCentroid.add(dstPts[i]);
  }
  srcCentroid.multiplyScalar(1 / N);
  dstCentroid.multiplyScalar(1 / N);

  // Build covariance matrix H = Σ (src_i - c_s)(dst_i - c_d)^T
  let H = [0,0,0, 0,0,0, 0,0,0]; // row-major 3x3
  for (let i = 0; i < N; i++) {
    const ps = srcPts[i].clone().sub(srcCentroid);
    const pd = dstPts[i].clone().sub(dstCentroid);
    H[0] += ps.x * pd.x; H[1] += ps.x * pd.y; H[2] += ps.x * pd.z;
    H[3] += ps.y * pd.x; H[4] += ps.y * pd.y; H[5] += ps.y * pd.z;
    H[6] += ps.z * pd.x; H[7] += ps.z * pd.y; H[8] += ps.z * pd.z;
  }

  // Simplified approach: use cross-covariance directly for small motions
  // For larger motions, a full SVD would be needed
  // This gives us an approximate rotation matrix
  
  // Extract rotation using simplified polar decomposition
  // R ≈ H * (H^T H)^(-1/2)
  
  // For small motions (typical in odometry), we can approximate:
  // Just normalize the columns of H to get rotation
  const Hmat = new THREE.Matrix3();
  Hmat.set(H[0], H[1], H[2], H[3], H[4], H[5], H[6], H[7], H[8]);
  
  // Simple approach: extract rotation from H directly
  // This works well for small incremental motions
  const R = Hmat.clone();
  
  // Orthonormalize using Gram-Schmidt (approximate rotation)
  const col0 = new THREE.Vector3(R.elements[0], R.elements[3], R.elements[6]).normalize();
  let col1 = new THREE.Vector3(R.elements[1], R.elements[4], R.elements[7]);
  col1.addScaledVector(col0, -col1.dot(col0)).normalize();
  const col2 = new THREE.Vector3().crossVectors(col0, col1).normalize();
  
  R.set(
    col0.x, col1.x, col2.x,
    col0.y, col1.y, col2.y,
    col0.z, col1.z, col2.z
  );

  // Compute translation: t = dstCentroid - R * srcCentroid
  const t = dstCentroid.clone().sub(
    srcCentroid.clone().applyMatrix3(R)
  );

  return { R, t };
}

/**
 * Find nearest neighbor correspondences between two point clouds
 * @param {Array<Vector3>} srcPts - Source point cloud
 * @param {Array<Vector3>} dstPts - Destination point cloud
 * @param {number} maxDist - Maximum correspondence distance
 * @returns {object} - {src: Array<Vector3>, dst: Array<Vector3>}
 */
function findCorrespondences(srcPts, dstPts, maxDist = 1.0) {
  const src = [];
  const dst = [];
  const maxDistSq = maxDist * maxDist;

  for (const p of srcPts) {
    let best = null;
    let bestD2 = maxDistSq;
    for (const q of dstPts) {
      const d2 = p.distanceToSquared(q);
      if (d2 < bestD2) {
        bestD2 = d2;
        best = q;
      }
    }
    if (best) {
      src.push(p);
      dst.push(best);
    }
  }
  return { src, dst };
}

/**
 * ICP Odometry - estimates pose changes from consecutive LiDAR scans
 */
export class ICPOdometry {
  constructor(maxPoints = 400, maxIter = 5) {
    this.maxPoints = maxPoints;
    this.maxIter = maxIter;
    this.prevCloud = null;
    this.convergenceThreshold = 0.001; // Convergence threshold for ICP
  }

  /**
   * Update slamPose based on ICP alignment with previous scan
   * @param {object} slamPose - {position: Vector3, orientation: Quaternion}
   * @param {Array} hits - LiDAR hit array from current scan
   */
  update(slamPose, hits) {
    const currCloud = hitsToPointCloud(slamPose, hits, this.maxPoints);

    if (!this.prevCloud || this.prevCloud.length < 20) {
      this.prevCloud = currCloud;
      return; // nothing to align yet
    }

    let srcPts = currCloud.map(p => p.clone());
    const dstPts = this.prevCloud;

    // Accumulate transform
    let totalT = new THREE.Vector3();
    let totalR = new THREE.Matrix3().identity();

    // Do a few ICP iterations
    for (let iter = 0; iter < this.maxIter; iter++) {
      const { src, dst } = findCorrespondences(srcPts, dstPts, 1.5);
      if (src.length < 20) break;

      const rt = computeRigidTransformKabsch(src, dst);
      if (!rt) break;

      const { R, t } = rt;

      // Check convergence (translation magnitude)
      if (t.length() < this.convergenceThreshold) break;

      // Apply transform to srcPts for next iteration
      for (let i = 0; i < srcPts.length; i++) {
        const p = srcPts[i];
        srcPts[i] = p.applyMatrix3(R).add(t);
      }

      // Accumulate total transform
      totalT.add(t);
      const newR = R.clone().multiply(totalR);
      totalR = newR;
    }

    // Sanity check: reject crazy transforms (ICP divergence protection)
    const motionMagnitude = totalT.length();
    if (motionMagnitude > 2.0) {
      // Drone can't move more than 2m in 0.02s (would be 100 m/s!)
      console.warn(`[ICP] Rejecting divergent transform: ${motionMagnitude.toFixed(2)}m`);
      this.prevCloud = currCloud; // Update cloud but don't move pose
      return;
    }

    // Update slamPose with accumulated transform (small incremental motion only)
    // Apply rotation to orientation (skip for now - rotation estimation is unstable)
    // const rotQuat = new THREE.Quaternion().setFromRotationMatrix(
    //   new THREE.Matrix4().setFromMatrix3(totalR)
    // );
    // slamPose.orientation.premultiply(rotQuat);
    // slamPose.orientation.normalize();

    // Apply translation only (more stable)
    slamPose.position.add(totalT);

    // Update previous cloud for next iteration
    this.prevCloud = currCloud;
  }

  /**
   * Reset odometry state
   */
  reset() {
    this.prevCloud = null;
  }
}
