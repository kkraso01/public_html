/**
 * Torus Gate Geometry Model for Drone Racing
 * 
 * Represents a torus (donut-shaped) gate with utilities for:
 * - Signed distance to torus surface (for collision avoidance)
 * - Distance to ring centerline (for optimal racing line)
 * 
 * Torus parameterization:
 * - center: gate position in world frame
 * - axis: normal vector to gate plane (drone should fly along this)
 * - R: major radius (ring center to centerline)
 * - r: minor radius (tube thickness)
 */

export class TorusGate {
  constructor(center, axis, majorRadius, minorRadius) {
    this.center = center.clone();            // THREE.Vector3
    this.axis = axis.clone().normalize();    // THREE.Vector3 (normal of gate plane)
    this.R = majorRadius;                    // ring radius (from center to ring circle)
    this.r = minorRadius;                    // tube radius (how thick the ring is)
  }

  /**
   * Signed distance to the torus surface
   * Negative = inside tube (safe)
   * Positive = outside tube (collision risk)
   * Zero = on surface
   */
  signedDistance(point) {
    const rel = point.clone().sub(this.center);

    // Decompose rel into components parallel and perpendicular to gate axis
    const proj = this.axis.clone().multiplyScalar(rel.dot(this.axis));  // along axis
    const inPlane = rel.clone().sub(proj);  // in gate plane

    const inPlaneLen = inPlane.length();
    const ringCenter = inPlaneLen - this.R;

    // Distance to the torus tube
    return Math.sqrt(ringCenter * ringCenter + proj.lengthSq()) - this.r;
  }

  /**
   * Distance to ring centerline (for "go through center" cost)
   * This is the optimal racing line through the gate
   */
  distanceToRingCenterline(point) {
    const rel = point.clone().sub(this.center);
    const proj = this.axis.clone().multiplyScalar(rel.dot(this.axis));
    const inPlane = rel.clone().sub(proj);
    const inPlaneLen = inPlane.length();

    if (inPlaneLen < 1e-6) return this.R; // on axis, return major radius
    
    // Point on ring centerline closest to query point
    const ringCenterPoint = inPlane.clone().multiplyScalar(this.R / inPlaneLen);
    return ringCenterPoint.add(this.center).distanceTo(point);
  }

  /**
   * Check if drone has passed through the gate
   * Returns true when drone crosses the gate plane and is close to centerline
   */
  hasPassed(position, previousPosition) {
    const rel = position.clone().sub(this.center);
    const prevRel = previousPosition.clone().sub(this.center);
    
    const axisDist = rel.dot(this.axis);
    const prevAxisDist = prevRel.dot(this.axis);
    
    // Check if crossed gate plane (from negative to positive side)
    const crossedPlane = prevAxisDist <= 0 && axisDist > 0;
    
    // Check if close enough to centerline (within tube)
    const radialDist = this.distanceToRingCenterline(position);
    const withinTube = radialDist < this.r * 1.5;
    
    return crossedPlane && withinTube;
  }
}
