export class Lidar {
  constructor({ numRays = 80, maxRange = 18, fovHorizontal = 120, fovVertical = 40 } = {}) {
    this.numRays = numRays;
    this.maxRange = maxRange;
    this.fovHorizontal = fovHorizontal * Math.PI / 180; // Convert to radians
    this.fovVertical = fovVertical * Math.PI / 180;
    // Create cone pattern - grid of rays
    this.raysPerRow = Math.ceil(Math.sqrt(numRays * 2)); // More horizontal rays
    this.rows = Math.ceil(numRays / this.raysPerRow);
  }

  scan(sim, pose) {
    const hits = [];
    
    // Forward-facing cone in body frame (+X is forward, +Y is left, +Z is up)
    let rayIndex = 0;
    for (let row = 0; row < this.rows && rayIndex < this.numRays; row++) {
      for (let col = 0; col < this.raysPerRow && rayIndex < this.numRays; col++) {
        // Map to [-0.5, 0.5] range then scale by FOV
        const horizontalAngle = ((col / (this.raysPerRow - 1)) - 0.5) * this.fovHorizontal;
        const verticalAngle = ((row / (this.rows - 1)) - 0.5) * this.fovVertical;
        
        // Create direction vector in body frame
        // Body frame: +X forward (nose), +Y left (port), +Z up
        // Horizontal sweep: rotation around Z axis (yaw in body frame)
        // Vertical sweep: rotation around Y axis (pitch in body frame)
        const cosV = Math.cos(verticalAngle);
        const sinV = Math.sin(verticalAngle);
        const cosH = Math.cos(horizontalAngle);
        const sinH = Math.sin(horizontalAngle);
        
        // Direction in body frame (before applying drone orientation)
        const dirBody = new THREE.Vector3(
          cosV * cosH,  // X component (forward)
          cosV * sinH,  // Y component (left/right)
          sinV          // Z component (up/down)
        );
        
        // Transform from body frame to world frame using drone's orientation
        const dir = dirBody.applyQuaternion(pose.orientation);
        dir.normalize();
        
        const hit = sim.raycast(pose.position, dir, this.maxRange);
        if (hit) {
          hits.push({ distance: hit.distance, direction: dir.clone(), point: hit.point.clone(), hit: true });
        } else {
          // Ray didn't hit anything - add max range endpoint for visualization
          const endPoint = pose.position.clone().add(dir.clone().multiplyScalar(this.maxRange));
          hits.push({ distance: this.maxRange, direction: dir.clone(), point: endPoint, hit: false });
        }
        rayIndex++;
      }
    }
    return hits;
  }
}
