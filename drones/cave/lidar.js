export class Lidar {
  constructor({ numRays = 36, maxRange = 18, verticalSpread = 0.2 } = {}) {
    this.numRays = numRays;
    this.maxRange = maxRange;
    this.verticalSpread = verticalSpread;
  }

  scan(sim, pose) {
    const hits = [];
    for (let i = 0; i < this.numRays; i++) {
      const theta = (i / this.numRays) * Math.PI * 2;
      const pitch = (Math.random() - 0.5) * this.verticalSpread;
      const dir = new THREE.Vector3(Math.cos(theta), pitch, Math.sin(theta)).applyQuaternion(pose.orientation);
      dir.normalize();
      const hit = sim.raycast(pose.position, dir, this.maxRange);
      if (hit) {
        hits.push({ distance: hit.distance, direction: dir.clone(), point: hit.point.clone() });
      }
    }
    return hits;
  }
}
