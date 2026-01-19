// Ammo ray-based LiDAR utility used by the autonomous planner.

const ammoVec3 = (v) => new Ammo.btVector3(v.x, v.y, v.z);

export class AmmoLidar {
  constructor(world, { rays = 48, maxRange = 20, verticalSpread = 0.25 } = {}) {
    this.world = world;
    this.rays = rays;
    this.maxRange = maxRange;
    this.verticalSpread = verticalSpread;
  }

  scan(pose) {
    if (!this.world) return [];
    const hits = [];
    const start = ammoVec3(pose.position);
    for (let i = 0; i < this.rays; i++) {
      const theta = (i / this.rays) * Math.PI * 2;
      const pitch = (Math.random() - 0.5) * this.verticalSpread;
      const dir = new THREE.Vector3(Math.cos(theta), pitch, Math.sin(theta)).applyQuaternion(pose.quaternion).normalize();
      const end = ammoVec3(pose.position.clone().add(dir.clone().multiplyScalar(this.maxRange)));
      const callback = new Ammo.ClosestRayResultCallback(start, end);
      this.world.rayTest(start, end, callback);
      if (callback.hasHit()) {
        const p = callback.get_hitPointWorld();
        hits.push({
          distance: pose.position.distanceTo(new THREE.Vector3(p.x(), p.y(), p.z())),
          direction: dir,
          point: new THREE.Vector3(p.x(), p.y(), p.z()),
        });
      }
      Ammo.destroy(callback);
      Ammo.destroy(end);
    }
    Ammo.destroy(start);
    return hits;
  }
}
