function createBoxMesh(size, position, material) {
  const mesh = new THREE.Mesh(new THREE.BoxGeometry(size.x, size.y, size.z), material);
  mesh.position.copy(position);
  mesh.castShadow = true;
  mesh.receiveShadow = true;
  return mesh;
}

export class CaveSim {
  constructor(scene) {
    this.scene = scene;
    this.obstacles = [];
    this._initEnvironment();
  }

  _initEnvironment() {
    const wallMaterial = new THREE.MeshStandardMaterial({ color: 0x0b1220, roughness: 0.8, metalness: 0.05 });
    const ground = new THREE.Mesh(new THREE.PlaneGeometry(60, 60), wallMaterial);
    ground.rotation.x = -Math.PI / 2;
    ground.receiveShadow = true;
    this.scene.add(ground);

    const rimMaterial = new THREE.MeshStandardMaterial({ color: 0x1f2937, roughness: 0.6 });
    const radius = 12;
    for (let i = 0; i < 10; i++) {
      const angle = (i / 10) * Math.PI * 2;
      const pos = new THREE.Vector3(Math.cos(angle) * radius, 2, Math.sin(angle) * radius);
      const box = createBoxMesh(new THREE.Vector3(3, 3.5, 0.6), pos, rimMaterial);
      box.rotation.y = -angle;
      this.scene.add(box);
      this._addObstacleBox(box, new THREE.Vector3(3, 3.5, 0.6));
    }

    for (let i = 0; i < 6; i++) {
      const x = (Math.random() - 0.5) * 14;
      const z = (Math.random() - 0.5) * 14;
      const size = new THREE.Vector3(1.5 + Math.random(), 2 + Math.random() * 2, 1.5 + Math.random());
      const column = createBoxMesh(size, new THREE.Vector3(x, size.y / 2, z), rimMaterial);
      this.scene.add(column);
      this._addObstacleBox(column, size);
    }
  }

  _addObstacleBox(mesh, size) {
    const half = size.clone().multiplyScalar(0.5);
    const min = mesh.position.clone().sub(new THREE.Vector3(half.x, half.y, half.z));
    const max = mesh.position.clone().add(new THREE.Vector3(half.x, half.y, half.z));
    this.obstacles.push({ min, max });
  }

  raycast(origin, direction, maxRange = 18) {
    let closest = null;
    this.obstacles.forEach((obs) => {
      const hit = this._intersectAABB(origin, direction, obs.min, obs.max, maxRange);
      if (hit && (!closest || hit.distance < closest.distance)) {
        closest = hit;
      }
    });
    return closest;
  }

  _intersectAABB(origin, dir, min, max, maxRange) {
    let tmin = 0;
    let tmax = maxRange;
    for (const axis of ['x', 'y', 'z']) {
      const invD = 1 / (dir[axis] === 0 ? 1e-6 : dir[axis]);
      let t0 = (min[axis] - origin[axis]) * invD;
      let t1 = (max[axis] - origin[axis]) * invD;
      if (invD < 0) [t0, t1] = [t1, t0];
      tmin = Math.max(tmin, t0);
      tmax = Math.min(tmax, t1);
      if (tmax <= tmin) return null;
    }
    const distance = tmin;
    if (distance < 0 || distance > maxRange) return null;
    const point = origin.clone().add(dir.clone().multiplyScalar(distance));
    return { distance, point };
  }

  collide(position, radius = 0.25) {
    return this.obstacles.some((obs) => {
      const closest = new THREE.Vector3(
        clamp(position.x, obs.min.x, obs.max.x),
        clamp(position.y, obs.min.y, obs.max.y),
        clamp(position.z, obs.min.z, obs.max.z),
      );
      return closest.distanceTo(position) <= radius;
    });
  }
}

function clamp(x, min, max) {
  return Math.min(Math.max(x, min), max);
}
