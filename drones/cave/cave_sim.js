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
    // Ground at Z=0 (XY plane) - Z-up convention, no rotation needed
    // Note: floor is already added in drone_cave_demo.js, don't add duplicate
    
    // Varied colors for cave walls - orange/brown/tan rock tones
    const rockColors = [0x8B6F47, 0xA0826D, 0x6B4423, 0x9C6644, 0x755C48];
    const radius = 12;
    for (let i = 0; i < 10; i++) {
      const angle = (i / 10) * Math.PI * 2;
      // Z-up: walls around perimeter, positioned in XY plane with Z height
      const pos = new THREE.Vector3(
        Math.cos(angle) * radius,  // X position
        Math.sin(angle) * radius,  // Y position
        2                          // Z altitude (wall height center)
      );
      const rimMaterial = new THREE.MeshStandardMaterial({ 
        color: rockColors[i % rockColors.length], 
        roughness: 0.7 + Math.random() * 0.2,
        metalness: 0.05 
      });
      // Wall dimensions: width (X/Y), depth (Y/X), height (Z)
      const box = createBoxMesh(new THREE.Vector3(3, 0.6, 3.5), pos, rimMaterial);
      box.rotation.z = -angle; // Rotate around Z axis to face center
      this.scene.add(box);
      this._addObstacleBox(box, new THREE.Vector3(3, 0.6, 3.5));
    }

    // Colored stalactites/columns - varied earth tones
    const columnColors = [0x7A5C3D, 0x9B7653, 0x654321, 0x8B7355, 0x6F5438, 0xA68064];
    const startClearRadius = 3.0; // Keep area clear around drone start position (0,0)
    
    for (let i = 0; i < 6; i++) {
      let x, y;
      let attempts = 0;
      // Find position that's not too close to origin (drone start)
      do {
        x = (Math.random() - 0.5) * 14;
        y = (Math.random() - 0.5) * 14;
        attempts++;
      } while (Math.sqrt(x*x + y*y) < startClearRadius && attempts < 20);
      
      // Z-up: columns extend vertically in Z direction
      const columnHeight = 2 + Math.random() * 2;
      const size = new THREE.Vector3(
        1.5 + Math.random(),  // X width
        1.5 + Math.random(),  // Y depth
        columnHeight          // Z height
      );
      const columnMaterial = new THREE.MeshStandardMaterial({ 
        color: columnColors[i % columnColors.length],
        roughness: 0.75 + Math.random() * 0.15,
        metalness: 0.05
      });
      // Position at ground (Z=0) plus half height
      const column = createBoxMesh(size, new THREE.Vector3(x, y, size.z / 2), columnMaterial);
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
