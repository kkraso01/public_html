// Library of visually rich obstacle presets for the cave simulator.
// Each preset returns geometry + material + collider metadata so the ObstacleManager
// can create Ammo bodies with sane masses and friction.

const DRAB_NOISE = 0.05;

function addVertexNoise(geometry, amplitude = DRAB_NOISE) {
  const pos = geometry.attributes.position;
  for (let i = 0; i < pos.count; i++) {
    const nx = (Math.random() - 0.5) * amplitude;
    const ny = (Math.random() - 0.5) * amplitude;
    const nz = (Math.random() - 0.5) * amplitude;
    pos.setXYZ(i, pos.getX(i) + nx, pos.getY(i) + ny, pos.getZ(i) + nz);
  }
  pos.needsUpdate = true;
  geometry.computeVertexNormals();
}

export const OBSTACLE_TYPES = {
  boulder: 'boulder',
  rock: 'rock',
  crate: 'crate',
  column: 'column',
  plate: 'plate',
};

function randomRoughMaterial(baseColor) {
  return new THREE.MeshStandardMaterial({
    color: baseColor,
    metalness: 0.05 + Math.random() * 0.1,
    roughness: 0.65 + Math.random() * 0.2,
    emissive: new THREE.Color(baseColor).multiplyScalar(0.05),
  });
}

export function buildObstaclePreset(type, overrides = {}) {
  const seed = Math.random();
  let preset;

  switch (type) {
    case OBSTACLE_TYPES.crate: {
      const size = overrides.size || new THREE.Vector3(0.6 + seed * 0.2, 0.6 + seed * 0.1, 0.6 + seed * 0.2);
      preset = {
        label: 'Crate',
        geometry: new THREE.BoxGeometry(size.x, size.y, size.z),
        material: randomRoughMaterial(0x6b7280),
        collider: { type: 'box', halfExtents: size.clone().multiplyScalar(0.5) },
        mass: 4.5 * size.x * size.y * size.z,
      };
      break;
    }
    case OBSTACLE_TYPES.column: {
      const radius = 0.35 + seed * 0.15;
      const height = 1.6 + seed * 0.6;
      const geom = new THREE.CylinderGeometry(radius * 0.9, radius, height, 16, 1, false);
      preset = {
        label: 'Column',
        geometry: geom,
        material: randomRoughMaterial(0x94a3b8),
        collider: { type: 'box', halfExtents: new THREE.Vector3(radius, height * 0.5, radius) },
        mass: 6 * radius * radius * height,
      };
      break;
    }
    case OBSTACLE_TYPES.rock: {
      const radius = 0.45 + seed * 0.25;
      const geom = new THREE.DodecahedronGeometry(radius, 1);
      addVertexNoise(geom, 0.08);
      preset = {
        label: 'Rock',
        geometry: geom,
        material: randomRoughMaterial(0x475569),
        collider: { type: 'hull' },
        mass: 8 * radius * radius * radius,
      };
      break;
    }
    case OBSTACLE_TYPES.plate: {
      const scale = 0.8 + seed * 0.6;
      const thickness = 0.08 + seed * 0.04;
      const geom = new THREE.BoxGeometry(1.6 * scale, thickness, 1.1 * scale);
      addVertexNoise(geom, 0.04);
      preset = {
        label: 'Plate',
        geometry: geom,
        material: randomRoughMaterial(0x1f2937),
        collider: { type: 'box', halfExtents: new THREE.Vector3(0.8 * scale, thickness * 0.5, 0.55 * scale) },
        mass: 2.2 * scale,
      };
      break;
    }
    case OBSTACLE_TYPES.boulder:
    default: {
      const radius = 0.55 + seed * 0.35;
      const geom = new THREE.IcosahedronGeometry(radius, 2);
      addVertexNoise(geom, 0.12);
      preset = {
        label: 'Boulder',
        geometry: geom,
        material: randomRoughMaterial(0x64748b),
        collider: { type: 'sphere', radius },
        mass: 10 * radius * radius * radius,
      };
    }
  }

  if (overrides.material) preset.material = overrides.material;
  if (overrides.mass) preset.mass = overrides.mass;
  if (overrides.collider) preset.collider = overrides.collider;

  return preset;
}

export function listPresets() {
  return [
    { type: OBSTACLE_TYPES.boulder, label: 'Boulder' },
    { type: OBSTACLE_TYPES.rock, label: 'Rock' },
    { type: OBSTACLE_TYPES.crate, label: 'Crate' },
    { type: OBSTACLE_TYPES.column, label: 'Column' },
    { type: OBSTACLE_TYPES.plate, label: 'Plate' },
  ];
}
