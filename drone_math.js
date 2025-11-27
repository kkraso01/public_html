// Common math utilities shared across demos.
// Lightweight helpers to keep the physics and control code focused on the dynamics.

export function clamp(value, min, max) {
  return Math.min(Math.max(value, min), max);
}

export function lerp(a, b, t) {
  return a + (b - a) * t;
}

export function smoothstep(edge0, edge1, x) {
  const t = clamp((x - edge0) / (edge1 - edge0), 0, 1);
  return t * t * (3 - 2 * t);
}

export function quatFromEuler(roll, pitch, yaw) {
  const q = new THREE.Quaternion();
  q.setFromEuler(new THREE.Euler(roll, pitch, yaw, 'XYZ'));
  return q;
}

export function eulerFromQuat(q) {
  const e = new THREE.Euler();
  e.setFromQuaternion(q, 'XYZ');
  return e;
}

export function skewMatrix(v) {
  return new THREE.Matrix3().set(
    0, -v.z, v.y,
    v.z, 0, -v.x,
    -v.y, v.x, 0,
  );
}

// Utility to transform a body-axis vector into world frame given a quaternion.
export function bodyToWorld(q, vBody) {
  return vBody.clone().applyQuaternion(q);
}

export function worldToBody(q, vWorld) {
  const inv = q.clone().invert();
  return vWorld.clone().applyQuaternion(inv);
}

// Gaussian noise helper used by sensors.
export function gaussianNoise(mean = 0, std = 1) {
  // Box-Muller transform
  let u = 0;
  let v = 0;
  while (u === 0) u = Math.random();
  while (v === 0) v = Math.random();
  return mean + std * Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v);
}
