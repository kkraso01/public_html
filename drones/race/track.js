export function buildRaceTrack(scene) {
  const gates = [];
  const gateMaterial = new THREE.MeshStandardMaterial({
    color: 0x8b5cf6,
    emissive: 0x7c3aed,
    emissiveIntensity: 1.2,
    side: THREE.DoubleSide,
  });
  
  // Create starting platform (+Z is up convention)
  // BoxGeometry(width=X, height=Y, depth=Z) - platform should be wide in X and Y, thin in Z
  const platformGeometry = new THREE.BoxGeometry(3, 3, 0.2);
  const platformMaterial = new THREE.MeshStandardMaterial({
    color: 0x10b981,
    emissive: 0x059669,
    emissiveIntensity: 0.6,
    metalness: 0.4,
    roughness: 0.6,
  });
  const platform = new THREE.Mesh(platformGeometry, platformMaterial);
  platform.position.set(-15, 0, 2.4); // +Z is altitude, moved further back on -X axis
  platform.castShadow = true;
  platform.receiveShadow = true;
  scene.add(platform);
  
  const radius = 1.5;  // Larger torus radius for easier passing
  const gatePositions = [];
  
  // Create vertical rings at different positions for increased difficulty (+Z is up, X is forward)
  // Zig-zag pattern: drone flies forward (+X) while weaving left and right through rings
  const ringConfig = [
    { x: 5, y: 0, z: 2.5 },      // Center
    { x: 9, y: 3, z: 2.5 },      // Right
    { x: 13, y: -3, z: 2.5 },    // Left
    { x: 17, y: 3, z: 2.5 },     // Right
    { x: 21, y: -3, z: 2.5 },    // Left
    { x: 25, y: 3, z: 2.5 },     // Right
    { x: 29, y: -3, z: 2.5 },    // Left
    { x: 33, y: 3, z: 2.5 },     // Right
    { x: 37, y: -3, z: 2.5 },    // Left
    { x: 41, y: 0, z: 2.5 },     // Center finish
  ];
  
  ringConfig.forEach((config) => {
    gatePositions.push(new THREE.Vector3(config.x, config.y, config.z));
  })

  gatePositions.forEach((pos) => {
    // Vertical ring perpendicular to flight path (Y-Z plane, since drone flies along +X)
    // TorusGeometry by default is in X-Y plane (normal in +Z direction)
    // We need it in Y-Z plane (normal in +X direction)
    // Rotate 90 degrees around Y-axis to achieve this
    const gate = new THREE.Mesh(new THREE.TorusGeometry(radius, 0.08, 12, 60), gateMaterial);
    gate.position.copy(pos);
    gate.rotation.y = Math.PI / 2; // Rotate around Y to make ring in Y-Z plane
    gate.castShadow = true;
    gate.receiveShadow = true;
    scene.add(gate);
    gates.push(gate);
  });

  const trackLine = new THREE.Line(
    new THREE.BufferGeometry().setFromPoints(gatePositions),
    new THREE.LineDashedMaterial({ color: 0x94a3b8, dashSize: 0.35, gapSize: 0.15 }),
  );
  trackLine.computeLineDistances();
  scene.add(trackLine);

  // Waypoints pass through exact center of each ring
  // Start on platform -> Approach to first gate -> Then proceed through rings
  const waypoints = [
    new THREE.Vector3(-15, 0, 2.4),    // Start on platform
    new THREE.Vector3(-5, 0, 2.5),     // Approach to first gate
    ...gatePositions
  ];
  
  return {
    gates,
    waypoints,
  };
}

export class ReferenceTrajectory {
  constructor(waypoints, totalTime) {
    this.points = waypoints.map((p) => p.clone());
    this.totalTime = totalTime;
    this.segmentTime = totalTime / Math.max(1, this.points.length - 1);
  }

  _catmullRom(p0, p1, p2, p3, t) {
    const t2 = t * t;
    const t3 = t2 * t;
    return p1
      .clone()
      .multiplyScalar(2)
      .add(p2.clone().sub(p0).multiplyScalar(t))
      .add(p0.clone().multiplyScalar(2).sub(p1.clone().multiplyScalar(5)).add(p2.clone().multiplyScalar(4)).sub(p3).multiplyScalar(t2))
      .add(p0.clone().negate().add(p1.clone().multiplyScalar(3)).sub(p2.clone().multiplyScalar(3)).add(p3).multiplyScalar(t3))
      .multiplyScalar(0.5);
  }

  sample(time) {
    const clamped = Math.min(Math.max(time, 0), this.totalTime * 0.999);
    const idx = Math.floor(clamped / this.segmentTime);
    const localT = (clamped - idx * this.segmentTime) / this.segmentTime;

    const i0 = Math.max(0, idx - 1);
    const i1 = Math.max(0, idx);
    const i2 = Math.min(this.points.length - 1, idx + 1);
    const i3 = Math.min(this.points.length - 1, idx + 2);

    const p = this._catmullRom(this.points[i0], this.points[i1], this.points[i2], this.points[i3], localT);
    const eps = 0.01;
    const pFwd = this._catmullRom(this.points[i0], this.points[i1], this.points[i2], this.points[i3], Math.min(1, localT + eps));
    const pBack = this._catmullRom(this.points[i0], this.points[i1], this.points[i2], this.points[i3], Math.max(0, localT - eps));
    const vel = pFwd.clone().sub(pBack).multiplyScalar(1 / (2 * eps * this.segmentTime));
    const acc = pFwd
      .clone()
      .add(pBack)
      .sub(p.clone().multiplyScalar(2))
      .multiplyScalar(1 / (eps * eps * this.segmentTime * this.segmentTime));

    const yaw = vel.lengthSq() > 1e-6 ? Math.atan2(vel.x, vel.z) : 0;
    return { position: p, velocity: vel, acceleration: acc, yaw };
  }

  // Alias for compatibility with different controller interfaces
  getStateAtTime(time) {
    return this.sample(time);
  }
}
