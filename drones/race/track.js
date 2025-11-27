export function buildRaceTrack(scene) {
  const gates = [];
  const gateMaterial = new THREE.MeshStandardMaterial({
    color: 0x8b5cf6,
    emissive: 0x7c3aed,
    emissiveIntensity: 1.2,
    side: THREE.DoubleSide,
  });
  const radius = 1.1;
  const gatePositions = [];
  for (let i = 0; i < 8; i++) {
    const angle = (i / 8) * Math.PI * 2;
    const x = Math.cos(angle) * 8;
    const z = Math.sin(angle) * 8;
    const y = 2 + Math.sin(angle * 2) * 1.1;
    gatePositions.push(new THREE.Vector3(x, y, z));
  }

  gatePositions.forEach((pos) => {
    const gate = new THREE.Mesh(new THREE.TorusGeometry(radius, 0.05, 12, 60), gateMaterial);
    gate.position.copy(pos);
    gate.rotation.x = Math.PI / 2;
    gate.castShadow = true;
    gate.receiveShadow = true;
    scene.add(gate);
    gates.push(gate);
  });

  const trackLine = new THREE.Line(
    new THREE.BufferGeometry().setFromPoints(gatePositions.concat([gatePositions[0]])),
    new THREE.LineDashedMaterial({ color: 0x94a3b8, dashSize: 0.35, gapSize: 0.15 }),
  );
  trackLine.computeLineDistances();
  scene.add(trackLine);

  return {
    gates,
    waypoints: [new THREE.Vector3(0, 2, 10), ...gatePositions, new THREE.Vector3(0, 2, 10)],
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
}
