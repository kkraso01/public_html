export class ObstacleManager {
  constructor(scene) {
    this.scene = scene;
    this.obstacles = [];
  }

  addObstacle(position, radius = 0.4, color = 0xf59e0b) {
    const mesh = new THREE.Mesh(
      new THREE.SphereGeometry(radius, 14, 14),
      new THREE.MeshStandardMaterial({ color, emissive: new THREE.Color(color).multiplyScalar(0.2) }),
    );
    mesh.position.copy(position);
    mesh.castShadow = true;
    mesh.receiveShadow = true;
    this.scene.add(mesh);
    const obstacle = { mesh, radius, kinematic: false };
    this.obstacles.push(obstacle);
    return obstacle;
  }

  getInteractableMeshes() {
    return this.obstacles.map((o) => o.mesh);
  }

  setKinematic(obstacle, value) {
    obstacle.kinematic = value;
  }

  updatePose(obstacle, position) {
    obstacle.mesh.position.copy(position);
  }

  collide(position, radius = 0.25) {
    return this.obstacles.some((o) => position.distanceTo(o.mesh.position) < o.radius + radius);
  }
}
