export class DraggableControls {
  constructor(camera, domElement, obstacleManager, { plane, onDragStart, onDragEnd } = {}) {
    this.camera = camera;
    this.domElement = domElement;
    this.manager = obstacleManager;
    this.plane = plane || new THREE.Plane(new THREE.Vector3(0, 1, 0), 0);
    this.onDragStart = onDragStart;
    this.onDragEnd = onDragEnd;

    this.raycaster = new THREE.Raycaster();
    this.dragged = null;
    this._lastIntersect = new THREE.Vector3();
    this._bindEvents();
  }

  dispose() {
    this.domElement.removeEventListener('pointerdown', this._onDown);
    window.removeEventListener('pointermove', this._onMove);
    window.removeEventListener('pointerup', this._onUp);
  }

  _bindEvents() {
    this._onDown = (ev) => this._handleDown(ev);
    this._onMove = (ev) => this._handleMove(ev);
    this._onUp = () => this._handleUp();
    this.domElement.addEventListener('pointerdown', this._onDown);
    window.addEventListener('pointermove', this._onMove);
    window.addEventListener('pointerup', this._onUp);
  }

  _ndcFromEvent(ev) {
    const rect = this.domElement.getBoundingClientRect();
    return new THREE.Vector2(
      ((ev.clientX - rect.left) / rect.width) * 2 - 1,
      -((ev.clientY - rect.top) / rect.height) * 2 + 1,
    );
  }

  _intersect(ev) {
    const ndc = this._ndcFromEvent(ev);
    this.raycaster.setFromCamera(ndc, this.camera);
    const intersects = this.raycaster.intersectObjects(this.manager.getInteractableMeshes(), false);
    return intersects[0];
  }

  _handleDown(ev) {
    const hit = this._intersect(ev);
    if (!hit) return;
    this.dragged = this.manager.obstacles.find((o) => o.mesh === hit.object);
    if (!this.dragged) return;
    this.manager.setKinematic(this.dragged, true);
    this._lastIntersect.copy(hit.point);
    this._highlight(this.dragged, true);
    if (this.onDragStart) this.onDragStart(this.dragged);
  }

  _handleMove(ev) {
    if (!this.dragged) return;
    const ndc = this._ndcFromEvent(ev);
    this.raycaster.setFromCamera(ndc, this.camera);
    const target = new THREE.Vector3();
    this.raycaster.ray.intersectPlane(this.plane, target);
    if (!Number.isFinite(target.x)) return;
    target.y = Math.max(target.y, 0.05);
    this.manager.updatePose(this.dragged, target);
    this._lastIntersect.copy(target);
  }

  _handleUp() {
    if (!this.dragged) return;
    this.manager.setKinematic(this.dragged, false);
    this._highlight(this.dragged, false);
    if (this.onDragEnd) this.onDragEnd(this.dragged, this._lastIntersect.clone());
    this.dragged = null;
  }

  _highlight(obstacle, enabled) {
    if (!obstacle || !obstacle.mesh.material) return;
    if (!obstacle._baseEmissive) obstacle._baseEmissive = obstacle.mesh.material.emissive?.clone?.();
    if (enabled) {
      obstacle.mesh.material.emissive = (obstacle.mesh.material.emissive || new THREE.Color()).clone().addScalar(0.25);
    } else if (obstacle._baseEmissive) {
      obstacle.mesh.material.emissive.copy(obstacle._baseEmissive);
    }
  }
}
