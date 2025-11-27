import { DronePhysicsEngine } from '../physics/drone_physics_engine.js';
import { ObstacleManager } from './obstacle_manager.js';
import { DraggableControls } from './draggable_controls.js';

function clamp(x, min, max) {
  return Math.min(Math.max(x, min), max);
}

export function initObstacleDropDemo(container, options = {}) {
  if (!container || typeof THREE === 'undefined') {
    console.warn('Obstacle demo requires a valid container and Three.js');
    return { pause() {}, resume() {}, restart() {}, setPausedFromVisibility() {}, destroy() {} };
  }
  const demo = new ObstacleDropDemo(container, options);
  demo.start();
  return {
    pause: () => demo.pause(true),
    resume: () => demo.resume(true),
    restart: () => demo.restart(),
    setPausedFromVisibility: (visible) => demo.setPausedFromVisibility(visible),
    destroy: () => demo.destroy(),
  };
}

class ObstacleDropDemo {
  constructor(container, options) {
    this.container = container;
    this.options = Object.assign({ width: container.clientWidth || 640, height: container.clientHeight || 360 }, options);
    this.physicsRate = 200;
    this.userPaused = false;
    this.visibilityPaused = false;
    this.lastFrame = null;
    this.accumulator = 0;
    this._frameReq = null;

    this._initScene();
    this._initDrone();
    this._initObstacles();
    this.restart();
  }

  _initScene() {
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x0b1220);
    this.camera = new THREE.PerspectiveCamera(60, this.options.width / this.options.height, 0.1, 80);
    this.camera.position.set(4, 3, 6);

    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderer.setSize(this.options.width, this.options.height);
    this.renderer.setPixelRatio(window.devicePixelRatio || 1);
    this.renderer.shadowMap.enabled = true;
    this.container.innerHTML = '';
    this.container.appendChild(this.renderer.domElement);

    const ambient = new THREE.AmbientLight(0x475569, 0.4);
    const dir = new THREE.DirectionalLight(0xffffff, 0.9);
    dir.position.set(5, 8, 5);
    dir.castShadow = true;
    this.scene.add(ambient, dir);

    const ground = new THREE.Mesh(
      new THREE.PlaneGeometry(30, 30),
      new THREE.MeshStandardMaterial({ color: 0x0f172a, roughness: 0.8 }),
    );
    ground.rotation.x = -Math.PI / 2;
    ground.receiveShadow = true;
    this.scene.add(ground);
  }

  _initDrone() {
    this.drone = new DronePhysicsEngine();
    this.drone.reset({ position: new THREE.Vector3(0, 1.2, 0) });

    const body = new THREE.Mesh(
      new THREE.BoxGeometry(0.25, 0.07, 0.25),
      new THREE.MeshStandardMaterial({ color: 0x38bdf8, metalness: 0.2, roughness: 0.5 }),
    );
    const rings = new THREE.Mesh(
      new THREE.RingGeometry(0.08, 0.12, 16),
      new THREE.MeshBasicMaterial({ color: 0xffffff }),
    );
    rings.rotation.x = Math.PI / 2;
    this.droneMesh = new THREE.Group();
    this.droneMesh.add(body);
    [-0.18, 0.18].forEach((x) => {
      [-0.18, 0.18].forEach((z) => {
        const r = rings.clone();
        r.position.set(x, 0.05, z);
        this.droneMesh.add(r);
      });
    });
    this.scene.add(this.droneMesh);
  }

  _initObstacles() {
    this.manager = new ObstacleManager(this.scene);
    this.manager.addObstacle(new THREE.Vector3(1.5, 0.4, 0), 0.5, 0xf97316);
    this.manager.addObstacle(new THREE.Vector3(-1.2, 0.4, -1.5), 0.45, 0x22c55e);
    this.manager.addObstacle(new THREE.Vector3(0, 0.4, 1.8), 0.55, 0x10b981);
    this.controls = new DraggableControls(this.camera, this.renderer.domElement, this.manager, {
      onDragEnd: () => {
        // nudge the controller so the vehicle reacts to the new obstacle position
        this._avoidanceVector = new THREE.Vector3();
      },
    });
  }

  restart() {
    this.drone.reset({ position: new THREE.Vector3(0, 1.2, 0), velocity: new THREE.Vector3() });
    this._avoidanceVector = new THREE.Vector3();
    this.simTime = 0;
  }

  pause(user = false) {
    if (user) this.userPaused = true;
    if (this._frameReq) cancelAnimationFrame(this._frameReq);
    this._frameReq = null;
  }

  resume(user = false) {
    if (user) this.userPaused = false;
    if (!this._frameReq && !this.userPaused && !this.visibilityPaused) {
      this.lastFrame = null;
      this._frameReq = requestAnimationFrame((t) => this._loop(t));
    }
  }

  setPausedFromVisibility(visible) {
    this.visibilityPaused = !visible;
    if (this.visibilityPaused) {
      this.pause(false);
    } else {
      this.resume(false);
    }
  }

  start() {
    this.resume(false);
  }

  destroy() {
    this.pause(false);
    this.controls?.dispose();
    this.container.innerHTML = '';
  }

  _loop(timestamp) {
    if (this.userPaused || this.visibilityPaused) return;
    if (this.lastFrame === null) this.lastFrame = timestamp;
    const dt = (timestamp - this.lastFrame) / 1000;
    this.lastFrame = timestamp;

    const fixedDt = 1 / this.physicsRate;
    this.accumulator += dt;
    while (this.accumulator >= fixedDt) {
      this._stepPhysics(fixedDt);
      this.accumulator -= fixedDt;
    }

    this._render();
    this._frameReq = requestAnimationFrame((t) => this._loop(t));
  }

  _stepPhysics(dt) {
    this.simTime += dt;
    const commands = this._computeMotorCommands();
    this.drone.applyMotorCommands(commands[0], commands[1], commands[2], commands[3]);
    this.drone.step(dt);

    if (this.manager.collide(this.drone.state.position)) {
      this.drone.state.velocity.multiplyScalar(-0.3);
    }
  }

  _computeMotorCommands() {
    const params = this.drone.params;
    const hoverRPM = Math.sqrt((params.mass * 9.81) / (4 * params.kF));
    const target = new THREE.Vector3(0, 1.2, 0);

    const nearest = this._nearestObstacle();
    let avoidance = new THREE.Vector3();
    if (nearest) {
      const diff = this.drone.state.position.clone().sub(nearest.mesh.position);
      const dist = Math.max(diff.length(), 0.01);
      const push = Math.max(0, 1.2 - dist);
      avoidance = diff.normalize().multiplyScalar(push * 2.0);
    }

    const posErr = target.clone().sub(this.drone.state.position).add(avoidance);
    const velErr = new THREE.Vector3().sub(this.drone.state.velocity);
    const desiredAcc = posErr.multiplyScalar(3.2).add(velErr.multiplyScalar(2.0));
    desiredAcc.clampLength(0, 4.5);

    const desiredYaw = 0;
    const thrustVector = desiredAcc.clone().add(new THREE.Vector3(0, 9.81, 0)).multiplyScalar(params.mass);

    const desiredOrientation = this._desiredOrientationFromThrust(thrustVector, desiredYaw);
    const qError = desiredOrientation.clone().multiply(this.drone.state.orientation.clone().invert());
    const axis = new THREE.Vector3(qError.x, qError.y, qError.z);
    const angle = 2 * Math.atan2(axis.length(), qError.w);
    const attError = axis.length() > 1e-6 ? axis.normalize().multiplyScalar(angle) : new THREE.Vector3();
    const torque = attError.multiplyScalar(4.0).add(this.drone.state.angularVelocity.clone().multiplyScalar(-0.25));

    const thrustMag = clamp(thrustVector.length(), 0, params.mass * 25);
    const kYaw = params.kM / Math.max(params.kF, 1e-9);
    const L = params.armLength;
    const m0 = 0.25 * thrustMag - torque.y / (2 * L) + torque.z / (4 * kYaw);
    const m1 = 0.25 * thrustMag + torque.x / (2 * L) - torque.z / (4 * kYaw);
    const m2 = 0.25 * thrustMag + torque.y / (2 * L) + torque.z / (4 * kYaw);
    const m3 = 0.25 * thrustMag - torque.x / (2 * L) - torque.z / (4 * kYaw);
    const thrusts = [m0, m1, m2, m3].map((m) => clamp(m, 0, thrustMag));
    const rpms = thrusts.map((t) => Math.sqrt(Math.max(t, 0) / Math.max(params.kF, 1e-9)));
    return rpms.map((r) => clamp(r, 0, params.rpmMax) || hoverRPM);
  }

  _nearestObstacle() {
    let best = null;
    let bestDist = Infinity;
    this.manager.obstacles.forEach((o) => {
      const d = this.drone.state.position.distanceTo(o.mesh.position) - o.radius;
      if (d < bestDist) {
        bestDist = d;
        best = o;
      }
    });
    return best;
  }

  _desiredOrientationFromThrust(thrustVec, yaw) {
    const zBody = thrustVec.lengthSq() > 1e-6 ? thrustVec.clone().normalize() : new THREE.Vector3(0, 1, 0);
    const xC = new THREE.Vector3(Math.sin(yaw), 0, Math.cos(yaw));
    const yBody = zBody.clone().cross(xC).normalize();
    if (yBody.lengthSq() < 1e-6) return new THREE.Quaternion();
    const xBody = yBody.clone().cross(zBody).normalize();
    const m = new THREE.Matrix4();
    m.makeBasis(xBody, yBody, zBody);
    const q = new THREE.Quaternion();
    q.setFromRotationMatrix(m);
    return q;
  }

  _render() {
    this.camera.lookAt(this.drone.state.position.clone());
    this.droneMesh.position.copy(this.drone.state.position);
    this.droneMesh.quaternion.copy(this.drone.state.orientation);
    this.renderer.render(this.scene, this.camera);
  }
}
