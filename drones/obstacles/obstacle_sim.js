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
    this.floorHeight = options.floorHeight ?? 0.05;
    this.userPaused = false;
    this.visibilityPaused = false;
    this.lastFrame = null;
    this.accumulator = 0;
    this._frameReq = null;
    this.cameraMode = 'chase';
    this.debugEnabled = false;

    this._initScene();
    this._initDrone();
    this._initObstacles();
    this._initHUD();
    this._bindInputs();
    this.restart();
  }

  _initScene() {
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x0b1220);
    const aspect = this.options.width / this.options.height;
    this.chaseCamera = new THREE.PerspectiveCamera(60, aspect, 0.05, 200);
    this.chaseCamera.position.set(4, 4, 8);
    this.overheadCamera = new THREE.PerspectiveCamera(60, aspect, 0.05, 200);
    this.overheadCamera.position.set(0, 24, 0);
    this.activeCamera = this.chaseCamera;

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
      new THREE.PlaneGeometry(40, 40),
      new THREE.MeshStandardMaterial({ color: 0x0f172a, roughness: 0.8 }),
    );
    ground.rotation.x = -Math.PI / 2;
    ground.position.y = this.floorHeight;
    ground.receiveShadow = true;
    this.scene.add(ground);

    const wallMat = new THREE.MeshStandardMaterial({ color: 0x111827, roughness: 0.9 });
    const north = new THREE.Mesh(new THREE.BoxGeometry(40, 6, 0.5), wallMat);
    north.position.set(0, 3, -20);
    const south = north.clone();
    south.position.z = 20;
    const east = new THREE.Mesh(new THREE.BoxGeometry(0.5, 6, 40), wallMat);
    east.position.set(20, 3, 0);
    const west = east.clone();
    west.position.x = -20;
    [north, south, east, west].forEach((w) => {
      w.receiveShadow = true;
      this.scene.add(w);
    });
  }

  _initDrone() {
    this.drone = new DronePhysicsEngine({ floorHeight: this.floorHeight });
    this.drone.reset({ position: new THREE.Vector3(0, 1.4, 0) });

    const body = new THREE.Mesh(
      new THREE.CapsuleGeometry(0.3, 1.0, 12, 16),
      new THREE.MeshStandardMaterial({ color: 0x38bdf8, metalness: 0.25, roughness: 0.45 }),
    );
    const arms = new THREE.Mesh(
      new THREE.BoxGeometry(1.8, 0.12, 0.12),
      new THREE.MeshStandardMaterial({ color: 0x7dd3fc, emissive: 0x0ea5e9 }),
    );
    arms.castShadow = true;
    const rings = new THREE.Mesh(
      new THREE.RingGeometry(0.16, 0.26, 18),
      new THREE.MeshBasicMaterial({ color: 0xffffff }),
    );
    rings.rotation.x = Math.PI / 2;
    this.droneMesh = new THREE.Group();
    this.droneMesh.add(body, arms);
    [-0.9, 0.9].forEach((x) => {
      [-0.9, 0.9].forEach((z) => {
        const r = rings.clone();
        r.position.set(x, 0.55, z);
        this.droneMesh.add(r);
      });
    });
    this.droneMesh.position.y = 0.8;
    this.scene.add(this.droneMesh);
  }

  _initObstacles() {
    this.manager = new ObstacleManager(this.scene);
    this.manager.addObstacle(new THREE.Vector3(1.5, 0.4, 0), 0.5, 0xf97316);
    this.manager.addObstacle(new THREE.Vector3(-1.2, 0.4, -1.5), 0.45, 0x22c55e);
    this.manager.addObstacle(new THREE.Vector3(0, 0.4, 1.8), 0.55, 0x10b981);
    this.controls = new DraggableControls(this.chaseCamera, this.renderer.domElement, this.manager, {
      onDragEnd: () => {
        // nudge the controller so the vehicle reacts to the new obstacle position
        this._avoidanceFromObstacles();
      },
    });
  }

  _initHUD() {
    this.overlay = document.createElement('div');
    this.overlay.style.cssText =
      'position:absolute; top:8px; left:8px; background:rgba(11,17,32,0.85); color:#e2e8f0; padding:10px; font-family:"Fira Code", monospace; border:1px solid rgba(14,165,233,0.35); border-radius:8px; z-index:5;';
    this.overlay.innerHTML = `
      <div style="font-size:12px; color:#7dd3fc;">Obstacle Avoider</div>
      <div id="obsState" style="font-size:12px;">Hovering + avoiding</div>
      <div id="obsAlt" style="font-size:12px;">Alt: 0.0 m</div>
    `;
    this.container.style.position = 'relative';
    this.container.appendChild(this.overlay);

    this.debugPanel = document.createElement('div');
    this.debugPanel.style.cssText =
      'position:absolute; top:8px; right:8px; background:rgba(8,10,18,0.85); color:#cbd5f5; padding:10px; font-family:"Fira Code", monospace; border:1px solid rgba(56,189,248,0.35); border-radius:8px; z-index:6; display:none; min-width:220px;';
    this.debugPanel.innerHTML = `
      <div style="font-size:12px; color:#38bdf8; margin-bottom:4px;">Debug</div>
      <div id="dbgPos" style="font-size:12px;">Pos: 0,0,0</div>
      <div id="dbgVel" style="font-size:12px;">Vel: 0</div>
      <div id="dbgRPM" style="font-size:12px;">RPM: 0|0|0|0</div>
      <div id="dbgThrust" style="font-size:12px;">Thrust: 0</div>
    `;
    this.container.appendChild(this.debugPanel);
  }

  _bindInputs() {
    window.addEventListener('keydown', (e) => {
      if (e.key === 'd' || e.key === 'D') {
        this.debugEnabled = !this.debugEnabled;
        if (this.debugPanel) this.debugPanel.style.display = this.debugEnabled ? 'block' : 'none';
      }
      if (e.key === 'c' || e.key === 'C') {
        this.cameraMode = this.cameraMode === 'chase' ? 'overhead' : 'chase';
      }
    });
  }

  restart() {
    this.drone.reset({ position: new THREE.Vector3(0, 1.6, 0), velocity: new THREE.Vector3() });
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
    const target = new THREE.Vector3(0, 1.6, 0);

    const avoidance = this._avoidanceFromObstacles();
    const posErr = target.clone().add(avoidance).sub(this.drone.state.position);
    const velErr = new THREE.Vector3().sub(this.drone.state.velocity);
    const desiredAcc = posErr.multiplyScalar(3.2).add(velErr.multiplyScalar(2.2));
    desiredAcc.clampLength(0, 5.2);

    const desiredYaw = 0;
    const thrustVector = desiredAcc.clone().add(new THREE.Vector3(0, 9.81, 0)).multiplyScalar(params.mass);

    const desiredOrientation = this._desiredOrientationFromThrust(thrustVector, desiredYaw);
    const qError = desiredOrientation.clone().multiply(this.drone.state.orientation.clone().invert());
    const axis = new THREE.Vector3(qError.x, qError.y, qError.z);
    const angle = 2 * Math.atan2(axis.length(), qError.w);
    const attError = axis.length() > 1e-6 ? axis.normalize().multiplyScalar(angle) : new THREE.Vector3();
    const torque = attError.multiplyScalar(4.0).add(this.drone.state.angularVelocity.clone().multiplyScalar(-0.25));

    const thrustMag = clamp(thrustVector.length(), 0, params.mass * 28);
    const kYaw = params.kM / Math.max(params.kF, 1e-9);
    const L = params.armLength;
    const m0 = 0.25 * thrustMag - torque.y / (2 * L) + torque.z / (4 * kYaw);
    const m1 = 0.25 * thrustMag + torque.x / (2 * L) - torque.z / (4 * kYaw);
    const m2 = 0.25 * thrustMag + torque.y / (2 * L) + torque.z / (4 * kYaw);
    const m3 = 0.25 * thrustMag - torque.x / (2 * L) - torque.z / (4 * kYaw);
    const thrusts = [m0, m1, m2, m3].map((m) => clamp(m, 0, thrustMag));
    const rpms = thrusts.map((t) => Math.sqrt(Math.max(t, 0) / Math.max(params.kF, 1e-9)));
    return rpms.map((r) => clamp(r, params.minRPM || 0, params.maxRPM || params.rpmMax || 25000));
  }

  _avoidanceFromObstacles() {
    const repulse = new THREE.Vector3();
    this.manager.obstacles.forEach((o) => {
      const offset = this.drone.state.position.clone().sub(o.mesh.position);
      const dist = Math.max(offset.length() - o.radius, 0.01);
      if (dist > 4) return;
      const strength = Math.max(0, 1.5 - dist);
      if (strength <= 0) return;
      repulse.add(offset.normalize().multiplyScalar(strength * 4.2));
    });
    return repulse.clampLength(0, 3.2);
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
    if (this.cameraMode === 'chase') {
      const forward = new THREE.Vector3(0, 0, -1).applyQuaternion(this.drone.state.orientation);
      const desiredPos = this.drone.state.position
        .clone()
        .add(forward.clone().multiplyScalar(-5.5))
        .add(new THREE.Vector3(0, 2.4, 0));
      desiredPos.y = Math.max(desiredPos.y, this.floorHeight + 0.6);
      this.chaseCamera.position.lerp(desiredPos, 0.08);
      this.chaseCamera.lookAt(this.drone.state.position.clone().add(new THREE.Vector3(0, 0.7, 0)));
      this.activeCamera = this.chaseCamera;
    } else {
      this.overheadCamera.lookAt(new THREE.Vector3(0, 0, 0));
      this.activeCamera = this.overheadCamera;
    }

    if (this.controls) this.controls.camera = this.activeCamera;

    this.droneMesh.position.copy(this.drone.state.position);
    this.droneMesh.quaternion.copy(this.drone.state.orientation);

    this._updateHUD();
    this.renderer.render(this.scene, this.activeCamera || this.chaseCamera);
  }

  _updateHUD() {
    if (!this.overlay) return;
    this.overlay.querySelector('#obsAlt').textContent = `Alt: ${this.drone.state.position.y.toFixed(2)} m`;
    this.overlay.querySelector('#obsState').textContent = `Avoiding ${this.manager.obstacles.length} obstacles`;
    if (this.debugEnabled && this.debugPanel) {
      const rpm = this.drone.state.motorRPM || [0, 0, 0, 0];
      const vel = this.drone.state.velocity.length();
      this.debugPanel.querySelector('#dbgPos').textContent =
        `Pos: ${this.drone.state.position.x.toFixed(2)}, ${this.drone.state.position.y.toFixed(2)}, ${this.drone.state.position.z.toFixed(2)}`;
      this.debugPanel.querySelector('#dbgVel').textContent = `Vel: ${vel.toFixed(2)} m/s`;
      this.debugPanel.querySelector('#dbgRPM').textContent = `RPM: ${rpm.map((r) => r.toFixed(0)).join(' | ')}`;
      this.debugPanel.querySelector('#dbgThrust').textContent = `Thrust: ${this.drone.state.totalThrust.toFixed(2)} N`;
    }
  }
}
