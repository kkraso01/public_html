import { DronePhysicsEngine } from '../physics/drone_physics_engine.js';
import { CaveSim } from './cave_sim.js';
import { Lidar } from './lidar.js';
import { OccupancyGrid, updateGridFromLidar, chooseFrontierTarget } from './mapping.js';

function clamp(x, min, max) {
  return Math.min(Math.max(x, min), max);
}

export function initDroneCaveDemo(container, options = {}) {
  if (!container || typeof THREE === 'undefined') {
    console.warn('Drone cave demo requires a valid container and Three.js');
    return { pause() {}, resume() {}, restart() {}, setPausedFromVisibility() {}, destroy() {} };
  }
  const demo = new DroneCaveDemo(container, options);
  demo.start();
  return {
    pause: () => demo.pause(true),
    resume: () => demo.resume(true),
    restart: () => demo.restart(),
    setPausedFromVisibility: (visible) => demo.setPausedFromVisibility(visible),
    destroy: () => demo.destroy(),
  };
}

class DroneCaveDemo {
  constructor(container, options) {
    this.container = container;
    this.options = Object.assign(
      { width: container.clientWidth || 900, height: container.clientHeight || 520, shadows: true },
      options,
    );
    this.physicsRate = 200;
    this.timeScale = 1;
    this.userPaused = false;
    this.visibilityPaused = false;
    this.lastFrame = null;
    this.accumulator = 0;
    this._frameReq = null;

    this._initScene();
    this._initSim();
    this._initDrone();
    this._initHUD();
    this.restart();
  }

  _initScene() {
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x04070e);
    this.camera = new THREE.PerspectiveCamera(60, this.options.width / this.options.height, 0.1, 120);
    this.camera.position.set(4, 3, 8);

    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderer.setSize(this.options.width, this.options.height);
    this.renderer.setPixelRatio(devicePixelRatio || 1);
    this.renderer.shadowMap.enabled = true;
    this.container.innerHTML = '';
    this.container.appendChild(this.renderer.domElement);

    const ambient = new THREE.AmbientLight(0x64748b, 0.25);
    this.scene.add(ambient);
    const dir = new THREE.DirectionalLight(0xbcd7ff, 1.4);
    dir.position.set(6, 10, 6);
    dir.castShadow = true;
    this.scene.add(dir);
  }

  _initSim() {
    this.cave = new CaveSim(this.scene);
    this.grid = new OccupancyGrid();
    this.lidar = new Lidar();
  }

  _initDrone() {
    this.drone = new DronePhysicsEngine();
    this.drone.reset({ position: new THREE.Vector3(0, 1.2, 4) });

    const body = new THREE.Mesh(
      new THREE.CapsuleGeometry(0.18, 0.3, 6, 12),
      new THREE.MeshStandardMaterial({ color: 0x22c55e, metalness: 0.3, roughness: 0.4 }),
    );
    body.castShadow = true;
    const ring = new THREE.Mesh(
      new THREE.TorusGeometry(0.28, 0.04, 6, 20),
      new THREE.MeshStandardMaterial({ color: 0x0ea5e9, emissive: 0x0284c7, roughness: 0.35 }),
    );
    ring.rotation.x = Math.PI / 2;
    this.droneMesh = new THREE.Group();
    this.droneMesh.add(body);
    this.droneMesh.add(ring);
    this.scene.add(this.droneMesh);
  }

  _initHUD() {
    this.overlay = document.createElement('div');
    this.overlay.style.cssText =
      'position:absolute; top:8px; left:8px; background:rgba(6,9,18,0.85); color:#d6e3ff; padding:10px; font-family:"Fira Code", monospace; border:1px solid rgba(59,130,246,0.5); border-radius:8px; z-index:5;';
    this.overlay.innerHTML = `
      <div style="font-size:12px; color:#7dd3fc;">Cave Explorer</div>
      <div id="caveState" style="font-size:12px; margin-top:6px;">Exploring</div>
      <div id="caveCoverage" style="font-size:12px;">Coverage: 0%</div>
    `;
    this.container.style.position = 'relative';
    this.container.appendChild(this.overlay);
  }

  restart() {
    this.drone.reset({ position: new THREE.Vector3(0, 1.2, 4), orientation: new THREE.Quaternion() });
    this.grid = new OccupancyGrid();
    this.target = new THREE.Vector3(0, 1.2, 0);
    this.simTime = 0;
    this.state = 'EXPLORING';
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
    this.container.innerHTML = '';
  }

  _loop(timestamp) {
    if (this.userPaused || this.visibilityPaused) return;
    if (this.lastFrame === null) this.lastFrame = timestamp;
    const dt = ((timestamp - this.lastFrame) / 1000) * this.timeScale;
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

    if (this.simTime % 0.2 < dt) {
      const hits = this.lidar.scan(this.cave, this.drone.state);
      updateGridFromLidar(this.grid, this.drone.state, hits);
      const coverage = this._coverage();
      this.overlay.querySelector('#caveCoverage').textContent = `Coverage: ${(coverage * 100).toFixed(1)}%`;
      const frontier = chooseFrontierTarget(this.grid, this.drone.state.position);
      if (frontier) this.target = frontier;
    }

    const commands = this._computeMotorCommands();
    this.drone.applyMotorCommands(commands[0], commands[1], commands[2], commands[3]);
    this.drone.step(dt);

    if (this.cave.collide(this.drone.state.position)) {
      this.drone.state.velocity.multiplyScalar(-0.2);
      this.drone.state.position.add(new THREE.Vector3(0, 0.05, 0));
    }

    this.overlay.querySelector('#caveState').textContent = `Exploring → (${this.target.x.toFixed(1)}, ${this.target.z.toFixed(1)})`;
  }

  _coverage() {
    let known = 0;
    for (let i = 0; i < this.grid.grid.length; i++) {
      if (Math.abs(this.grid.grid[i]) > 0.01) known++;
    }
    return known / this.grid.grid.length;
  }

  _computeMotorCommands() {
    const params = this.drone.params;
    const kp = 2.8;
    const kd = 2.2;
    const kR = 3.0;
    const kOmega = 0.25;
    const targetPos = this.target.clone();
    targetPos.y = 1.4;

    const posErr = targetPos.clone().sub(this.drone.state.position);
    const velErr = new THREE.Vector3().sub(this.drone.state.velocity);
    const desiredAcc = posErr.multiplyScalar(kp).add(velErr.multiplyScalar(kd));
    desiredAcc.clampLength(0, 2.5);

    const desiredYaw = Math.atan2(posErr.x, posErr.z);
    const thrustVector = desiredAcc.clone().add(new THREE.Vector3(0, 9.81, 0)).multiplyScalar(params.mass);

    const desiredOrientation = this._desiredOrientationFromThrust(thrustVector, desiredYaw);
    const qError = desiredOrientation.clone().multiply(this.drone.state.orientation.clone().invert());
    const axis = new THREE.Vector3(qError.x, qError.y, qError.z);
    const angle = 2 * Math.atan2(axis.length(), qError.w);
    const attError = axis.length() > 1e-6 ? axis.normalize().multiplyScalar(angle) : new THREE.Vector3();
    const torque = attError.multiplyScalar(kR).add(this.drone.state.angularVelocity.clone().multiplyScalar(-kOmega));

    const thrustMag = clamp(thrustVector.length(), 0, params.mass * 20);
    const kYaw = params.kM / Math.max(params.kF, 1e-9);
    const L = params.armLength;
    const m0 = 0.25 * thrustMag - torque.y / (2 * L) + torque.z / (4 * kYaw);
    const m1 = 0.25 * thrustMag + torque.x / (2 * L) - torque.z / (4 * kYaw);
    const m2 = 0.25 * thrustMag + torque.y / (2 * L) + torque.z / (4 * kYaw);
    const m3 = 0.25 * thrustMag - torque.x / (2 * L) - torque.z / (4 * kYaw);
    const thrusts = [m0, m1, m2, m3].map((m) => clamp(m, 0, thrustMag));
    const rpms = thrusts.map((t) => Math.sqrt(Math.max(t, 0) / Math.max(params.kF, 1e-9)));
    return rpms.map((r) => clamp(r, 0, params.rpmMax));
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
    this.camera.lookAt(this.drone.state.position.clone().add(new THREE.Vector3(0, 0.2, 0)));
    this.droneMesh.position.copy(this.drone.state.position);
    this.droneMesh.quaternion.copy(this.drone.state.orientation);
    this.renderer.render(this.scene, this.camera);
  }
}
