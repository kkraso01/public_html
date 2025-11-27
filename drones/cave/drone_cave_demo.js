import { DronePhysicsEngine } from '../physics/drone_physics_engine.js';
import { CaveSim } from './cave_sim.js';
import { Lidar } from './lidar.js';
import { OccupancyGrid, updateGridFromLidar, chooseFrontierTarget, planPath } from './mapping.js';

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
    this.floorHeight = options.floorHeight ?? 0.05;
    this.physicsRate = 200;
    this.timeScale = 1;
    this.userPaused = false;
    this.visibilityPaused = false;
    this.lastFrame = null;
    this.accumulator = 0;
    this._frameReq = null;
    this.debugEnabled = false;
    this.cameraMode = 'overhead';

    this._initScene();
    this._initSim();
    this._initDrone();
    this._initHUD();
    this._bindInputs();
    this.restart();
  }

  _initScene() {
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x04070e);
    const aspect = this.options.width / this.options.height;
    this.chaseCamera = new THREE.PerspectiveCamera(60, aspect, 0.05, 200);
    this.chaseCamera.position.set(4, 4, 10);
    this.overheadCamera = new THREE.PerspectiveCamera(60, aspect, 0.05, 200);
    this.overheadCamera.position.set(0, 26, 0);
    this.overheadCamera.up.set(0, 0, -1);
    this.overheadCamera.lookAt(new THREE.Vector3(0, 0, 0));
    this.activeCamera = this.cameraMode === 'overhead' ? this.overheadCamera : this.chaseCamera;

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

    const floorGeo = new THREE.PlaneGeometry(200, 200);
    const floorMat = new THREE.MeshStandardMaterial({ color: 0x20252b, roughness: 0.8, metalness: 0.1 });
    const floor = new THREE.Mesh(floorGeo, floorMat);
    floor.rotation.x = -Math.PI / 2;
    floor.position.y = this.floorHeight;
    floor.receiveShadow = true;
    this.scene.add(floor);
    const wallMat = new THREE.MeshStandardMaterial({ color: 0x0b1220, roughness: 0.85, metalness: 0.05 });
    const wallBox = new THREE.BoxGeometry(60, 8, 0.6);
    const wallNorth = new THREE.Mesh(wallBox, wallMat);
    wallNorth.position.set(0, 4, -30);
    const wallSouth = wallNorth.clone();
    wallSouth.position.set(0, 4, 30);
    const wallEast = new THREE.Mesh(new THREE.BoxGeometry(0.6, 8, 60), wallMat);
    wallEast.position.set(30, 4, 0);
    const wallWest = wallEast.clone();
    wallWest.position.set(-30, 4, 0);
    [wallNorth, wallSouth, wallEast, wallWest].forEach((w) => {
      w.receiveShadow = true;
      w.castShadow = true;
      this.scene.add(w);
    });
  }

  _initSim() {
    this.cave = new CaveSim(this.scene);
    this.grid = new OccupancyGrid();
    this.lidar = new Lidar();
  }

  _initDrone() {
    this.drone = new DronePhysicsEngine({ floorHeight: this.floorHeight });
    this.drone.reset({ position: new THREE.Vector3(0, 1.4, 4) });

    const body = new THREE.Mesh(
      new THREE.CapsuleGeometry(0.32, 1.05, 12, 18),
      new THREE.MeshStandardMaterial({ color: 0x22c55e, metalness: 0.3, roughness: 0.4 }),
    );
    body.castShadow = true;
    const ring = new THREE.Mesh(
      new THREE.TorusGeometry(0.45, 0.06, 10, 32),
      new THREE.MeshStandardMaterial({ color: 0x0ea5e9, emissive: 0x0284c7, roughness: 0.35 }),
    );
    ring.rotation.x = Math.PI / 2;
    const arms = new THREE.Mesh(
      new THREE.BoxGeometry(2.0, 0.12, 0.14),
      new THREE.MeshStandardMaterial({ color: 0x38bdf8, emissive: 0x0ea5e9 }),
    );
    arms.castShadow = true;
    const props = new THREE.Mesh(
      new THREE.RingGeometry(0.18, 0.28, 18),
      new THREE.MeshBasicMaterial({ color: 0xe0f2fe }),
    );
    props.rotation.x = Math.PI / 2;
    this.droneMesh = new THREE.Group();
    this.droneMesh.add(body, ring, arms);
    [-1, 1].forEach((x) => {
      [-1, 1].forEach((z) => {
        const p = props.clone();
        p.position.set(x * 1.0, 0.6, z * 1.0);
        this.droneMesh.add(p);
      });
    });
    this.droneMesh.position.y = 0.8;
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
      <div id="caveFrontiers" style="font-size:12px;">Frontiers: 0</div>
      <div id="caveTarget" style="font-size:12px;">Target: none</div>
    `;
    this.container.style.position = 'relative';
    this.container.appendChild(this.overlay);

    this.cameraLabel = document.createElement('div');
    this.cameraLabel.style.cssText = 'font-size:12px; color:#cbd5f5; margin-top:6px;';
    this.cameraLabel.textContent = `Camera: ${this.cameraMode.toUpperCase()}`;
    this.overlay.appendChild(this.cameraLabel);

    this.debugPanel = document.createElement('div');
    this.debugPanel.style.cssText =
      'position:absolute; top:8px; right:8px; background:rgba(6,8,18,0.85); color:#e0f2fe; padding:10px; font-family:"Fira Code", monospace; border:1px solid rgba(34,211,238,0.35); border-radius:8px; z-index:6; display:none; min-width:220px;';
    this.debugPanel.innerHTML = `
      <div style="font-size:12px; color:#67e8f9; margin-bottom:4px;">Debug</div>
      <div id="dbgPos" style="font-size:12px;">Pos: 0, 0, 0</div>
      <div id="dbgAlt" style="font-size:12px;">Alt: 0.00 m</div>
      <div id="dbgVel" style="font-size:12px;">Vel: 0.00 m/s (0,0,0)</div>
      <div id="dbgRPM" style="font-size:12px;">RPM: 0 | 0 | 0 | 0</div>
      <div id="dbgThrust" style="font-size:12px;">Thrust: 0.0 N</div>
      <div id="dbgAtt" style="font-size:12px;">R/P/Y: 0 / 0 / 0</div>
      <div id="dbgAttErr" style="font-size:12px;">Att Err: 0.00</div>
      <div id="dbgFrontier" style="font-size:12px;">Frontiers: 0</div>
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
        if (this.cameraMode === 'chase') {
          this.cameraMode = 'overhead';
          this.activeCamera = this.overheadCamera;
        } else {
          this.cameraMode = 'chase';
          this.activeCamera = this.chaseCamera;
        }
        if (this.cameraLabel) this.cameraLabel.textContent = `Camera: ${this.cameraMode.toUpperCase()}`;
      }
    });
  }

  restart() {
    this.drone.reset({ position: new THREE.Vector3(0, 1.4, 4), orientation: new THREE.Quaternion() });
    this.grid = new OccupancyGrid();
    this.target = new THREE.Vector3(0, 1.4, 0);
    this.path = [];
    this.pathIndex = 0;
    this.lastHits = [];
    this.frontierInfo = null;
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
      this.lastHits = this.lidar.scan(this.cave, this.drone.state);
      updateGridFromLidar(this.grid, this.drone.state, this.lastHits);
      const coverage = this._coverage();
      this.overlay.querySelector('#caveCoverage').textContent = `Coverage: ${(coverage * 100).toFixed(1)}%`;
      this.frontierInfo = chooseFrontierTarget(this.grid, this.drone.state.position);
      if (this.frontierInfo?.point) {
        const newPath = planPath(this.grid, this.drone.state.position, this.frontierInfo.point);
        if (newPath.length) {
          this.path = newPath;
          this.pathIndex = 0;
        }
      }
      if (this.frontierInfo) {
        this.overlay.querySelector('#caveFrontiers').textContent = `Frontiers: ${this.frontierInfo.clusterCount}`;
        this.overlay.querySelector('#caveTarget').textContent =
          `Target: (${this.frontierInfo.point.x.toFixed(1)}, ${this.frontierInfo.point.z.toFixed(1)})`;
      }
    }

    const waypoint = this.path[this.pathIndex] || this.frontierInfo?.point || this.target;
    if (waypoint) {
      this.target.copy(waypoint);
      if (this.drone.state.position.distanceTo(waypoint) < 0.7 && this.pathIndex < this.path.length - 1) {
        this.pathIndex += 1;
      }
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
    targetPos.y = Math.max(1.4, this.target.y || 1.4);

    const posErr = targetPos.clone().sub(this.drone.state.position);
    const velErr = new THREE.Vector3().sub(this.drone.state.velocity);
    const avoidance = this._avoidanceFromHits();
    const desiredAcc = posErr.multiplyScalar(kp).add(velErr.multiplyScalar(kd)).add(avoidance);
    desiredAcc.clampLength(0, 3.2);

    const desiredYaw = Math.atan2(posErr.x, posErr.z);
    const thrustVector = desiredAcc.clone().add(new THREE.Vector3(0, 9.81, 0)).multiplyScalar(params.mass);

    const desiredOrientation = this._desiredOrientationFromThrust(thrustVector, desiredYaw);
    this.drone.desiredOrientationQuat.copy(desiredOrientation);
    const kp_att = this.drone.params.kp_att;
    const kd_att = this.drone.params.kd_att;
    const currentQuat = this.drone.state.orientationQuat || this.drone.state.orientation;
    const qError = desiredOrientation.clone().multiply(currentQuat.clone().invert());
    const errorAxis = new THREE.Vector3(qError.x, qError.y, qError.z).multiplyScalar(2.0);
    const torque = errorAxis.multiplyScalar(kp_att).add(this.drone.state.angularVelocity.clone().multiplyScalar(-kd_att));

    const thrustMag = clamp(thrustVector.length(), 0, params.mass * 20);
    const kYaw = params.kM / Math.max(params.kF, 1e-9);
    const L = params.armLength;
    const m0 = 0.25 * thrustMag - torque.y / (2 * L) + torque.z / (4 * kYaw);
    const m1 = 0.25 * thrustMag + torque.x / (2 * L) - torque.z / (4 * kYaw);
    const m2 = 0.25 * thrustMag + torque.y / (2 * L) + torque.z / (4 * kYaw);
    const m3 = 0.25 * thrustMag - torque.x / (2 * L) - torque.z / (4 * kYaw);
    const thrusts = [m0, m1, m2, m3].map((m) => clamp(m, 0, thrustMag));
    const rpms = thrusts.map((t) => Math.sqrt(Math.max(t, 0) / Math.max(params.kF, 1e-9)));
    return rpms.map((r) => {
      const clamped = clamp(r, params.minRPM, params.maxRPM);
      return THREE.MathUtils.clamp((clamped - params.minRPM) / (params.maxRPM - params.minRPM), 0, 1);
    });
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
        .add(forward.clone().multiplyScalar(-6.5))
        .add(new THREE.Vector3(0, 3.0, 0));
      desiredPos.y = Math.max(desiredPos.y, this.floorHeight + 0.8);
      this.chaseCamera.position.lerp(desiredPos, 0.08);
      this.chaseCamera.lookAt(this.drone.state.position.clone().add(new THREE.Vector3(0, 0.8, 0)));
      this.activeCamera = this.chaseCamera;
    } else {
      this.overheadCamera.position.y = 26;
      this.overheadCamera.lookAt(new THREE.Vector3(0, 0, 0));
      this.activeCamera = this.overheadCamera;
    }
    this.droneMesh.position.copy(this.drone.state.position);
    this.droneMesh.quaternion.copy(this.drone.state.orientation);
    this._updateDebugPanel();
    this.renderer.render(this.scene, this.activeCamera || this.overheadCamera);
  }

  _updateDebugPanel() {
    if (!this.debugEnabled || !this.debugPanel) return;
    const st = this.drone.state;
    const velMag = st.velocity.length();
    const rpm = st.motorRPM || [0, 0, 0, 0];
    const euler = new THREE.Euler().setFromQuaternion(st.orientationQuat || st.orientation);
    const desired = this.drone.desiredOrientationQuat || new THREE.Quaternion();
    const qErr = desired.clone().multiply((st.orientationQuat || st.orientation).clone().invert());
    const attErrMag = 2 * new THREE.Vector3(qErr.x, qErr.y, qErr.z).length();

    this.debugPanel.querySelector('#dbgPos').textContent =
      `Pos: ${st.position.x.toFixed(2)}, ${st.position.y.toFixed(2)}, ${st.position.z.toFixed(2)}`;
    this.debugPanel.querySelector('#dbgAlt').textContent = `Alt: ${st.position.y.toFixed(2)} m`;
    this.debugPanel.querySelector('#dbgVel').textContent =
      `Vel: ${velMag.toFixed(2)} m/s (${st.velocity.x.toFixed(2)}, ${st.velocity.y.toFixed(2)}, ${st.velocity.z.toFixed(2)})`;
    this.debugPanel.querySelector('#dbgRPM').textContent = `RPM: ${rpm.map((r) => r.toFixed(0)).join(' | ')}`;
    this.debugPanel.querySelector('#dbgThrust').textContent = `Thrust: ${st.totalThrust.toFixed(2)} N`;
    this.debugPanel.querySelector('#dbgAtt').textContent =
      `R/P/Y: ${(THREE.MathUtils.radToDeg(euler.x)).toFixed(1)} / ${(THREE.MathUtils.radToDeg(euler.y)).toFixed(1)} / ${(THREE.MathUtils.radToDeg(euler.z)).toFixed(1)}`;
    this.debugPanel.querySelector('#dbgAttErr').textContent = `Att Err: ${attErrMag.toFixed(3)}`;
    if (this.frontierInfo) {
      this.debugPanel.querySelector('#dbgFrontier').textContent =
        `Frontiers: ${this.frontierInfo.clusterCount} | Cluster size ${this.frontierInfo.targetSize}`;
    }
  }

  _avoidanceFromHits() {
    if (!this.lastHits || !this.lastHits.length) return new THREE.Vector3();
    return this.lastHits.reduce((acc, h) => {
      const strength = Math.max(0, 1 - h.distance / this.lidar.maxRange);
      return acc.add(h.direction.clone().multiplyScalar(-strength * 3.5));
    }, new THREE.Vector3()).clampLength(0, 2.5);
  }
}
