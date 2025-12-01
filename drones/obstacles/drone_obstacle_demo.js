import { DronePhysicsEngine } from '../physics/drone_physics_engine.js';
import { CRAZYFLIE_PARAMS } from '../physics/params_crazyflie.js';
import { StationaryController } from '../stationary/stationary_controller.js';
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
    this.floorHeight = options.floorHeight ?? 0.05;
    this.physicsRate = 200;
    this.userPaused = false;
    this.visibilityPaused = false;
    this.lastFrame = null;
    this.accumulator = 0;
    this._frameReq = null;
    this.debugEnabled = false;
    this.cameraMode = 'overhead';

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
    this.chaseCamera = new THREE.PerspectiveCamera(60, aspect, 0.1, 80);
    this.chaseCamera.position.set(4, 3, 6);
    this.overheadCamera = new THREE.PerspectiveCamera(60, aspect, 0.1, 150);
    this.overheadCamera.position.set(0, 25, 0);
    this.overheadCamera.up.set(0, 0, -1);
    this.overheadCamera.lookAt(new THREE.Vector3(0, 0, 0));
    this.activeCamera = this.cameraMode === 'overhead' ? this.overheadCamera : this.chaseCamera;

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

    const floorGeo = new THREE.PlaneGeometry(200, 200);
    const floorMat = new THREE.MeshStandardMaterial({ color: 0x20252b, roughness: 0.8, metalness: 0.1 });
    const floor = new THREE.Mesh(floorGeo, floorMat);
    floor.rotation.x = -Math.PI / 2;
    floor.position.y = this.floorHeight;
    floor.receiveShadow = true;
    this.scene.add(floor);
  }

  _initDrone() {
    this.params = CRAZYFLIE_PARAMS;
    this.drone = new DronePhysicsEngine(this.params);
    this.drone.reset({ position: new THREE.Vector3(0, 1.2, 0) });

    // Initialize ETH controller
    this.controller = new StationaryController(this.params);
    this.controller.updateGains({
      kp: { x: 4.0, y: 4.0, z: 8.0 },
      kd: { x: 3.0, y: 3.0, z: 5.0 },
      ki: { x: 0.08, y: 0.08, z: 0.12 },
    });
    this.controller.updateAttitudeGains({
      kR: { x: 1.0, y: 1.0, z: 5.0 },
      kOmega: { x: 0.2, y: 0.2, z: 0.3 },
    });
    this.controller.eth.maxAcc = 15.0;
    this.controller.eth.maxTiltAngle = 60 * Math.PI / 180;

    // Create body matching stationary demo
    const bodyGeometry = new THREE.BoxGeometry(0.28, 0.28, 0.08);
    const topMat = new THREE.MeshStandardMaterial({ color: 0xff0000, metalness: 0.35, roughness: 0.45 });
    const bottomMat = new THREE.MeshStandardMaterial({ color: 0x0000ff, metalness: 0.35, roughness: 0.45 });
    const sideMat = new THREE.MeshStandardMaterial({ color: 0x22d3ee, metalness: 0.35, roughness: 0.45 });
    const bodyMaterials = [sideMat, sideMat, topMat, bottomMat, sideMat, sideMat];
    const body = new THREE.Mesh(bodyGeometry, bodyMaterials);
    body.castShadow = true;
    body.receiveShadow = true;
    
    const armMat = new THREE.MeshStandardMaterial({ color: 0x0ea5e9, metalness: 0.25, roughness: 0.6 });
    const arm = new THREE.Mesh(new THREE.BoxGeometry(0.8, 0.12, 0.05), armMat);
    arm.castShadow = true;
    arm.receiveShadow = true;

    this.droneMesh = new THREE.Group();
    this.droneMesh.add(body, arm);

    // Add colored motor rotors
    const rotorGeo = new THREE.RingGeometry(0.09, 0.14, 16);
    const rotorColors = [0xff0000, 0x00ff00, 0x0000ff, 0xffff00];
    const rotorPositions = [
      new THREE.Vector3(0.28, 0.28, 0.06),
      new THREE.Vector3(0.28, -0.28, 0.06),
      new THREE.Vector3(-0.28, -0.28, 0.06),
      new THREE.Vector3(-0.28, 0.28, 0.06),
    ];
    rotorPositions.forEach((p, i) => {
      const rotorMat = new THREE.MeshStandardMaterial({ color: rotorColors[i], emissive: rotorColors[i], emissiveIntensity: 0.5, metalness: 0.6, roughness: 0.4 });
      const rTop = new THREE.Mesh(rotorGeo, rotorMat);
      rTop.position.copy(p);
      rTop.position.z += 0.01;
      rTop.castShadow = true;
      this.droneMesh.add(rTop);
      const rBottom = new THREE.Mesh(rotorGeo, rotorMat);
      rBottom.position.copy(p);
      rBottom.position.z -= 0.01;
      rBottom.castShadow = true;
      this.droneMesh.add(rBottom);
    });

    this.scene.add(this.droneMesh);
  }

  _initObstacles() {
    this.manager = new ObstacleManager(this.scene);
    this.manager.addObstacle(new THREE.Vector3(1.5, 0.4, 0), 0.5, 0xf97316);
    this.manager.addObstacle(new THREE.Vector3(-1.2, 0.4, -1.5), 0.45, 0x22c55e);
    this.manager.addObstacle(new THREE.Vector3(0, 0.4, 1.8), 0.55, 0x10b981);
    this.controls = new DraggableControls(this.cameraMode === 'overhead' ? this.overheadCamera : this.chaseCamera, this.renderer.domElement, this.manager, {
      onDragEnd: () => {
        this._avoidanceVector = new THREE.Vector3();
      },
    });
  }

  _initHUD() {
    this.container.style.position = 'relative';
    this.overlay = document.createElement('div');
    this.overlay.style.cssText =
      'position:absolute; top:8px; left:8px; background:rgba(6,9,18,0.85); color:#e2e8f0; padding:10px; font-family:"Fira Code", monospace; border:1px solid rgba(125,211,252,0.4); border-radius:8px; z-index:5;';
    this.overlay.innerHTML = `
      <div style="font-size:12px; color:#7dd3fc;">Obstacle Arena</div>
      <div id="obsState" style="font-size:12px; margin-top:6px;">Hovering</div>
      <div id="obsInfo" style="font-size:12px;">C: Camera, D: Debug</div>
    `;
    this.container.appendChild(this.overlay);

    this.cameraLabel = document.createElement('div');
    this.cameraLabel.style.cssText = 'font-size:12px; color:#cbd5f5; margin-top:6px;';
    this.cameraLabel.textContent = `Camera: ${this.cameraMode.toUpperCase()}`;
    this.overlay.appendChild(this.cameraLabel);

    this.debugPanel = document.createElement('div');
    this.debugPanel.style.cssText =
      'position:absolute; top:8px; right:8px; background:rgba(6,8,18,0.85); color:#cbd5f5; padding:10px; font-family:"Fira Code", monospace; border:1px solid rgba(14,165,233,0.45); border-radius:8px; z-index:6; display:none; min-width:220px;';
    this.debugPanel.innerHTML = `
      <div style="font-size:12px; color:#7dd3fc; margin-bottom:4px;">Debug</div>
      <div id="dbgAlt" style="font-size:12px;">Alt: 0.00 m</div>
      <div id="dbgVel" style="font-size:12px;">Vel: 0.00 m/s (0,0,0)</div>
      <div id="dbgRPM" style="font-size:12px;">RPM: 0 | 0 | 0 | 0</div>
      <div id="dbgThrust" style="font-size:12px;">Thrust: 0.0 N</div>
      <div id="dbgAtt" style="font-size:12px;">R/P/Y: 0 / 0 / 0</div>
      <div id="dbgAttErr" style="font-size:12px;">Att Err: 0.00</div>
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
        this.controls.camera = this.activeCamera;
      }
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
    
    // Use ETH controller with obstacle avoidance
    const state = this.drone.getState();
    const target = new THREE.Vector3(0, 1.2, 0);
    
    const nearest = this._nearestObstacle();
    let avoidance = new THREE.Vector3();
    if (nearest) {
      const diff = state.position.clone().sub(nearest.mesh.position);
      const dist = Math.max(diff.length(), 0.01);
      const push = Math.max(0, 1.2 - dist);
      avoidance = diff.normalize().multiplyScalar(push * 2.0);
    }
    
    const targetObj = {
      position: target,
      velocity: new THREE.Vector3(0, 0, 0),
      acceleration: avoidance, // Use avoidance as feedforward
      yaw: 0,
    };
    
    const result = this.controller.eth.computeControl(state, targetObj, dt);
    this.drone.applyMotorCommands(result.motorCommands[0], result.motorCommands[1], result.motorCommands[2], result.motorCommands[3]);
    this.drone.step(dt);

    const postState = this.drone.getState();
    if (this.manager.collide(postState.position)) {
      this.drone.state.v_W.multiplyScalar(-0.3);
    }
  }

  _computeMotorCommands() {
    const params = this.drone.params;
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
    this.drone.desiredOrientationQuat.copy(desiredOrientation);
    const kp_att = params.kp_att;
    const kd_att = params.kd_att;
    const currentQuat = this.drone.state.orientationQuat || this.drone.state.orientation;
    const qError = desiredOrientation.clone().multiply(currentQuat.clone().invert());
    const errorAxis = new THREE.Vector3(qError.x, qError.y, qError.z).multiplyScalar(2.0);
    const torque = errorAxis.multiplyScalar(kp_att).add(this.drone.state.angularVelocity.clone().multiplyScalar(-kd_att));

    const thrustMag = clamp(thrustVector.length(), 0, params.mass * 25);
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
    if (this.cameraMode === 'chase') {
      const desiredPos = this.drone.state.position.clone().add(new THREE.Vector3(0, 1.5, 4));
      this.chaseCamera.position.lerp(desiredPos, 0.08);
      this.chaseCamera.lookAt(this.drone.state.position.clone().add(new THREE.Vector3(0, 0.2, 0)));
      this.activeCamera = this.chaseCamera;
    } else {
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

    this.debugPanel.querySelector('#dbgAlt').textContent = `Alt: ${st.position.y.toFixed(2)} m`;
    this.debugPanel.querySelector('#dbgVel').textContent =
      `Vel: ${velMag.toFixed(2)} m/s (${st.velocity.x.toFixed(2)}, ${st.velocity.y.toFixed(2)}, ${st.velocity.z.toFixed(2)})`;
    this.debugPanel.querySelector('#dbgRPM').textContent = `RPM: ${rpm.map((r) => r.toFixed(0)).join(' | ')}`;
    this.debugPanel.querySelector('#dbgThrust').textContent = `Thrust: ${st.totalThrust.toFixed(2)} N`;
    this.debugPanel.querySelector('#dbgAtt').textContent =
      `R/P/Y: ${(THREE.MathUtils.radToDeg(euler.x)).toFixed(1)} / ${(THREE.MathUtils.radToDeg(euler.y)).toFixed(1)} / ${(THREE.MathUtils.radToDeg(euler.z)).toFixed(1)}`;
    this.debugPanel.querySelector('#dbgAttErr').textContent = `Att Err: ${attErrMag.toFixed(3)}`;
  }
}
