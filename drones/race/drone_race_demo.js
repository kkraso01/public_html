import { DronePhysicsEngine } from '../physics/drone_physics_engine.js';
import { buildRaceTrack, ReferenceTrajectory } from './track.js';
import { createRaceHUD, updateHUD } from './ui.js';

function clamp(x, min, max) {
  return Math.min(Math.max(x, min), max);
}

export function initDroneRaceDemo(container, options = {}) {
  if (!container || typeof THREE === 'undefined') {
    console.warn('Drone race demo requires a valid container and Three.js');
    return { pause() {}, resume() {}, restart() {}, setPausedFromVisibility() {}, destroy() {} };
  }
  const demo = new DroneRaceDemo(container, options);
  demo.start();
  return {
    pause: () => demo.pause(true),
    resume: () => demo.resume(true),
    restart: () => demo.restart(),
    setPausedFromVisibility: (visible) => demo.setPausedFromVisibility(visible),
    destroy: () => demo.destroy(),
  };
}

class DroneRaceDemo {
  constructor(container, options) {
    this.container = container;
    this.options = Object.assign(
      {
        highQuality: true,
        shadows: true,
        width: container.clientWidth || 800,
        height: container.clientHeight || 450,
      },
      options,
    );

    this.physicsRate = 240;
    this.timeScale = 1.0;
    this.state = 'COUNTDOWN';
    this.userPaused = false;
    this.visibilityPaused = false;
    this.lastFrame = null;
    this.accumulator = 0;
    this._frameReq = null;
    this.debugEnabled = false;
    this.cameraMode = 'chase';

    this._initScene();
    this._initDrone();
    this._initTrack();
    this._initHUD();
    this._bindInputs();
    this.restart();
  }

  _initScene() {
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x0a1020);
    const aspect = this.options.width / this.options.height;
    this.chaseCamera = new THREE.PerspectiveCamera(60, aspect, 0.1, 500);
    this.chaseCamera.position.set(0, 2.5, 8);
    this.overheadCamera = new THREE.PerspectiveCamera(60, aspect, 0.1, 500);
    this.overheadCamera.position.set(0, 25, 0);
    this.overheadCamera.up.set(0, 0, -1);
    this.overheadCamera.lookAt(new THREE.Vector3(0, 0, 0));
    this.activeCamera = this.cameraMode === 'overhead' ? this.overheadCamera : this.chaseCamera;

    this.renderer = new THREE.WebGLRenderer({ antialias: !!this.options.highQuality });
    this.renderer.setSize(this.options.width, this.options.height);
    this.renderer.setPixelRatio(this.options.highQuality ? window.devicePixelRatio : 1);
    this.renderer.shadowMap.enabled = !!this.options.shadows;
    this.container.innerHTML = '';
    this.container.appendChild(this.renderer.domElement);

    const ambient = new THREE.AmbientLight(0x8899ff, 0.35);
    const hemi = new THREE.HemisphereLight(0x6272ff, 0x090b14, 0.65);
    const dir = new THREE.DirectionalLight(0xffffff, 1.1);
    dir.position.set(6, 10, 6);
    dir.castShadow = !!this.options.shadows;
    this.scene.add(ambient, hemi, dir);

    const floorGeo = new THREE.PlaneGeometry(200, 200);
    const floorMat = new THREE.MeshStandardMaterial({ color: 0x20252b, roughness: 0.8, metalness: 0.1 });
    const floor = new THREE.Mesh(floorGeo, floorMat);
    floor.rotation.x = -Math.PI / 2;
    floor.position.y = 0;
    floor.receiveShadow = true;
    this.scene.add(floor);

    this.trailGeometry = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3()]);
    this.trailLine = new THREE.Line(
      this.trailGeometry,
      new THREE.LineBasicMaterial({ color: 0x8be9fd, transparent: true, opacity: 0.55 }),
    );
    this.scene.add(this.trailLine);
  }

  _initDrone() {
    this.drone = new DronePhysicsEngine();
    const body = new THREE.Mesh(
      new THREE.CylinderGeometry(0.12, 0.12, 0.04, 12),
      new THREE.MeshStandardMaterial({ color: 0x7dd3fc, emissive: 0x1f2937, metalness: 0.4, roughness: 0.3 }),
    );
    body.rotation.z = Math.PI / 2;
    body.castShadow = true;
    body.receiveShadow = true;

    const arms = new THREE.Mesh(
      new THREE.BoxGeometry(0.35, 0.02, 0.02),
      new THREE.MeshStandardMaterial({ color: 0x93c5fd, emissive: 0x0ea5e9 }),
    );
    arms.castShadow = true;

    this.droneMesh = new THREE.Group();
    this.droneMesh.add(body, arms);

    const prop = new THREE.Mesh(
      new THREE.RingGeometry(0.06, 0.08, 16),
      new THREE.MeshBasicMaterial({ color: 0xf8fafc }),
    );
    prop.rotation.x = Math.PI / 2;
    [-0.17, 0.17].forEach((x) => {
      [-0.17, 0.17].forEach((y) => {
        const p = prop.clone();
        p.position.set(x, 0.01, y);
        this.droneMesh.add(p);
      });
    });

    this.scene.add(this.droneMesh);
  }

  _initTrack() {
    const { gates, waypoints } = buildRaceTrack(this.scene);
    this.gates = gates;
    this.waypoints = waypoints;
    this.trajectory = new ReferenceTrajectory(waypoints, 22);
  }

  _initHUD() {
    this.overlay = createRaceHUD(this.container);
    this.cameraLabel = document.createElement('div');
    this.cameraLabel.style.cssText = 'font-size:12px; color:#cbd5f5; margin-top:4px;';
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
      }
    });
  }

  restart() {
    this.drone.reset({ position: new THREE.Vector3(0, 1.6, 0), orientation: new THREE.Quaternion() });
    this.simTime = 0;
    this.gateIndex = 0;
    this.state = 'RUNNING';
    this.trailGeometry.setFromPoints([this.drone.state.position.clone(), this.drone.state.position.clone()]);
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
    if (this.state !== 'RUNNING') return;
    this.simTime += dt;
    const target = this.trajectory.sample(this.simTime % this.trajectory.totalTime);
    const commands = this._computeMotorCommands(target);
    this.drone.applyMotorCommands(commands[0], commands[1], commands[2], commands[3]);
    this.drone.step(dt);

    const pos = this.drone.state.position;
    const vel = this.drone.state.velocity.length();
    updateHUD(this.overlay, {
      state: this.state,
      time: this.simTime,
      gateIdx: this.gateIndex,
      gateTotal: this.gates.length,
      speed: vel,
    });

    const gatePos = this.waypoints[this.gateIndex + 1] || this.waypoints[this.waypoints.length - 1];
    if (pos.distanceTo(gatePos) < 1.5 && this.gateIndex < this.gates.length) {
      this.gateIndex += 1;
      if (this.gateIndex >= this.gates.length) this.state = 'COMPLETE';
    }
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

  _computeMotorCommands(target) {
    const kp = 4.5;
    const kd = 3.0;
    const params = this.drone.params;

    const posErr = target.position.clone().sub(this.drone.state.position);
    const velErr = target.velocity.clone().sub(this.drone.state.velocity);
    const desiredAcc = target.acceleration
      .clone()
      .add(posErr.multiplyScalar(kp))
      .add(velErr.multiplyScalar(kd));

    const thrustVector = desiredAcc.clone().add(new THREE.Vector3(0, 9.81, 0)).multiplyScalar(params.mass);
    const thrustMag = clamp(thrustVector.length(), 0, params.mass * 30);

    const desiredOrientation = this._desiredOrientationFromThrust(thrustVector, target.yaw);
    this.drone.desiredOrientationQuat.copy(desiredOrientation);

    const kp_att = params.kp_att;
    const kd_att = params.kd_att;

    const currentQuat = this.drone.state.orientationQuat || this.drone.state.orientation;
    const qError = desiredOrientation.clone().multiply(currentQuat.clone().invert());
    const errorAxis = new THREE.Vector3(qError.x, qError.y, qError.z).multiplyScalar(2.0);

    const torque = errorAxis.multiplyScalar(kp_att).add(this.drone.state.angularVelocity.clone().multiplyScalar(-kd_att));

    const kYaw = params.kM / Math.max(params.kF, 1e-9);
    const L = params.armLength;
    const m0 = 0.25 * thrustMag - torque.y / (2 * L) + torque.z / (4 * kYaw);
    const m1 = 0.25 * thrustMag + torque.x / (2 * L) - torque.z / (4 * kYaw);
    const m2 = 0.25 * thrustMag + torque.y / (2 * L) + torque.z / (4 * kYaw);
    const m3 = 0.25 * thrustMag - torque.x / (2 * L) - torque.z / (4 * kYaw);

    const thrusts = [m0, m1, m2, m3].map((m) => clamp(m, 0, thrustMag));
    const rpms = thrusts.map((t) => Math.sqrt(Math.max(t, 0) / Math.max(params.kF, 1e-9)));
    const commands = rpms.map((rpm) => {
      const clamped = clamp(rpm, params.minRPM, params.maxRPM);
      return THREE.MathUtils.clamp((clamped - params.minRPM) / (params.maxRPM - params.minRPM), 0, 1);
    });

    return commands;
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

    const points = this.trailGeometry.getAttribute('position');
    points.setXYZ(0, this.drone.state.position.x, this.drone.state.position.y, this.drone.state.position.z);
    points.setXYZ(1, this.drone.state.position.x, this.drone.state.position.y, this.drone.state.position.z);
    points.needsUpdate = true;

    this._updateDebugPanel();
    this.renderer.render(this.scene, this.activeCamera || this.chaseCamera);
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
