import { DronePhysicsEngine } from '../physics/drone_physics_engine.js';
import { CRAZYFLIE_PARAMS } from '../physics/params_crazyflie.js';
import { StationaryController } from './stationary_controller.js';

export function initStationaryHoverDemo(container, options = {}) {
  if (!container || typeof THREE === 'undefined') {
    console.warn('Stationary demo requires a valid container and Three.js');
    return { pause() {}, resume() {}, restart() {}, setPausedFromVisibility() {}, destroy() {}, handleResize() {} };
  }
  const demo = new StationaryHoverDemo(container, options);
  demo.start();
  return {
    pause: () => demo.pause(true),
    resume: () => demo.resume(true),
    restart: () => demo.restart(),
    setPausedFromVisibility: (visible) => demo.setPausedFromVisibility(visible),
    destroy: () => demo.destroy(),
    handleResize: () => demo.handleResize(),
  };
}

class StationaryHoverDemo {
  constructor(container, options) {
    this.container = container;
    this.controlContainer = options.controlContainer;
    this.options = Object.assign({ width: container.clientWidth || 800, height: container.clientHeight || 480 }, options);
    this.physicsRate = 240;
    this.userPaused = false;
    this.visibilityPaused = false;
    this.lastFrame = null;
    this.accumulator = 0;
    this._frameReq = null;

    this.params = CRAZYFLIE_PARAMS;

    this.target = {
      position: new THREE.Vector3(0, 0.6, 0),
      velocity: new THREE.Vector3(0, 0, 0),
      yaw: 0,
    };

    this.controller = new StationaryController(this.params);

    this._initScene();
    this._initDrone();
    this._initOverlay();
    this.restart();
  }

  _initScene() {
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x0c111c);

    const aspect = this.options.width / this.options.height;
    this.camera = new THREE.PerspectiveCamera(60, aspect, 0.05, 200);
    this.camera.position.set(2, 2, 2);
    this.camera.lookAt(0, 0.6, 0);
    this.camera.up.set(0, 1, 0);

    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderer.setSize(this.options.width, this.options.height);
    this.renderer.setPixelRatio(window.devicePixelRatio || 1);
    this.renderer.shadowMap.enabled = true;
    this.container.innerHTML = '';
    this.container.appendChild(this.renderer.domElement);

    this.scene.add(new THREE.AmbientLight(0xffffff, 0.5));
    const key = new THREE.DirectionalLight(0xffffff, 1.3);
    key.position.set(5, 8, 5);
    key.castShadow = true;
    this.scene.add(key);

    const floor = new THREE.Mesh(
      new THREE.PlaneGeometry(50, 50),
      new THREE.MeshStandardMaterial({ color: 0x20252b }),
    );
    floor.rotation.x = -Math.PI / 2;
    floor.receiveShadow = true;
    this.scene.add(floor);

    const grid = new THREE.GridHelper(10, 20, 0x475569, 0x1f2937);
    this.scene.add(grid);
  }

  _initDrone() {
    this.drone = new DronePhysicsEngine(this.params);
    this.drone.reset({
      position: new THREE.Vector3(0, 0.2, 0),
      velocity: new THREE.Vector3(0, 0, 0),
      orientation: new THREE.Quaternion().set(0, 0, 0, 1),
      angularVelocity: new THREE.Vector3(0, 0, 0),
    });

    const body = new THREE.Mesh(
      new THREE.BoxGeometry(0.28, 0.08, 0.28),
      new THREE.MeshStandardMaterial({ color: 0x22d3ee, metalness: 0.35, roughness: 0.45 }),
    );
    body.castShadow = true;
    body.receiveShadow = true;
    const armMat = new THREE.MeshStandardMaterial({ color: 0x0ea5e9, metalness: 0.25, roughness: 0.6 });
    const arm = new THREE.Mesh(new THREE.BoxGeometry(0.8, 0.05, 0.12), armMat);
    arm.castShadow = true;
    arm.receiveShadow = true;

    this.droneMesh = new THREE.Group();
    this.droneMesh.add(body);
    this.droneMesh.add(arm);

    const rotorGeo = new THREE.RingGeometry(0.09, 0.14, 16);
    const rotorMat = new THREE.MeshStandardMaterial({ color: 0xf8fafc, emissive: 0x0ea5e9, emissiveIntensity: 0.2 });
    const rotorPositions = [
      new THREE.Vector3(0.4, 0.06, 0.4),
      new THREE.Vector3(-0.4, 0.06, 0.4),
      new THREE.Vector3(-0.4, 0.06, -0.4),
      new THREE.Vector3(0.4, 0.06, -0.4),
    ];
    rotorPositions.forEach((p) => {
      const r = new THREE.Mesh(rotorGeo, rotorMat);
      r.rotation.x = Math.PI / 2;
      r.position.copy(p);
      r.castShadow = true;
      this.droneMesh.add(r);
    });

    this.scene.add(this.droneMesh);
  }

  _initOverlay() {
    const container = this.controlContainer || this.container;
    container.style.position = container === this.container ? 'relative' : container.style.position || 'relative';
    this.overlay = document.createElement('div');
    this.overlay.style.cssText =
      'position:absolute; top:8px; left:8px; background:rgba(12,17,28,0.85); color:#e5e7eb; padding:12px; font-family:"Fira Code", monospace; border:1px solid rgba(34,211,238,0.45); border-radius:10px; z-index:5; max-width:320px; display:grid; gap:8px;';

    const title = document.createElement('div');
    title.textContent = 'Stationary Hover – ETH Cascaded Controller';
    title.style.cssText = 'color:#22d3ee; font-weight:700; font-size:13px; grid-column:1/-1;';
    this.overlay.appendChild(title);

    this.controls = {};
    const sliderDefs = [
      { key: 'kp', label: 'Kp (pos)', min: 0, max: 12, step: 0.1, value: this.controller.eth.Kp.x },
      { key: 'ki', label: 'Ki (pos)', min: 0, max: 6, step: 0.05, value: this.controller.eth.Ki.x },
      { key: 'kd', label: 'Kd (pos)', min: 0, max: 8, step: 0.05, value: this.controller.eth.Kd.x },
    ];

    sliderDefs.forEach((def) => {
      const wrapper = document.createElement('div');
      wrapper.style.cssText = 'display:flex; flex-direction:column; gap:4px; background:rgba(99,102,241,0.08); border:1px solid rgba(99,102,241,0.3); border-radius:6px; padding:8px;';
      const label = document.createElement('label');
      label.textContent = `${def.label}: ${def.value.toFixed(2)}`;
      label.style.cssText = 'font-size:12px; color:#cbd5e1; display:flex; justify-content:space-between;';
      const input = document.createElement('input');
      input.type = 'range';
      input.min = def.min;
      input.max = def.max;
      input.step = def.step;
      input.value = def.value;
      input.className = 'pid-slider';
      input.addEventListener('input', () => {
        const val = parseFloat(input.value);
        label.textContent = `${def.label}: ${val.toFixed(2)}`;
        this.controller.updateGains({ [def.key]: val });
      });
      wrapper.appendChild(label);
      wrapper.appendChild(input);
      this.overlay.appendChild(wrapper);
      this.controls[def.key] = input;
    });

    this.hud = document.createElement('div');
    this.hud.style.cssText = 'display:grid; grid-template-columns:1fr; gap:4px; font-size:12px; color:#cbd5f5;';
    this.hud.innerHTML = `
      <div id="hudMode">Mode: ETH cascaded</div>
      <div id="hudAlt">Altitude: 0.00 m</div>
      <div id="hudError">Position error: (0,0,0)</div>
      <div id="hudVel">Velocity: (0,0,0)</div>
    `;
    this.overlay.appendChild(this.hud);

    container.appendChild(this.overlay);
  }

  restart() {
    this.drone.reset({
      position: new THREE.Vector3(0, 0.2, 0),
      velocity: new THREE.Vector3(0, 0, 0),
      orientation: new THREE.Quaternion().set(0, 0, 0, 1),
      angularVelocity: new THREE.Vector3(0, 0, 0),
    });
    this.target.position.set(0, 0.6, 0);
    this.target.velocity.set(0, 0, 0);
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

  destroy() {
    this.pause(false);
    this.container.innerHTML = '';
  }

  start() {
    this.resume(false);
  }

  handleResize() {
    const width = this.container.clientWidth || this.options.width;
    const height = this.container.clientHeight || this.options.height;
    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(width, height);
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
    const state = this.drone.getState();
    const { motorCommands } = this.controller.compute(state, dt);
    this.drone.applyMotorCommands(motorCommands[0], motorCommands[1], motorCommands[2], motorCommands[3]);
    this.drone.step(dt);
  }

  _render() {
    const state = this.drone.getState();
    this.droneMesh.position.copy(state.position);
    this.droneMesh.quaternion.copy(state.orientationQuat);

    this._updateHUD(state);
    this.renderer.render(this.scene, this.camera);
  }

  _updateHUD(state) {
    if (!this.hud) return;
    const err = this.target.position.clone().sub(state.position);
    const vel = state.velocity;
    this.hud.querySelector('#hudMode').textContent = 'Mode: ETH cascaded';
    this.hud.querySelector('#hudAlt').textContent = `Altitude: ${state.position.y.toFixed(2)} m`;
    this.hud.querySelector('#hudError').textContent = `Position error: (${err.x.toFixed(2)}, ${err.y.toFixed(2)}, ${err.z.toFixed(2)})`;
    this.hud.querySelector('#hudVel').textContent = `Velocity: (${vel.x.toFixed(2)}, ${vel.y.toFixed(2)}, ${vel.z.toFixed(2)})`;
  }
}
