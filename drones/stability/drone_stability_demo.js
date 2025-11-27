import { Quadrotor } from '../../recycle_bin/drone-sim-core/physics/drone_core_physics.js';
import { GeometricController } from '../../recycle_bin/drone-sim-core/ai/drone_control.js';
import { clamp } from '../../recycle_bin/drone-sim-core/utils/drone_math.js';

export function initDroneStabilityDemo(container, options = {}) {
  if (!container || typeof THREE === 'undefined') {
    console.warn('Drone stability demo requires a valid container and Three.js');
    return { pause() {}, resume() {}, restart() {}, setPausedFromVisibility() {}, destroy() {} };
  }
  const demo = new DroneStabilityDemo(container, options);
  demo.start();
  return {
    pause: () => demo.pause(true),
    resume: () => demo.resume(true),
    restart: () => demo.restart(),
    setPausedFromVisibility: (visible) => demo.setPausedFromVisibility(visible),
    destroy: () => demo.destroy(),
  };
}

class DroneStabilityDemo {
  constructor(container, options) {
    this.container = container;
    this.options = Object.assign({ width: container.clientWidth || 800, height: container.clientHeight || 450 }, options);
    this.physicsRate = 240;
    this.userPaused = false;
    this.visibilityPaused = false;
    this.lastFrame = null;
    this.accumulator = 0;
    this._frameReq = null;

    this.targetPosition = new THREE.Vector3(0, 1.5, 0);
    this.targetYaw = 0;

    this.gains = {
      kp_pos: 6,
      kd_pos: 4.5,
      kp_att: 6,
      kd_att: 0.25,
      kp_yaw: 4,
      kd_yaw: 0.25,
      hoverTrim: 0,
    };

    this._initScene();
    this._initDrone();
    this._initController();
    this._initOverlay();
    this.restart();
  }

  _initScene() {
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x0c111c);

    const aspect = this.options.width / this.options.height;
    this.camera = new THREE.PerspectiveCamera(60, aspect, 0.05, 200);
    this.camera.position.set(0, 6, -8);
    this.camera.lookAt(new THREE.Vector3(0, 1.5, 0));

    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderer.setSize(this.options.width, this.options.height);
    this.renderer.setPixelRatio(window.devicePixelRatio || 1);
    this.renderer.shadowMap.enabled = true;
    this.container.innerHTML = '';
    this.container.appendChild(this.renderer.domElement);

    const ambient = new THREE.AmbientLight(0x7c8cab, 0.45);
    const dir = new THREE.DirectionalLight(0xffffff, 0.95);
    dir.position.set(4, 8, -3);
    dir.castShadow = true;
    this.scene.add(ambient, dir);

    const floorMat = new THREE.MeshStandardMaterial({ color: 0x1f2937, roughness: 0.85, metalness: 0.1 });
    const floor = new THREE.Mesh(new THREE.PlaneGeometry(12, 12), floorMat);
    floor.rotation.x = -Math.PI / 2;
    floor.position.y = 0;
    floor.receiveShadow = true;
    this.scene.add(floor);

    const wallMat = new THREE.MeshStandardMaterial({ color: 0x111827, roughness: 0.9, metalness: 0.05, side: THREE.DoubleSide });
    const wallGeoX = new THREE.PlaneGeometry(10, 5);
    const wallGeoZ = new THREE.PlaneGeometry(10, 5);

    const wallPos = [
      { geo: wallGeoZ, pos: new THREE.Vector3(0, 2.5, -5), rot: new THREE.Euler(0, 0, 0) },
      { geo: wallGeoZ, pos: new THREE.Vector3(0, 2.5, 5), rot: new THREE.Euler(0, Math.PI, 0) },
      { geo: wallGeoX, pos: new THREE.Vector3(-5, 2.5, 0), rot: new THREE.Euler(0, Math.PI / 2, 0) },
      { geo: wallGeoX, pos: new THREE.Vector3(5, 2.5, 0), rot: new THREE.Euler(0, -Math.PI / 2, 0) },
    ];

    wallPos.forEach((w) => {
      const mesh = new THREE.Mesh(w.geo, wallMat);
      mesh.position.copy(w.pos);
      mesh.rotation.copy(w.rot);
      mesh.receiveShadow = true;
      this.scene.add(mesh);
    });

    const ceiling = new THREE.Mesh(new THREE.PlaneGeometry(12, 12), floorMat.clone());
    ceiling.rotation.x = Math.PI / 2;
    ceiling.position.y = 5;
    ceiling.receiveShadow = true;
    this.scene.add(ceiling);
  }

  _initDrone() {
    this.drone = new Quadrotor({ mass: 1.05, inertia: new THREE.Vector3(0.021, 0.021, 0.04) });

    const body = new THREE.Mesh(
      new THREE.BoxGeometry(0.28, 0.08, 0.28),
      new THREE.MeshStandardMaterial({ color: 0x22d3ee, metalness: 0.35, roughness: 0.45 }),
    );
    body.castShadow = true;
    const armMat = new THREE.MeshStandardMaterial({ color: 0x0ea5e9, metalness: 0.25, roughness: 0.6 });
    const arm = new THREE.Mesh(new THREE.BoxGeometry(0.8, 0.05, 0.12), armMat);
    arm.castShadow = true;

    this.droneMesh = new THREE.Group();
    this.droneMesh.add(body);
    this.droneMesh.add(arm);

    const rotorGeo = new THREE.RingGeometry(0.09, 0.14, 16);
    const rotorMat = new THREE.MeshBasicMaterial({ color: 0xf8fafc });
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
      this.droneMesh.add(r);
    });

    this.axesHelper = new THREE.AxesHelper(0.6);
    this.axesHelper.visible = false;
    this.droneMesh.add(this.axesHelper);

    this.velocityHelper = new THREE.ArrowHelper(new THREE.Vector3(), new THREE.Vector3(), 0, 0x22d3ee);
    this.velocityHelper.visible = false;
    this.droneMesh.add(this.velocityHelper);

    this.scene.add(this.droneMesh);
  }

  _initController() {
    this.controller = new GeometricController({
      mass: this.drone.params?.mass || 1.05,
      inertia: this.drone.params?.inertia || new THREE.Vector3(0.021, 0.021, 0.04),
      kpPos: new THREE.Vector3(this.gains.kp_pos, this.gains.kp_pos, this.gains.kp_pos),
      kdPos: new THREE.Vector3(this.gains.kd_pos, this.gains.kd_pos, this.gains.kd_pos),
      kR: new THREE.Vector3(this.gains.kp_att, this.gains.kp_att, this.gains.kp_yaw),
      kOmega: new THREE.Vector3(this.gains.kd_att, this.gains.kd_att, this.gains.kd_yaw),
      maxThrust: 30,
    });
  }

  _initOverlay() {
    this.container.style.position = 'relative';
    this.overlay = document.createElement('div');
    this.overlay.style.cssText =
      'position:absolute; top:8px; left:8px; background:rgba(12,17,28,0.9); color:#e5e7eb; padding:10px; font-family:"Fira Code", monospace; border:1px solid rgba(34,211,238,0.45); border-radius:8px; z-index:5; max-width:260px;';
    this.overlay.innerHTML = '<div style="color:#22d3ee; font-size:13px; margin-bottom:6px;">Stability Tuning</div>';
    this.container.appendChild(this.overlay);

    this.controls = {};
    const sliderDefs = [
      { key: 'kp_pos', label: 'kp_pos', min: 0, max: 12, step: 0.1 },
      { key: 'kd_pos', label: 'kd_pos', min: 0, max: 10, step: 0.1 },
      { key: 'kp_att', label: 'kp_att', min: 0, max: 12, step: 0.1 },
      { key: 'kd_att', label: 'kd_att', min: 0, max: 4, step: 0.05 },
      { key: 'kp_yaw', label: 'kp_yaw', min: 0, max: 12, step: 0.1 },
      { key: 'kd_yaw', label: 'kd_yaw', min: 0, max: 4, step: 0.05 },
      { key: 'hoverTrim', label: 'hover_thrust_trim (N)', min: -3, max: 3, step: 0.05 },
    ];

    sliderDefs.forEach((def) => {
      const wrapper = document.createElement('div');
      wrapper.style.cssText = 'margin-bottom:6px; font-size:12px;';
      const label = document.createElement('label');
      label.textContent = `${def.label}: `;
      label.style.marginRight = '6px';
      const value = document.createElement('span');
      value.textContent = this.gains[def.key];
      value.style.color = '#a5b4fc';
      value.style.marginLeft = '4px';
      const input = document.createElement('input');
      input.type = 'range';
      input.min = def.min;
      input.max = def.max;
      input.step = def.step;
      input.value = this.gains[def.key];
      input.style.width = '160px';
      input.addEventListener('input', () => {
        const val = parseFloat(input.value);
        this.gains[def.key] = val;
        value.textContent = val.toFixed(2);
        this._updateControllerGains();
      });
      wrapper.appendChild(label);
      wrapper.appendChild(input);
      wrapper.appendChild(value);
      this.overlay.appendChild(wrapper);
      this.controls[def.key] = input;
    });

    const checkboxRow = document.createElement('div');
    checkboxRow.style.cssText = 'margin-top:8px; font-size:12px; display:flex; gap:12px; color:#cbd5e1;';

    const axisToggle = this._makeCheckbox('show axes', (checked) => {
      this.axesHelper.visible = checked;
    });
    const velToggle = this._makeCheckbox('show velocity', (checked) => {
      this.velocityHelper.visible = checked;
    });
    checkboxRow.appendChild(axisToggle.wrapper);
    checkboxRow.appendChild(velToggle.wrapper);
    this.overlay.appendChild(checkboxRow);
  }

  _makeCheckbox(labelText, onChange) {
    const wrapper = document.createElement('label');
    wrapper.style.cssText = 'display:flex; align-items:center; gap:4px; cursor:pointer;';
    const input = document.createElement('input');
    input.type = 'checkbox';
    input.addEventListener('change', () => onChange(input.checked));
    const label = document.createElement('span');
    label.textContent = labelText;
    wrapper.appendChild(input);
    wrapper.appendChild(label);
    return { wrapper, input };
  }

  _updateControllerGains() {
    if (!this.controller) return;
    this.controller.params.kpPos.set(this.gains.kp_pos, this.gains.kp_pos, this.gains.kp_pos);
    this.controller.params.kdPos.set(this.gains.kd_pos, this.gains.kd_pos, this.gains.kd_pos);
    this.controller.params.kR.set(this.gains.kp_att, this.gains.kp_att, this.gains.kp_yaw);
    this.controller.params.kOmega.set(this.gains.kd_att, this.gains.kd_att, this.gains.kd_yaw);
  }

  restart() {
    this.drone.reset({
      position: new THREE.Vector3(0, 0.5, 0),
      velocity: new THREE.Vector3(),
      quaternion: new THREE.Quaternion(),
      omega: new THREE.Vector3(),
    });
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
    const desired = {
      position: this.targetPosition,
      velocity: new THREE.Vector3(),
      acceleration: new THREE.Vector3(0, 0, 0),
      yaw: this.targetYaw,
    };
    const control = this.controller.computeControl(state, desired);
    const thrust = clamp(control.thrust + this.gains.hoverTrim, 0, this.controller.params.maxThrust);
    this.drone.step({ thrust, torque: control.torque }, dt);
  }

  _render() {
    const state = this.drone.getState();
    this.droneMesh.position.copy(state.position);
    this.droneMesh.quaternion.copy(state.quaternion);

    if (this.velocityHelper.visible) {
      const speed = state.velocity.length();
      const dir = speed > 1e-4 ? state.velocity.clone().normalize() : new THREE.Vector3(0, 0, 1);
      this.velocityHelper.setDirection(dir);
      this.velocityHelper.setLength(Math.min(Math.max(speed, 0.05), 2));
      this.velocityHelper.position.set(0, 0, 0);
    }

    this.renderer.render(this.scene, this.camera);
  }
}
