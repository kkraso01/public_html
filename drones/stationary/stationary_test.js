import { DronePhysicsEngine } from '../physics/drone_physics_engine.js';
import { CRAZYFLIE_PARAMS } from '../physics/params_crazyflie.js';
import { StationaryController } from './stationary_controller.js';

export function initStationaryHoverDemo(container, options = {}) {
  if (!container || typeof THREE === 'undefined') {
    console.warn('Stationary demo requires a valid container and Three.js');
    return { pause() {}, resume() {}, restart() {}, setPausedFromVisibility() {}, destroy() {}, handleResize() {} };
  }
  const demo = new StationaryHoverDemo(container, options);
  demo.pause(true);
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

/**
 * Stationary Hover Demo with ETH Cascaded Controller
 * 
 * X-FRAME MOTOR CONFIGURATION:
 * ----------------------------
 * Motor 0 (Red):    Front-Left  at (+X, -Z)
 * Motor 1 (Green):  Front-Right at (+X, +Z)
 * Motor 2 (Blue):   Back-Left   at (-X, -Z)
 * Motor 3 (Yellow): Back-Right  at (-X, +Z)
 * 
 * Control allocation:
 *   - Roll  (tau_x): M0 vs M2 (left vs right diagonal)
 *   - Pitch (tau_y): M1 vs M3 (front-right vs back-right diagonal)
 *   - Yaw   (tau_z): M0-M1+M2-M3 (counter-rotating pairs)
 */
class StationaryHoverDemo {
  constructor(container, options) {
    this.container = container;
    this.controlContainer = options.controlContainer;
    this.options = Object.assign({ width: container.clientWidth || 800, height: container.clientHeight || 480 }, options);
    this.physicsRate = 240;
    this.userPaused = true;
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

    this.droneInitialPosition = new THREE.Vector3(0, 0, 0.6);  // Hover at z=0.6 (+Z is up)
    this.droneInitialOrientation = new THREE.Quaternion().set(0, 0, 0, 1);

    this.controller = new StationaryController(this.params);
    // Initialize with tuned ETH Zürich gains (per-axis) for stable hover
    this.controller.updateGains({
      kp: { x: 6.0, y: 6.0, z: 8.0 },
      kd: { x: 4.0, y: 4.0, z: 5.0 },
      ki: { x: 0.12, y: 0.12, z: 0.12 },
    });
    this.controller.updateAttitudeGains({
      kR: { x: 9.0, y: 9.0, z: 5.0 },
      kOmega: { x: 0.4, y: 0.4, z: 0.3 },
    });
    // Limit climb aggressiveness to reduce vertical oscillations
    this.controller.eth.maxAcc = 15.0;

    this._initScene();
    this._initDrone();
    this._initCameraControls();
    this._initOverlay();
    this.restart();
  }

  _initScene() {
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x0c111c);

    const aspect = this.options.width / this.options.height;
    this.camera = new THREE.PerspectiveCamera(60, aspect, 0.05, 200);
    this.camera.position.set(2, -2, 2);  // Adjust for +Z up convention
    this.camera.lookAt(0, 0, 0.6);
    this.camera.up.set(0, 0, 1);  // +Z is up

    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderer.setSize(this.options.width, this.options.height);
    this.renderer.setPixelRatio(window.devicePixelRatio || 1);
    this.renderer.shadowMap.enabled = true;
    this.container.innerHTML = '';
    this.container.appendChild(this.renderer.domElement);

    this.scene.add(new THREE.AmbientLight(0xffffff, 0.6));
    const key = new THREE.DirectionalLight(0xffffff, 1.5);
    key.position.set(5, 8, 5);
    key.castShadow = true;
    this.scene.add(key);

    const floor = new THREE.Mesh(
      new THREE.PlaneGeometry(50, 50),
      new THREE.MeshStandardMaterial({ color: 0x20252b }),
    );
    // No rotation needed - plane is already in XY plane, which is horizontal when +Z is up
    floor.receiveShadow = true;
    this.scene.add(floor);

    // GridHelper creates grid in XZ plane by default, rotate it to XY plane for +Z up
    const grid = new THREE.GridHelper(10, 20, 0x475569, 0x1f2937);
    grid.rotation.x = Math.PI / 2;
    this.scene.add(grid);
  }

  _initCameraControls() {
    // Camera drag controls - independent of pause state
    this.cameraControls = {
      isDragging: false,
      previousMousePosition: { x: 0, y: 0 },
      rotation: { x: 0, y: 0 },
    };

    this.renderer.domElement.addEventListener('mousedown', (e) => {
      this.cameraControls.isDragging = true;
      this.cameraControls.previousMousePosition = { x: e.clientX, y: e.clientY };
    });

    this.renderer.domElement.addEventListener('mousemove', (e) => {
      if (this.cameraControls.isDragging) {
        const deltaX = e.clientX - this.cameraControls.previousMousePosition.x;
        const deltaY = e.clientY - this.cameraControls.previousMousePosition.y;

        this.cameraControls.rotation.y += deltaX * 0.005;
        this.cameraControls.rotation.x += deltaY * 0.005;

        // Clamp pitch
        this.cameraControls.rotation.x = Math.max(-Math.PI / 2, Math.min(Math.PI / 2, this.cameraControls.rotation.x));

        this.cameraControls.previousMousePosition = { x: e.clientX, y: e.clientY };

        // Update camera position
        const radius = 3;
        this.camera.position.x = radius * Math.sin(this.cameraControls.rotation.y) * Math.cos(this.cameraControls.rotation.x);
        this.camera.position.y = 0.6 + radius * Math.sin(this.cameraControls.rotation.x);
        this.camera.position.z = radius * Math.cos(this.cameraControls.rotation.y) * Math.cos(this.cameraControls.rotation.x);
        this.camera.lookAt(0, 0.6, 0);
        
        // Force re-render during drag
        this._render();
      }
    });

    this.renderer.domElement.addEventListener('mouseup', () => {
      this.cameraControls.isDragging = false;
    });

    this.renderer.domElement.addEventListener('mouseleave', () => {
      this.cameraControls.isDragging = false;
    });
  }

  _initDrone() {
    this.drone = new DronePhysicsEngine(this.params);
    this.drone.reset({
      position: new THREE.Vector3(0, 1, 0),
      velocity: new THREE.Vector3(0, 0, 0),
      orientation: new THREE.Quaternion().set(0, 0, 0, 1),
      angularVelocity: new THREE.Vector3(0, 0, 0),
    });

    // Create body with different materials for top and bottom
    // With +Z up: dimensions are (X, Y, Z) = (length, width, height)
    const bodyGeometry = new THREE.BoxGeometry(0.28, 0.28, 0.08);
    const topMat = new THREE.MeshStandardMaterial({ color: 0xff0000, metalness: 0.35, roughness: 0.45 });
    const bottomMat = new THREE.MeshStandardMaterial({ color: 0x0000ff, metalness: 0.35, roughness: 0.45 });
    const sideMat = new THREE.MeshStandardMaterial({ color: 0x22d3ee, metalness: 0.35, roughness: 0.45 });
    
    // BoxGeometry face order: right(+X), left(-X), top(+Z), bottom(-Z), front(+Y), back(-Y)
    const bodyMaterials = [sideMat, sideMat, topMat, bottomMat, sideMat, sideMat];
    const body = new THREE.Mesh(bodyGeometry, bodyMaterials);
    body.castShadow = true;
    body.receiveShadow = true;
    const armMat = new THREE.MeshStandardMaterial({ color: 0x0ea5e9, metalness: 0.25, roughness: 0.6 });
    // Arms along X axis with thin Z profile: (length=X, width=Y, height=Z)
    const arm = new THREE.Mesh(new THREE.BoxGeometry(0.8, 0.12, 0.05), armMat);
    arm.castShadow = true;
    arm.receiveShadow = true;

    this.droneMesh = new THREE.Group();
    this.droneMesh.add(body);
    this.droneMesh.add(arm);

    // ====================================================================
    // MOTOR CONFIGURATION: X-frame quadcopter (ETH Zürich / Crazyflie)
    // ====================================================================
    // Coordinate system: X=forward, Y=left, Z=up (ENU convention)
    // 
    // Physics engine torques (drone_physics_engine.js):
    //   τ_x = L * kF * (ω²[1] - ω²[3])  → Roll  (right motors push left)
    //   τ_y = L * kF * (ω²[2] - ω²[0])  → Pitch (front motors down tilts back)
    //   τ_z = kM * (-ω²[0] + ω²[1] - ω²[2] + ω²[3]) → Yaw (CW negative, CCW positive)
    //
    // Motor layout (top view, looking down -Z axis):
    //        Front (+X)
    //    M0 ←─── M1
    //     ╲     ╱
    //  (-Y)╲   ╱(+Y)
    //       ╲ ╱
    //       ╱ ╲
    //  (-Y)╱   ╲(+Y)
    //     ╱     ╲
    //    M3 ───→ M2
    //        Back (-X)
    //
    // Motor 0 (Red):    Front-Left  at (+X, -Y, +Z) → CW
    // Motor 1 (Green):  Front-Right at (+X, +Y, +Z) → CCW
    // Motor 2 (Blue):   Back-Right  at (-X, +Y, +Z) → CW
    // Motor 3 (Yellow): Back-Left   at (-X, -Y, +Z) → CCW
    // ====================================================================
    // ETH Zürich / Crazyflie motor layout (X-configuration)
    // Body frame: X forward, Y left, Z up (ENU convention)
    const rotorGeo = new THREE.RingGeometry(0.09, 0.14, 16);
    const rotorColors = [0xff0000, 0x00ff00, 0x0000ff, 0xffff00]; // Red, Green, Blue, Yellow
    const rotorPositions = [
      new THREE.Vector3(0.28, 0.28, 0.06),   // Motor 0 - Front-Left (CW) - Red: +X (front), +Y (left)
      new THREE.Vector3(0.28, -0.28, 0.06),  // Motor 1 - Front-Right (CCW) - Green: +X (front), -Y (right)
      new THREE.Vector3(-0.28, -0.28, 0.06), // Motor 2 - Back-Right (CW) - Blue: -X (back), -Y (right)
      new THREE.Vector3(-0.28, 0.28, 0.06),  // Motor 3 - Back-Left (CCW) - Yellow: -X (back), +Y (left)
    ];
    rotorPositions.forEach((p, i) => {
      const rotorMat = new THREE.MeshStandardMaterial({ color: rotorColors[i], emissive: rotorColors[i], emissiveIntensity: 0.5, metalness: 0.6, roughness: 0.4 });
      
      // Top blade (visible from above, +Z direction)
      // RingGeometry lies in XY plane by default (no rotation needed for +Z up)
      const rTop = new THREE.Mesh(rotorGeo, rotorMat);
      rTop.position.copy(p);
      rTop.position.z += 0.01; // Slight offset above rotor hub
      rTop.castShadow = true;
      rTop.receiveShadow = true;
      this.droneMesh.add(rTop);
      
      // Bottom blade (visible from below, -Z direction)
      const rBottom = new THREE.Mesh(rotorGeo, rotorMat);
      rBottom.position.copy(p);
      rBottom.position.z -= 0.01; // Slight offset below rotor hub
      rBottom.castShadow = true;
      rBottom.receiveShadow = true;
      this.droneMesh.add(rBottom);
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

    // Control buttons
    const buttonContainer = document.createElement('div');
    buttonContainer.style.cssText = 'display:grid; grid-template-columns:1fr 1fr 1fr; gap:6px; grid-column:1/-1;';
    
    const pauseBtn = document.createElement('button');
    pauseBtn.textContent = 'Resume';
    pauseBtn.style.cssText = 'padding:8px; background:#0ea5e9; color:#0c111c; border:none; border-radius:6px; font-weight:600; cursor:pointer; font-size:12px;';
    pauseBtn.addEventListener('click', () => {
      this.userPaused = !this.userPaused;
      if (this.userPaused) {
        this.pause(true);
        pauseBtn.textContent = 'Resume';
        // Enable drone position sliders when paused
        ['droneX', 'droneY', 'droneZ'].forEach((key) => {
          if (this.controls[key]) {
            this.controls[key].disabled = false;
            this.controls[key].style.cssText = 'cursor:pointer; opacity:1;';
          }
        });
      } else {
        this.resume(true);
        pauseBtn.textContent = 'Pause';
        // Disable drone position sliders when running
        ['droneX', 'droneY', 'droneZ'].forEach((key) => {
          if (this.controls[key]) {
            this.controls[key].disabled = true;
            this.controls[key].style.cssText = 'cursor:not-allowed; opacity:0.5;';
          }
        });
      }
    });
    
    const stepBtn = document.createElement('button');
    stepBtn.textContent = 'Step';
    stepBtn.style.cssText = 'padding:8px; background:#10b981; color:#0c111c; border:none; border-radius:6px; font-weight:600; cursor:pointer; font-size:12px;';
    stepBtn.addEventListener('click', () => {
      if (this.userPaused) {
        const fixedDt = 1 / this.physicsRate;
        this._stepPhysics(fixedDt);
        this._render();
      }
    });
    
    const resetBtn = document.createElement('button');
    resetBtn.textContent = 'Reset';
    resetBtn.style.cssText = 'padding:8px; background:#f59e0b; color:#0c111c; border:none; border-radius:6px; font-weight:600; cursor:pointer; font-size:12px;';
    resetBtn.addEventListener('click', () => {
      this.restart();
      if (this.userPaused) {
        this._render();
      }
    });
    
    buttonContainer.appendChild(pauseBtn);
    buttonContainer.appendChild(stepBtn);
    buttonContainer.appendChild(resetBtn);
    this.overlay.appendChild(buttonContainer);

    // Target position and orientation sliders
    const targetContainer = document.createElement('div');
    targetContainer.style.cssText = 'display:flex; flex-direction:column; gap:6px; background:rgba(168,85,247,0.08); border:1px solid rgba(168,85,247,0.3); border-radius:6px; padding:8px; grid-column:1/-1;';
    
    const targetLabel = document.createElement('div');
    targetLabel.textContent = 'Drone Initial Position & Orientation (Paused Only)';
    targetLabel.style.cssText = 'color:#a78bfa; font-weight:600; font-size:12px;';
    targetContainer.appendChild(targetLabel);

    this.controls = {};
    const targetSliderDefs = [
      { key: 'droneX', label: 'Drone X', min: -2, max: 2, step: 0.1, value: this.droneInitialPosition.x, getter: () => this.droneInitialPosition.x, setter: (v) => { this.droneInitialPosition.x = v; this.restart(); } },
      { key: 'droneY', label: 'Drone Y', min: 0, max: 2, step: 0.1, value: this.droneInitialPosition.y, getter: () => this.droneInitialPosition.y, setter: (v) => { this.droneInitialPosition.y = v; this.restart(); } },
      { key: 'droneZ', label: 'Drone Z', min: -2, max: 2, step: 0.1, value: this.droneInitialPosition.z, getter: () => this.droneInitialPosition.z, setter: (v) => { this.droneInitialPosition.z = v; this.restart(); } },
    ];

    targetSliderDefs.forEach((def) => {
      const wrapper = document.createElement('div');
      wrapper.style.cssText = 'display:flex; flex-direction:column; gap:4px;';
      const label = document.createElement('label');
      label.textContent = `${def.label}: ${def.value.toFixed(2)}`;
      label.style.cssText = 'font-size:11px; color:#d8b4fe;';
      const input = document.createElement('input');
      input.type = 'range';
      input.min = def.min;
      input.max = def.max;
      input.step = def.step;
      input.value = def.value;
      input.disabled = false;
      input.style.cssText = 'cursor:pointer; opacity:1;';
      input.addEventListener('change', () => {
        if (this.userPaused) {
          const val = parseFloat(input.value);
          label.textContent = `${def.label}: ${val.toFixed(2)}`;
          def.setter(val);
          this._render();
        }
      });
      input.addEventListener('input', () => {
        const val = parseFloat(input.value);
        label.textContent = `${def.label}: ${val.toFixed(2)}`;
      });
      wrapper.appendChild(label);
      wrapper.appendChild(input);
      targetContainer.appendChild(wrapper);
      this.controls[def.key] = input;
      this.controls[def.key]._def = def;
    });

    this.overlay.appendChild(targetContainer);

    const sliderDefs = [
      { key: 'kp', label: 'Kp (pos)', min: 0, max: 12, step: 0.1, value: this.controller.eth.Kp.x, onInput: (v) => this.controller.updateGains({ kp: v }) },
      { key: 'ki', label: 'Ki (pos)', min: 0, max: 2, step: 0.02, value: this.controller.eth.Ki.x, onInput: (v) => this.controller.updateGains({ ki: v }) },
      { key: 'kd', label: 'Kd (pos)', min: 0, max: 10, step: 0.05, value: this.controller.eth.Kd.x, onInput: (v) => this.controller.updateGains({ kd: v }) },
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
        (def.onInput || (() => {}))(val);
      });
      wrapper.appendChild(label);
      wrapper.appendChild(input);
      this.overlay.appendChild(wrapper);
      this.controls[def.key] = input;
    });

    const attitudeLabel = document.createElement('div');
    attitudeLabel.textContent = 'Attitude (inner loop) gains';
    attitudeLabel.style.cssText = 'color:#94a3b8; font-weight:600; font-size:12px; margin-top:6px;';
    this.overlay.appendChild(attitudeLabel);

    const attitudeSliderDefs = [
      { key: 'kRxy', label: 'KR roll/pitch', min: 0, max: 14, step: 0.1, value: this.controller.eth.KR.x, onInput: (v) => this.controller.updateAttitudeGains({ kR: { x: v, y: v } }) },
      { key: 'kRz', label: 'KR yaw', min: 0, max: 10, step: 0.1, value: this.controller.eth.KR.z, onInput: (v) => this.controller.updateAttitudeGains({ kR: { z: v } }) },
      { key: 'kOxy', label: 'Kω roll/pitch', min: 0, max: 2, step: 0.02, value: this.controller.eth.Komega.x, onInput: (v) => this.controller.updateAttitudeGains({ kOmega: { x: v, y: v } }) },
      { key: 'kOz', label: 'Kω yaw', min: 0, max: 2, step: 0.02, value: this.controller.eth.Komega.z, onInput: (v) => this.controller.updateAttitudeGains({ kOmega: { z: v } }) },
    ];

    attitudeSliderDefs.forEach((def) => {
      const wrapper = document.createElement('div');
      wrapper.style.cssText = 'display:flex; flex-direction:column; gap:4px; background:rgba(14,165,233,0.08); border:1px solid rgba(14,165,233,0.3); border-radius:6px; padding:8px;';
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
        (def.onInput || (() => {}))(val);
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
      <div id="hudMotors" style="display:flex; gap:8px; flex-wrap:wrap;">
        <span>Motors:</span>
        <span>M0=<span id="motor0" style="color:#ff0000; font-weight:bold;">0</span></span>
        <span>M1=<span id="motor1" style="color:#00ff00; font-weight:bold;">0</span></span>
        <span>M2=<span id="motor2" style="color:#0000ff; font-weight:bold;">0</span></span>
        <span>M3=<span id="motor3" style="color:#ffff00; font-weight:bold;">0</span></span>
      </div>
    `;
    this.overlay.appendChild(this.hud);

    container.appendChild(this.overlay);

    // Enable drone position sliders on init since starting paused
    ['droneX', 'droneY', 'droneZ'].forEach((key) => {
      if (this.controls[key]) {
        this.controls[key].disabled = false;
        this.controls[key].style.cssText = 'cursor:pointer; opacity:1;';
      }
    });
  }

  restart() {
    this.drone.reset({
      position: this.droneInitialPosition.clone(),
      velocity: new THREE.Vector3(0, 0, 0),
      orientation: this.droneInitialOrientation.clone(),
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
    // Don't resume; keep paused on start
    this._render();
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
    
    // Create target object for controller
    const target = {
      position: this.target.position.clone(),
      velocity: this.target.velocity.clone(),
      acceleration: new THREE.Vector3(0, 0, 0),
      yaw: this.target.yaw,
    };
    
    const result = this.controller.eth.computeControl(state, target, dt);
    const motorCommands = result.motorCommands;
    this.lastMotorCommands = motorCommands;
    
    // Store intermediate values for debugging
    this.lastControlResult = result;
    
    // Calculate position error and velocity for logging
    const posError = this.target.position.clone().sub(state.position);
    const vel = state.velocity;
    
    // Log every 10 steps to avoid spam
    // Motor colors: M0=Red(FL), M1=Green(FR), M2=Blue(BL), M3=Yellow(BR)
    if (!this.logCounter) this.logCounter = 0;
    this.logCounter++;
    if (this.logCounter >= 10) {
      const pwm0 = Math.round(motorCommands[0] * 65535); // Front-Left (Red)
      const pwm1 = Math.round(motorCommands[1] * 65535); // Front-Right (Green)
      const pwm2 = Math.round(motorCommands[2] * 65535); // Back-Left (Blue)
      const pwm3 = Math.round(motorCommands[3] * 65535); // Back-Right (Yellow)
      
      // Extract Euler angles for debugging
      const euler = new THREE.Euler().setFromQuaternion(state.orientationQuat, 'XYZ');
      const roll = (euler.x * 180 / Math.PI).toFixed(1);
      const pitch = (euler.y * 180 / Math.PI).toFixed(1);
      const yaw = (euler.z * 180 / Math.PI).toFixed(1);
      
      // Get desired orientation
      let desRoll = 0, desPitch = 0;
      if (this.lastControlResult && this.lastControlResult.desiredOrientation) {
        const desEuler = new THREE.Euler().setFromQuaternion(this.lastControlResult.desiredOrientation, 'XYZ');
        desRoll = (desEuler.x * 180 / Math.PI).toFixed(1);
        desPitch = (desEuler.y * 180 / Math.PI).toFixed(1);
      }
      
      console.log(
        `[${this.simTime.toFixed(3)}s] Pos: (${state.position.x.toFixed(2)}, ${state.position.y.toFixed(2)}, ${state.position.z.toFixed(2)}) | ` +
        `Error: (${posError.x.toFixed(2)}, ${posError.y.toFixed(2)}, ${posError.z.toFixed(2)}) | ` +
        `Vel: (${vel.x.toFixed(2)}, ${vel.y.toFixed(2)}, ${vel.z.toFixed(2)}) | ` +
        `Actual RPY: (${roll}°, ${pitch}°, ${yaw}°) | Desired RP: (${desRoll}°, ${desPitch}°) | Motors: ` +
        `M0=%c${pwm0}%c, M1=%c${pwm1}%c, M2=%c${pwm2}%c, M3=%c${pwm3}%c`,
        'color: #ff0000; font-weight: bold', 'color: inherit',
        'color: #00ff00; font-weight: bold', 'color: inherit',
        'color: #0000ff; font-weight: bold', 'color: inherit',
        'color: #ffff00; font-weight: bold', 'color: inherit'
      );
      this.logCounter = 0;
    }
    
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
    
    // Display last motor commands if available (convert 0-1 to 0-65535 for PWM equivalent)
    if (this.lastMotorCommands) {
      const pwm0 = Math.round(this.lastMotorCommands[0] * 65535);
      const pwm1 = Math.round(this.lastMotorCommands[1] * 65535);
      const pwm2 = Math.round(this.lastMotorCommands[2] * 65535);
      const pwm3 = Math.round(this.lastMotorCommands[3] * 65535);
      this.hud.querySelector('#motor0').textContent = pwm0;
      this.hud.querySelector('#motor1').textContent = pwm1;
      this.hud.querySelector('#motor2').textContent = pwm2;
      this.hud.querySelector('#motor3').textContent = pwm3;
    }
  }
}
