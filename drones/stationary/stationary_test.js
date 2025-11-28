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

    this.droneInitialPosition = new THREE.Vector3(0, 0.6, 0);
    this.droneInitialOrientation = new THREE.Quaternion().set(0, 0, 0, 1);

    this.controller = new StationaryController(this.params);

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
    this.camera.position.set(2, 2, 2);
    this.camera.lookAt(0, 0.6, 0);
    this.camera.up.set(0, 1, 0);

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
    floor.rotation.x = -Math.PI / 2;
    floor.receiveShadow = true;
    this.scene.add(floor);

    const grid = new THREE.GridHelper(10, 20, 0x475569, 0x1f2937);
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
    const bodyGeometry = new THREE.BoxGeometry(0.28, 0.08, 0.28);
    const topMat = new THREE.MeshStandardMaterial({ color: 0xff0000, metalness: 0.35, roughness: 0.45 });
    const bottomMat = new THREE.MeshStandardMaterial({ color: 0x0000ff, metalness: 0.35, roughness: 0.45 });
    const sideMat = new THREE.MeshStandardMaterial({ color: 0x22d3ee, metalness: 0.35, roughness: 0.45 });
    
    // BoxGeometry face order: right, left, top, bottom, front, back
    const bodyMaterials = [sideMat, sideMat, topMat, bottomMat, sideMat, sideMat];
    const body = new THREE.Mesh(bodyGeometry, bodyMaterials);
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
    const rotorMat = new THREE.MeshStandardMaterial({ color: 0xf8fafc, emissive: 0x0ea5e9, emissiveIntensity: 0.5, metalness: 0.6, roughness: 0.4 });
    const rotorPositions = [
      new THREE.Vector3(0.4, 0.06, 0.4),
      new THREE.Vector3(-0.4, 0.06, 0.4),
      new THREE.Vector3(-0.4, 0.06, -0.4),
      new THREE.Vector3(0.4, 0.06, -0.4),
    ];
    rotorPositions.forEach((p) => {
      // Top blade (facing up)
      const rTop = new THREE.Mesh(rotorGeo, rotorMat);
      rTop.rotation.x = Math.PI / 2;
      rTop.position.copy(p);
      rTop.castShadow = true;
      rTop.receiveShadow = true;
      this.droneMesh.add(rTop);
      
      // Bottom blade (facing down)
      const rBottom = new THREE.Mesh(rotorGeo, rotorMat);
      rBottom.rotation.x = -Math.PI / 2;
      rBottom.position.copy(p);
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
      { key: 'kp', label: 'Kp (pos)', min: 0, max: 12, step: 0.1, value: this.controller.eth.Kp.x * 0.5 },
      { key: 'ki', label: 'Ki (pos)', min: 0, max: 6, step: 0.05, value: this.controller.eth.Ki.x * 0.5 },
      { key: 'kd', label: 'Kd (pos)', min: 0, max: 8, step: 0.05, value: this.controller.eth.Kd.x * 0.5 },
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
      <div id="hudMotors">Motors: (0,0,0,0)</div>
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
    
    // Calculate position error and velocity for logging
    const posError = this.target.position.clone().sub(state.position);
    const vel = state.velocity;
    
    // Log every 10 steps to avoid spam
    if (!this.logCounter) this.logCounter = 0;
    this.logCounter++;
    if (this.logCounter >= 10) {
      const pwm1 = Math.round(motorCommands[0] * 65535);
      const pwm2 = Math.round(motorCommands[1] * 65535);
      const pwm3 = Math.round(motorCommands[2] * 65535);
      const pwm4 = Math.round(motorCommands[3] * 65535);
      console.log(`[${this.simTime.toFixed(3)}s] Pos: (${state.position.x.toFixed(2)}, ${state.position.y.toFixed(2)}, ${state.position.z.toFixed(2)}) | Error: (${posError.x.toFixed(2)}, ${posError.y.toFixed(2)}, ${posError.z.toFixed(2)}) | Vel: (${vel.x.toFixed(2)}, ${vel.y.toFixed(2)}, ${vel.z.toFixed(2)}) | Motors: (${pwm1}, ${pwm2}, ${pwm3}, ${pwm4})`);
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
      const pwm1 = Math.round(this.lastMotorCommands[0] * 65535);
      const pwm2 = Math.round(this.lastMotorCommands[1] * 65535);
      const pwm3 = Math.round(this.lastMotorCommands[2] * 65535);
      const pwm4 = Math.round(this.lastMotorCommands[3] * 65535);
      this.hud.querySelector('#hudMotors').textContent = `Motors: (${pwm1}, ${pwm2}, ${pwm3}, ${pwm4})`;
    }
  }
}
