/*
 * AlphaPilot-style autonomous drone race demo.
 * This file exposes initDroneRaceDemo(container, options) which creates a self-contained
 * Three.js scene with a single autonomous drone flying a preplanned track.
 */

function initDroneRaceDemo(container, options = {}) {
  if (!container) {
    console.error('initDroneRaceDemo: container element is required');
    return {
      pause() {},
      resume() {},
      restart() {},
      setPausedFromVisibility() {},
    };
  }

  // Gracefully handle environments without Three.js/WebGL support.
  if (typeof THREE === 'undefined' || !window.WebGLRenderingContext) {
    const fallback = document.createElement('div');
    fallback.textContent = 'WebGL / Three.js unavailable. Drone race demo could not start.';
    fallback.style.cssText = 'padding:12px; color:#fff; background:#222; font-family:monospace;';
    container.innerHTML = '';
    container.appendChild(fallback);
    return {
      pause() {},
      resume() {},
      restart() {},
      setPausedFromVisibility() {},
    };
  }

  const demo = new DroneRaceDemo(container, options);
  demo.start();

  return {
    pause: () => demo.pause(),
    resume: () => demo.resume(),
    restart: () => demo.restart(),
    setPausedFromVisibility: (visible) => demo.setPausedFromVisibility(visible),
  };
}

class DroneRaceDemo {
  constructor(container, options) {
    this.container = container;
    this.options = Object.assign(
      {
        width: container.clientWidth || 640,
        height: container.clientHeight || 360,
        enableShadows: true,
        highQuality: true,
      },
      options,
    );

    this._rafId = null;
    this._lastTimestamp = null;
    this._paused = false;
    this._userPaused = false;

    // Simulation parameters
    this.gravity = new THREE.Vector3(0, -9.81, 0);
    this.damping = 0.98;
    this.timeScale = 1.0;

    // Race state
    this.state = 'idle';
    this.gateIndex = 0;
    this.gatesPassed = 0;
    this.raceTime = 0;
    this.countdown = 1.5;

    // Drone physical state (true)
    this.trueState = {
      position: new THREE.Vector3(),
      velocity: new THREE.Vector3(),
      orientation: new THREE.Euler(0, 0, 0, 'YXZ'),
      angularVelocity: new THREE.Vector3(),
    };

    // Estimated state (simulated VIO with noise & drift)
    this.estimatedState = {
      position: new THREE.Vector3(),
      velocity: new THREE.Vector3(),
      orientation: new THREE.Euler(0, 0, 0, 'YXZ'),
    };
    this.vioBias = new THREE.Vector3();

    // Controller gains (tuneable)
    this.pid = {
      posP: 2.5,
      velP: 2.2,
      yawP: 2.5,
      altitudeP: 3.0,
    };

    this.maxAccel = 18;
    this.maxSpeed = 12;
    this.maxYawRate = Math.PI;

    this._initStyles();
    this._initThree();
    this._initWorld();
    this._initUI();
    this._resetDrone();
  }

  _initStyles() {
    if (document.getElementById('drone-race-demo-styles')) return;
    const style = document.createElement('style');
    style.id = 'drone-race-demo-styles';
    style.textContent = `
      .drone-race-container { position: relative; overflow: hidden; background: linear-gradient(180deg, #0f172a 0%, #0b1020 100%); }
      .drone-race-overlay { position: absolute; top: 10px; left: 10px; padding: 10px 12px; background: rgba(10, 14, 26, 0.65); color: #e5e7eb; font-family: 'Fira Code', 'SFMono-Regular', Consolas, 'Liberation Mono', Menlo, monospace; border: 1px solid rgba(255, 255, 255, 0.08); border-radius: 8px; box-shadow: 0 10px 30px rgba(0,0,0,0.25); }
      .drone-race-overlay h4 { margin: 0 0 6px 0; font-size: 13px; letter-spacing: 0.04em; color: #a5b4fc; }
      .drone-race-overlay .stat { font-size: 12px; line-height: 1.4; }
      .drone-race-controls { position: absolute; bottom: 12px; left: 10px; padding: 10px 12px; background: rgba(10, 14, 26, 0.65); color: #e5e7eb; font-family: 'Inter', system-ui, -apple-system, sans-serif; border: 1px solid rgba(255,255,255,0.08); border-radius: 8px; box-shadow: 0 10px 30px rgba(0,0,0,0.2); display: flex; flex-direction: column; gap: 8px; min-width: 200px; }
      .drone-race-controls button { cursor: pointer; padding: 8px 10px; border: 1px solid rgba(255,255,255,0.15); border-radius: 6px; background: linear-gradient(135deg, rgba(79,70,229,0.8), rgba(59,130,246,0.75)); color: #fff; font-weight: 600; letter-spacing: 0.01em; transition: transform 0.08s ease, box-shadow 0.12s ease; }
      .drone-race-controls button:hover { transform: translateY(-1px); box-shadow: 0 6px 18px rgba(59,130,246,0.4); }
      .drone-race-controls button:active { transform: translateY(0); box-shadow: none; }
      .drone-race-controls label { font-size: 12px; color: #cbd5e1; display: flex; flex-direction: column; gap: 4px; }
      .drone-race-controls input[type=range] { accent-color: #8b5cf6; }
    `;
    document.head.appendChild(style);
  }

  _initThree() {
    this.container.classList.add('drone-race-container');
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x0b1224);

    const fov = 60;
    const aspect = this.options.width / this.options.height;
    this.camera = new THREE.PerspectiveCamera(fov, aspect, 0.1, 500);
    this.camera.position.set(0, 4, 10);

    this.renderer = new THREE.WebGLRenderer({ antialias: !!this.options.highQuality });
    this.renderer.setSize(this.options.width, this.options.height);
    this.renderer.setPixelRatio(this.options.highQuality ? window.devicePixelRatio : 1);
    this.renderer.shadowMap.enabled = !!this.options.enableShadows;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;

    this.container.innerHTML = '';
    this.container.appendChild(this.renderer.domElement);

    window.addEventListener('resize', () => {
      const width = this.options.width || this.container.clientWidth;
      const height = this.options.height || this.container.clientHeight;
      this.camera.aspect = width / height;
      this.camera.updateProjectionMatrix();
      this.renderer.setSize(width, height);
    });
  }

  _initWorld() {
    // Lighting
    const ambient = new THREE.AmbientLight(0x8795a1, 0.6);
    this.scene.add(ambient);

    const dirLight = new THREE.DirectionalLight(0xffffff, 0.9);
    dirLight.position.set(8, 15, 12);
    dirLight.castShadow = !!this.options.enableShadows;
    dirLight.shadow.mapSize.set(1024, 1024);
    this.scene.add(dirLight);

    // Floor
    const floorGeo = new THREE.PlaneGeometry(80, 80);
    const floorMat = new THREE.MeshStandardMaterial({ color: 0x0f172a, roughness: 0.6, metalness: 0.1 });
    const floor = new THREE.Mesh(floorGeo, floorMat);
    floor.rotation.x = -Math.PI / 2;
    floor.receiveShadow = true;
    this.scene.add(floor);

    // Sparse columns/walls for indoor vibe
    const columnGeo = new THREE.CylinderGeometry(0.3, 0.4, 6, 16);
    const columnMat = new THREE.MeshStandardMaterial({ color: 0x1f2937, roughness: 0.8 });
    const columnPositions = [
      [6, 3, 6],
      [-6, 3, -4],
      [10, 3, -2],
      [-10, 3, 8],
    ];
    columnPositions.forEach(([x, y, z]) => {
      const col = new THREE.Mesh(columnGeo, columnMat);
      col.position.set(x, y, z);
      col.castShadow = col.receiveShadow = true;
      this.scene.add(col);
    });

    // Track & gates
    this.track = this._initTrack();
    this._createGates();

    // Drone visual
    this._createDroneMesh();
  }

  _initTrack() {
    const gates = [];
    const points = [
      new THREE.Vector3(-12, 2.2, -8),
      new THREE.Vector3(-6, 3.4, -2),
      new THREE.Vector3(-2, 2.8, 5),
      new THREE.Vector3(4, 3.2, 8),
      new THREE.Vector3(10, 2.6, 4),
      new THREE.Vector3(14, 3.4, -2),
      new THREE.Vector3(8, 3.0, -8),
      new THREE.Vector3(2, 2.6, -12),
      new THREE.Vector3(-6, 2.4, -10),
      new THREE.Vector3(-10, 3.0, -4),
      new THREE.Vector3(-12, 2.8, 2),
      new THREE.Vector3(-6, 3.2, 6),
    ];

    for (let i = 0; i < points.length; i++) {
      const current = points[i];
      const next = points[(i + 1) % points.length];
      const normal = new THREE.Vector3().subVectors(next, current).normalize();
      gates.push({ id: i + 1, position: current.clone(), normal, radius: 2.5 });
    }

    // Precompute trajectory samples using Catmull–Rom spline
    this.trajectory = this._planTrajectory(points);
    return gates;
  }

  _planTrajectory(points) {
    const samples = [];
    const catmull = new THREE.CatmullRomCurve3(points, true, 'catmullrom', 0.25);
    const totalLength = catmull.getLength();
    const desiredSpeed = this.options.highQuality ? 7.5 : 6.0;
    const totalTime = totalLength / desiredSpeed;
    const segments = 400;
    for (let i = 0; i <= segments; i++) {
      const u = i / segments;
      const pos = catmull.getPointAt(u);
      const tangent = catmull.getTangentAt(u).normalize();
      samples.push({
        t: u * totalTime,
        position: pos,
        direction: tangent,
      });
    }
    return { samples, totalTime };
  }

  _createGates() {
    const gateMaterial = (color) =>
      new THREE.MeshStandardMaterial({ color, emissive: color, emissiveIntensity: 0.25, metalness: 0.2, roughness: 0.4 });

    this.gateMeshes = [];
    const ringThickness = 0.2;
    const gateSize = 3.2;

    this.track.forEach((gate, idx) => {
      const colorPalette = [0x60a5fa, 0xa855f7, 0x22d3ee, 0xf97316, 0x4ade80, 0xf43f5e];
      const color = colorPalette[idx % colorPalette.length];

      const frameGeo = new THREE.TorusGeometry(gateSize, ringThickness, 16, 32);
      const frameMat = gateMaterial(color);
      const frame = new THREE.Mesh(frameGeo, frameMat);
      frame.position.copy(gate.position);

      // Orient torus so its normal aligns to gate.normal (default normal is +Z)
      const quat = new THREE.Quaternion();
      quat.setFromUnitVectors(new THREE.Vector3(0, 0, 1), gate.normal.clone().normalize());
      frame.quaternion.copy(quat);
      frame.castShadow = frame.receiveShadow = true;

      // Add label
      const label = this._makeTextSprite(`Gate ${gate.id}`, color);
      label.position.copy(gate.position.clone().add(new THREE.Vector3(0, gateSize * 0.7, 0)));

      this.scene.add(frame);
      this.scene.add(label);
      this.gateMeshes.push({ frame, label });
    });
  }

  _makeTextSprite(text, color) {
    const canvas = document.createElement('canvas');
    const ctx = canvas.getContext('2d');
    const padding = 12;
    ctx.font = '20px "Fira Code", monospace';
    const textWidth = ctx.measureText(text).width + padding * 2;
    canvas.width = textWidth;
    canvas.height = 36;

    ctx.fillStyle = 'rgba(15, 23, 42, 0.8)';
    ctx.fillRect(0, 0, canvas.width, canvas.height);
    ctx.strokeStyle = 'rgba(255,255,255,0.12)';
    ctx.strokeRect(0, 0, canvas.width, canvas.height);
    ctx.fillStyle = `#${color.toString(16).padStart(6, '0')}`;
    ctx.textBaseline = 'middle';
    ctx.fillText(text, padding, canvas.height / 2);

    const texture = new THREE.CanvasTexture(canvas);
    texture.minFilter = THREE.LinearFilter;
    const material = new THREE.SpriteMaterial({ map: texture, transparent: true });
    const sprite = new THREE.Sprite(material);
    const scale = 1.1;
    sprite.scale.set(canvas.width / 100 * scale, canvas.height / 100 * scale, 1);
    return sprite;
  }

  _createDroneMesh() {
    const bodyGeo = new THREE.BoxGeometry(0.7, 0.2, 0.4);
    const bodyMat = new THREE.MeshStandardMaterial({ color: 0x93c5fd, metalness: 0.3, roughness: 0.35, emissive: 0x1f2937, emissiveIntensity: 0.2 });
    this.droneMesh = new THREE.Mesh(bodyGeo, bodyMat);
    this.droneMesh.castShadow = true;

    const armGeo = new THREE.CylinderGeometry(0.04, 0.04, 0.7, 8);
    const armMat = new THREE.MeshStandardMaterial({ color: 0x1e293b });
    const armOffsets = [
      [0, 0, 0.25],
      [0, 0, -0.25],
    ];
    armOffsets.forEach(([x, y, z]) => {
      const arm = new THREE.Mesh(armGeo, armMat);
      arm.rotation.z = Math.PI / 2;
      arm.position.set(x, y, z);
      arm.castShadow = true;
      this.droneMesh.add(arm);
    });

    // Simple rotor disks
    const rotorGeo = new THREE.CircleGeometry(0.15, 16);
    const rotorMat = new THREE.MeshBasicMaterial({ color: 0x38bdf8, transparent: true, opacity: 0.6 });
    const rotorOffsets = [
      [0.35, 0, 0.35],
      [-0.35, 0, 0.35],
      [0.35, 0, -0.35],
      [-0.35, 0, -0.35],
    ];
    this.rotors = [];
    rotorOffsets.forEach(([x, y, z]) => {
      const rotor = new THREE.Mesh(rotorGeo, rotorMat);
      rotor.rotation.x = -Math.PI / 2;
      rotor.position.set(x, 0.12, z);
      this.droneMesh.add(rotor);
      this.rotors.push(rotor);
    });

    this.scene.add(this.droneMesh);
  }

  _initUI() {
    const overlay = document.createElement('div');
    overlay.className = 'drone-race-overlay';
    overlay.innerHTML = `
      <h4>Autonomous Drone</h4>
      <div class="stat" id="drone-race-state"></div>
      <div class="stat" id="drone-race-gate"></div>
      <div class="stat" id="drone-race-lap"></div>
      <div class="stat" id="drone-race-speed"></div>
      <div class="stat" id="drone-race-error"></div>
    `;

    const controls = document.createElement('div');
    controls.className = 'drone-race-controls';
    controls.innerHTML = `
      <button id="drone-race-toggle">Pause</button>
      <button id="drone-race-restart">Restart Race</button>
      <label>Playback Speed <input id="drone-race-speed-slider" type="range" min="0.5" max="2" value="1" step="0.1"></label>
    `;

    this.container.appendChild(overlay);
    this.container.appendChild(controls);

    this.stateLabel = overlay.querySelector('#drone-race-state');
    this.gateLabel = overlay.querySelector('#drone-race-gate');
    this.lapLabel = overlay.querySelector('#drone-race-lap');
    this.speedLabel = overlay.querySelector('#drone-race-speed');
    this.errorLabel = overlay.querySelector('#drone-race-error');

    const toggleBtn = controls.querySelector('#drone-race-toggle');
    toggleBtn.addEventListener('click', () => {
      if (this._paused) {
        this._userPaused = false;
        this.resume();
        toggleBtn.textContent = 'Pause';
      } else {
        this._userPaused = true;
        this.pause();
        toggleBtn.textContent = 'Resume';
      }
    });

    controls.querySelector('#drone-race-restart').addEventListener('click', () => {
      this.restart();
      toggleBtn.textContent = 'Pause';
    });

    controls.querySelector('#drone-race-speed-slider').addEventListener('input', (e) => {
      this.timeScale = parseFloat(e.target.value);
    });
  }

  _resetDrone() {
    const start = this.trajectory.samples[0];
    this.trueState.position.copy(start.position);
    this.trueState.velocity.set(0, 0, 0);
    this.trueState.orientation.set(0, 0, 0);
    this.estimatedState.position.copy(start.position);
    this.estimatedState.velocity.set(0, 0, 0);
    this.estimatedState.orientation.set(0, 0, 0);
    this.vioBias.set(0, 0, 0);

    this.gateIndex = 0;
    this.gatesPassed = 0;
    this.raceTime = 0;
    this.countdown = 1.5;
    this.state = 'countdown';
  }

  start() {
    this.resume();
  }

  pause() {
    if (this._paused) return;
    this._paused = true;
    if (this._rafId != null) {
      cancelAnimationFrame(this._rafId);
      this._rafId = null;
    }
  }

  resume() {
    if (!this._paused) return;
    this._paused = false;
    this._lastTimestamp = null;
    this._rafId = requestAnimationFrame(this._loop.bind(this));
  }

  restart() {
    this._resetDrone();
    this._lastTimestamp = null;
    if (!this._paused) {
      this.pause();
      this.resume();
    }
  }

  setPausedFromVisibility(visible) {
    if (!visible) {
      this.pause();
    } else if (!this._userPaused) {
      this.resume();
    }
  }

  _loop(timestamp) {
    if (this._paused) {
      this._lastTimestamp = timestamp;
      return;
    }
    if (this._lastTimestamp == null) this._lastTimestamp = timestamp;
    const dt = Math.min((timestamp - this._lastTimestamp) / 1000, 0.05);
    this._lastTimestamp = timestamp;

    const scaledDt = dt * this.timeScale;
    this.update(scaledDt);
    this.renderer.render(this.scene, this.camera);

    this._rafId = requestAnimationFrame(this._loop.bind(this));
  }

  update(dt) {
    this._updateRaceState(dt);
    if (this.state === 'racing') {
      this.raceTime += dt;
    }

    const reference = this._getReferenceAtTime(this.raceTime);
    const controlOutput = this._updateControl(reference, dt);
    this._updatePhysics(controlOutput, dt);
    this._updateVIO(dt);
    this._updateGateCrossing();
    this._updateCamera(dt, reference);
    this._updateHUD(reference);
  }

  _updateRaceState(dt) {
    if (this.state === 'countdown') {
      this.countdown -= dt;
      if (this.countdown <= 0) {
        this.state = 'racing';
        this.raceTime = 0;
      }
    } else if (this.state === 'finished') {
      // remain finished
    }
  }

  _getReferenceAtTime(time) {
    const traj = this.trajectory;
    if (!traj || traj.samples.length === 0) return { position: new THREE.Vector3(), direction: new THREE.Vector3(1, 0, 0), yaw: 0 };
    const wrappedTime = traj.totalTime > 0 ? time % traj.totalTime : 0;

    // Find two samples around wrappedTime
    let i = 0;
    while (i < traj.samples.length - 1 && traj.samples[i + 1].t < wrappedTime) i++;
    const a = traj.samples[i];
    const b = traj.samples[(i + 1) % traj.samples.length];
    const span = (b.t >= a.t ? b.t - a.t : (traj.totalTime - a.t) + b.t) || 1e-6;
    const alpha = ((wrappedTime - a.t + (wrappedTime < a.t ? traj.totalTime : 0)) / span) || 0;
    const position = new THREE.Vector3().lerpVectors(a.position, b.position, alpha);
    const direction = new THREE.Vector3().lerpVectors(a.direction, b.direction, alpha).normalize();
    const yaw = Math.atan2(direction.x, direction.z);
    return { position, direction, yaw };
  }

  _updateControl(reference, dt) {
    // Position PID to desired velocity/acceleration
    const posError = new THREE.Vector3().subVectors(reference.position, this.estimatedState.position);
    const desiredVel = new THREE.Vector3().copy(reference.direction).multiplyScalar(this.maxSpeed * 0.55).add(posError.multiplyScalar(this.pid.posP));
    const velError = desiredVel.sub(this.estimatedState.velocity);
    const desiredAcc = velError.multiplyScalar(this.pid.velP);

    // Altitude control
    const altitudeError = reference.position.y - this.estimatedState.position.y;
    desiredAcc.y += altitudeError * this.pid.altitudeP;

    // Limit acceleration magnitude
    if (desiredAcc.length() > this.maxAccel) desiredAcc.setLength(this.maxAccel);

    // Yaw control to face along the path direction
    const currentYaw = this.estimatedState.orientation.y;
    let yawError = reference.yaw - currentYaw;
    yawError = Math.atan2(Math.sin(yawError), Math.cos(yawError));
    const yawRateCmd = THREE.MathUtils.clamp(yawError * this.pid.yawP, -this.maxYawRate, this.maxYawRate);

    return { desiredAcc, yawRateCmd };
  }

  _updatePhysics(control, dt) {
    // Apply acceleration in world frame (simplified point-mass model)
    const acc = control.desiredAcc.clone();
    acc.add(this.gravity);
    this.trueState.velocity.add(acc.multiplyScalar(dt));

    if (this.trueState.velocity.length() > this.maxSpeed) {
      this.trueState.velocity.setLength(this.maxSpeed);
    }

    this.trueState.velocity.multiplyScalar(this.damping);
    this.trueState.position.add(this.trueState.velocity.clone().multiplyScalar(dt));

    // Orientation integrates yaw rate, roll/pitch relax back to neutral
    this.trueState.orientation.y += control.yawRateCmd * dt;
    this.trueState.orientation.x *= 0.92;
    this.trueState.orientation.z *= 0.92;

    // Update drone mesh pose
    this.droneMesh.position.copy(this.trueState.position);
    this.droneMesh.rotation.copy(this.trueState.orientation);

    // Spin rotors visually
    this.rotors.forEach((rotor, idx) => {
      const spin = (idx % 2 === 0 ? 1 : -1) * 30 * dt;
      rotor.material.opacity = 0.45 + 0.15 * Math.random();
      rotor.rotation.y += spin;
    });
  }

  _updateVIO(dt) {
    // Random walk bias to emulate VIO drift
    const biasStep = new THREE.Vector3(
      (Math.random() - 0.5) * 0.01,
      (Math.random() - 0.5) * 0.01,
      (Math.random() - 0.5) * 0.01,
    );
    this.vioBias.add(biasStep.multiplyScalar(dt * 4));

    const noise = () => (Math.random() - 0.5) * 0.03;
    this.estimatedState.position.copy(this.trueState.position).add(this.vioBias).add(new THREE.Vector3(noise(), noise(), noise()));
    this.estimatedState.velocity.copy(this.trueState.velocity);
    this.estimatedState.orientation.copy(this.trueState.orientation);
    this.estimatedState.orientation.y += noise() * 0.5;
  }

  _updateGateCrossing() {
    if (this.state !== 'racing') return;
    const gate = this.track[this.gateIndex];
    if (!gate) {
      this.state = 'finished';
      return;
    }

    const toDrone = new THREE.Vector3().subVectors(this.trueState.position, gate.position);
    const distToPlane = toDrone.dot(gate.normal);
    const projected = toDrone.clone().sub(gate.normal.clone().multiplyScalar(distToPlane));
    const withinRadius = projected.length() <= gate.radius;

    if (withinRadius && distToPlane < 0.5 && distToPlane > -0.5) {
      // Pass through gate by checking sign change over last frame
      if (!this._lastGateSign) this._lastGateSign = distToPlane > 0 ? 1 : -1;
      const signNow = distToPlane > 0 ? 1 : -1;
      if (signNow !== this._lastGateSign) {
        this.gateIndex = (this.gateIndex + 1) % this.track.length;
        this.gatesPassed += 1;
        if (this.gateIndex === 0) {
          this.state = 'finished';
        }
      }
      this._lastGateSign = signNow;
    }
  }

  _updateCamera(dt, reference) {
    // Third-person chase camera with look-ahead
    const lookAhead = reference.direction.clone().multiplyScalar(4);
    const desiredPos = this.trueState.position.clone().add(new THREE.Vector3(0, 2, 0)).sub(reference.direction.clone().multiplyScalar(6));
    this.camera.position.lerp(desiredPos, 0.08);
    const target = this.trueState.position.clone().add(lookAhead);
    this.camera.lookAt(target);
  }

  _updateHUD(reference) {
    const stateLabel = this.state.toUpperCase();
    const gateText = `Gate: ${this.gateIndex + 1} / ${this.track.length}`;
    const lapText = `Lap: ${this._formatTime(this.raceTime)}`;
    const speed = this.trueState.velocity.length();
    const speedText = `Speed: ${speed.toFixed(1)} m/s`;
    const vioError = new THREE.Vector3().subVectors(this.trueState.position, this.estimatedState.position).length();
    const vioText = `VIO Pos Err: ${vioError.toFixed(2)} m`;

    this.stateLabel.textContent = `State: ${stateLabel}`;
    this.gateLabel.textContent = gateText;
    this.lapLabel.textContent = lapText;
    this.speedLabel.textContent = speedText;
    this.errorLabel.textContent = vioText;
  }

  _formatTime(t) {
    const minutes = Math.floor(t / 60);
    const seconds = Math.floor(t % 60);
    const millis = Math.floor((t - Math.floor(t)) * 1000)
      .toString()
      .padStart(3, '0');
    return `${minutes.toString().padStart(2, '0')}:${seconds
      .toString()
      .padStart(2, '0')}.${millis}`;
  }
}

// Export for module environments
if (typeof module !== 'undefined' && module.exports) {
  module.exports = { initDroneRaceDemo };
}
