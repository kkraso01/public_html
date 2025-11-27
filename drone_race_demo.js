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

    // Difficulty presets
    this.difficultyPresets = {
      EASY: { maxSpeed: 7, maxAccel: 10, vioDrift: 0.5, gateRadius: 3.0, latency: 0.02 },
      MEDIUM: { maxSpeed: 10, maxAccel: 15, vioDrift: 1.0, gateRadius: 2.5, latency: 0.035 },
      HARD: { maxSpeed: 18, maxAccel: 25, vioDrift: 1.8, gateRadius: 1.8, latency: 0.05 },
    };
    this.difficulty = 'MEDIUM';
    this.vioLatency = 0.0;

    // Race state
    this.state = 'idle';
    this.gateIndex = 0;
    this.gatesPassed = 0;
    this.raceTime = 0;
    this.countdown = 1.5;
    this.crashTimer = 0;
    this.replayMode = false;
    this.replayBuffer = [];
    this.replayDuration = 10;
    this.replayCursor = 0;
    this.replayPlaying = false;

    // Visual effects
    this.trailPoints = [];
    this.trailGeometry = null;
    this.trailLine = null;
    this.trailMaxTime = 1.8;
    this.trailSegments = 120;
    this.gateAnimations = [];
    this.crashLabel = null;
    this.shakeSeed = Math.random() * 1000;
    this.shakeTime = 0;

    // FPV
    this.fpvEnabled = true;
    this.fpvCamera = null;
    this.fpvRenderer = null;
    this.fpvCanvas = null;
    this.fpvFrameTimer = 0;

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
    this._initTrail();
    this._initFPV();
    this._resetDrone();
    this._applyDifficulty(this.difficulty);
  }

  _initStyles() {
    if (document.getElementById('drone-race-demo-styles')) return;
    const style = document.createElement('style');
    style.id = 'drone-race-demo-styles';
    style.textContent = `
      .drone-race-container { position: relative; overflow: hidden; background: linear-gradient(180deg, #0f172a 0%, #0b1020 100%); }
      
      .drone-race-overlay { position: absolute; top: 10px; left: 10px; padding: 10px 12px; background: rgba(10, 14, 26, 0.75); backdrop-filter: blur(8px); color: #e5e7eb; font-family: 'Fira Code', 'SFMono-Regular', Consolas, 'Liberation Mono', Menlo, monospace; border: 1px solid rgba(129, 140, 248, 0.25); border-radius: 8px; box-shadow: 0 10px 30px rgba(0,0,0,0.35); max-width: 140px; }
      
      .drone-race-overlay h4 { margin: 0 0 6px 0; font-size: 13px; letter-spacing: 0.04em; color: #a5b4fc; font-weight: 600; }
      
      .drone-race-overlay .stat { font-size: 12px; line-height: 1.4; color: #cbd5e1; }
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
    this.columns = [];
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
      this.columns.push(col);
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
    const desiredSpeed = this.options.highQuality ? 8.0 : 6.5;
    const velocities = points.map((_, i) => {
      const prev = points[(i - 1 + points.length) % points.length];
      const next = points[(i + 1) % points.length];
      return new THREE.Vector3().subVectors(next, prev).multiplyScalar(0.5);
    });
    const zero = new THREE.Vector3();
    let totalTime = 0;

    for (let i = 0; i < points.length; i++) {
      const p0 = points[i];
      const p1 = points[(i + 1) % points.length];
      const v0 = velocities[i];
      const v1 = velocities[(i + 1) % velocities.length];
      const a0 = zero.clone();
      const a1 = zero.clone();
      const j0 = zero.clone();
      const j1 = zero.clone();
      const length = new THREE.Vector3().subVectors(p1, p0).length();
      const duration = Math.max(0.6, length / desiredSpeed);
      totalTime += duration;
      const coeffs = this._computeMinSnapCoeffs(p0, p1, v0, v1, a0, a1, j0, j1, duration);
      const steps = 40;
      for (let s = 0; s < steps; s++) {
        const t = (s / steps) * duration;
        const pos = this._evalPoly(coeffs, t);
        const vel = this._evalPolyDerivative(coeffs, t, 1);
        const dir = vel.lengthSq() > 1e-6 ? vel.clone().normalize() : new THREE.Vector3(1, 0, 0);
        samples.push({ t: totalTime - duration + t, position: pos, direction: dir });
      }
    }

    // Close loop
    const first = samples[0];
    samples.push({ t: totalTime, position: first.position.clone(), direction: first.direction.clone() });
    return { samples, totalTime };
  }

  _computeMinSnapCoeffs(p0, p1, v0, v1, a0, a1, j0, j1, T) {
    const A = [
      [1, 0, 0, 0, 0, 0, 0, 0],
      [0, 1, 0, 0, 0, 0, 0, 0],
      [0, 0, 2, 0, 0, 0, 0, 0],
      [0, 0, 0, 6, 0, 0, 0, 0],
      [1, T, Math.pow(T, 2), Math.pow(T, 3), Math.pow(T, 4), Math.pow(T, 5), Math.pow(T, 6), Math.pow(T, 7)],
      [0, 1, 2 * T, 3 * Math.pow(T, 2), 4 * Math.pow(T, 3), 5 * Math.pow(T, 4), 6 * Math.pow(T, 5), 7 * Math.pow(T, 6)],
      [0, 0, 2, 6 * T, 12 * Math.pow(T, 2), 20 * Math.pow(T, 3), 30 * Math.pow(T, 4), 42 * Math.pow(T, 5)],
      [0, 0, 0, 6, 24 * T, 60 * Math.pow(T, 2), 120 * Math.pow(T, 3), 210 * Math.pow(T, 4)],
    ];

    const solveAxis = (start, end, vs, ve, as, ae, js, je) => {
      const b = [start, vs, as, js, end, ve, ae, je];
      const m = A.map((row, r) => row.map((c) => c));
      const x = b.slice();
      for (let i = 0; i < 8; i++) {
        // pivot
        let pivot = m[i][i];
        if (Math.abs(pivot) < 1e-8) pivot = 1e-8;
        for (let j = i; j < 8; j++) m[i][j] /= pivot;
        x[i] /= pivot;
        for (let r = 0; r < 8; r++) {
          if (r === i) continue;
          const factor = m[r][i];
          for (let c = i; c < 8; c++) m[r][c] -= factor * m[i][c];
          x[r] -= factor * x[i];
        }
      }
      return x;
    };

    const cx = solveAxis(p0.x, p1.x, v0.x, v1.x, a0.x, a1.x, j0.x, j1.x);
    const cy = solveAxis(p0.y, p1.y, v0.y, v1.y, a0.y, a1.y, j0.y, j1.y);
    const cz = solveAxis(p0.z, p1.z, v0.z, v1.z, a0.z, a1.z, j0.z, j1.z);
    return { x: cx, y: cy, z: cz, T };
  }

  _evalPoly(coeffs, t) {
    const evalAxis = (c) =>
      c[0] + c[1] * t + c[2] * t ** 2 + c[3] * t ** 3 + c[4] * t ** 4 + c[5] * t ** 5 + c[6] * t ** 6 + c[7] * t ** 7;
    return new THREE.Vector3(evalAxis(coeffs.x), evalAxis(coeffs.y), evalAxis(coeffs.z));
  }

  _evalPolyDerivative(coeffs, t, order) {
    const evalAxis = (c) => {
      if (order === 1)
        return c[1] + 2 * c[2] * t + 3 * c[3] * t ** 2 + 4 * c[4] * t ** 3 + 5 * c[5] * t ** 4 + 6 * c[6] * t ** 5 + 7 * c[7] * t ** 6;
      if (order === 2)
        return 2 * c[2] + 6 * c[3] * t + 12 * c[4] * t ** 2 + 20 * c[5] * t ** 3 + 30 * c[6] * t ** 4 + 42 * c[7] * t ** 5;
      return 0;
    };
    return new THREE.Vector3(evalAxis(coeffs.x), evalAxis(coeffs.y), evalAxis(coeffs.z));
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
      this.gateAnimations.push({ intensity: 0, scale: 1, burstTime: 0 });
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

    // Simple rotor disks with correct rotation pairs
    // Front-left (0) & back-right (3) spin one direction
    // Front-right (1) & back-left (2) spin opposite direction
    const rotorGeo = new THREE.CircleGeometry(0.15, 16);
    const rotorMat = new THREE.MeshBasicMaterial({ color: 0x38bdf8, transparent: true, opacity: 0.6 });
    const rotorOffsets = [
      { pos: [0.35, 0, 0.35], spinDir: 1 },   // Front-left: CCW
      { pos: [-0.35, 0, 0.35], spinDir: -1 }, // Front-right: CW
      { pos: [-0.35, 0, -0.35], spinDir: 1 }, // Back-left: CCW
      { pos: [0.35, 0, -0.35], spinDir: -1 }, // Back-right: CW
    ];
    this.rotors = [];
    rotorOffsets.forEach(({ pos: [x, y, z], spinDir }) => {
      const rotor = new THREE.Mesh(rotorGeo, rotorMat);
      rotor.rotation.x = -Math.PI / 2;
      rotor.position.set(x, 0.12, z);
      rotor.userData.spinDir = spinDir;
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

    this.container.appendChild(overlay);

    this.stateLabel = overlay.querySelector('#drone-race-state');
    this.gateLabel = overlay.querySelector('#drone-race-gate');
    this.lapLabel = overlay.querySelector('#drone-race-lap');
    this.speedLabel = overlay.querySelector('#drone-race-speed');
    this.errorLabel = overlay.querySelector('#drone-race-error');

    const crash = document.createElement('div');
    crash.style.cssText = 'margin-top:6px;font-size:13px;font-weight:700;color:#f87171;display:none;';
    crash.textContent = 'CRASH';
    overlay.appendChild(crash);
    this.crashLabel = crash;

    const difficultyRow = document.createElement('div');
    difficultyRow.className = 'stat';
    difficultyRow.innerHTML = 'Difficulty: <select id="drone-difficulty"><option>EASY</option><option selected>MEDIUM</option><option>HARD</option></select>';
    overlay.appendChild(difficultyRow);

    const replayRow = document.createElement('div');
    replayRow.className = 'stat';
    replayRow.innerHTML = 'Replay: <button id="drone-replay-play">Play</button> <input id="drone-replay-slider" type="range" min="0" max="1" step="0.001" value="1" style="width:100px">';
    overlay.appendChild(replayRow);

    const fpvRow = document.createElement('div');
    fpvRow.className = 'stat';
    fpvRow.innerHTML = '<label><input type="checkbox" id="drone-fpv-toggle" checked> FPV Cam</label>';
    overlay.appendChild(fpvRow);

    // Attach to external control elements (will be in the HTML panel)
    this._attachExternalControls();
  }

  _attachExternalControls() {
    const toggleBtn = document.querySelector('#drone-race-toggle');
    const restartBtn = document.querySelector('#drone-race-restart');
    const speedSlider = document.querySelector('#drone-race-speed-slider');
    const pospSlider = document.querySelector('#drone-pid-posp');
    const velpSlider = document.querySelector('#drone-pid-velp');
    const yawpSlider = document.querySelector('#drone-pid-yawp');
    const altpSlider = document.querySelector('#drone-pid-altp');
    const maxaccelSlider = document.querySelector('#drone-maxaccel');
    const maxspeedSlider = document.querySelector('#drone-maxspeed');
    const dampingSlider = document.querySelector('#drone-damping');
    const resetBtn = document.querySelector('#drone-pid-reset');
    const difficultySelect = document.querySelector('#drone-difficulty');
    const replayPlay = document.querySelector('#drone-replay-play');
    const replaySlider = document.querySelector('#drone-replay-slider');
    const fpvToggle = document.querySelector('#drone-fpv-toggle');

    if (toggleBtn) {
      toggleBtn.addEventListener('click', () => {
        if (this._paused) {
          this._userPaused = false;
          this.resume();
          toggleBtn.textContent = 'Resume';
        } else {
          this._userPaused = true;
          this.pause();
          toggleBtn.textContent = 'Pause';
        }
      });
    }

    if (restartBtn) {
      restartBtn.addEventListener('click', () => {
        this.restart();
        if (toggleBtn) toggleBtn.textContent = 'Pause';
      });
    }

    if (speedSlider) {
      speedSlider.addEventListener('input', (e) => {
        this.timeScale = parseFloat(e.target.value);
      });
    }

    // Controller tuning
    if (pospSlider) {
      pospSlider.addEventListener('input', (e) => {
        const val = parseFloat(e.target.value);
        this.pid.posP = val;
        const display = document.querySelector('#drone-posp-val');
        if (display) display.textContent = val.toFixed(2);
      });
    }

    if (velpSlider) {
      velpSlider.addEventListener('input', (e) => {
        const val = parseFloat(e.target.value);
        this.pid.velP = val;
        const display = document.querySelector('#drone-velp-val');
        if (display) display.textContent = val.toFixed(2);
      });
    }

    if (yawpSlider) {
      yawpSlider.addEventListener('input', (e) => {
        const val = parseFloat(e.target.value);
        this.pid.yawP = val;
        const display = document.querySelector('#drone-yawp-val');
        if (display) display.textContent = val.toFixed(2);
      });
    }

    if (altpSlider) {
      altpSlider.addEventListener('input', (e) => {
        const val = parseFloat(e.target.value);
        this.pid.altitudeP = val;
        const display = document.querySelector('#drone-altp-val');
        if (display) display.textContent = val.toFixed(2);
      });
    }

    // Dynamics
    if (maxaccelSlider) {
      maxaccelSlider.addEventListener('input', (e) => {
        const val = parseFloat(e.target.value);
        this.maxAccel = val;
        const display = document.querySelector('#drone-maxaccel-val');
        if (display) display.textContent = val.toFixed(1);
      });
    }

    if (maxspeedSlider) {
      maxspeedSlider.addEventListener('input', (e) => {
        const val = parseFloat(e.target.value);
        this.maxSpeed = val;
        const display = document.querySelector('#drone-maxspeed-val');
        if (display) display.textContent = val.toFixed(1);
      });
    }

    if (dampingSlider) {
      dampingSlider.addEventListener('input', (e) => {
        const val = parseFloat(e.target.value);
        this.damping = val;
        const display = document.querySelector('#drone-damping-val');
        if (display) display.textContent = val.toFixed(2);
      });
    }

    // Reset button
    if (resetBtn) {
      resetBtn.addEventListener('click', () => {
        this.pid.posP = 2.5;
        this.pid.velP = 2.2;
        this.pid.yawP = 2.5;
        this.pid.altitudeP = 3.0;
        this.maxAccel = 18;
        this.maxSpeed = 12;
        this.damping = 0.98;

        if (pospSlider) pospSlider.value = 2.5;
        if (velpSlider) velpSlider.value = 2.2;
        if (yawpSlider) yawpSlider.value = 2.5;
        if (altpSlider) altpSlider.value = 3.0;
        if (maxaccelSlider) maxaccelSlider.value = 18;
        if (maxspeedSlider) maxspeedSlider.value = 12;
        if (dampingSlider) dampingSlider.value = 0.98;

        document.querySelector('#drone-posp-val').textContent = '2.50';
        document.querySelector('#drone-velp-val').textContent = '2.20';
        document.querySelector('#drone-yawp-val').textContent = '2.50';
        document.querySelector('#drone-altp-val').textContent = '3.00';
        document.querySelector('#drone-maxaccel-val').textContent = '18.0';
        document.querySelector('#drone-maxspeed-val').textContent = '12.0';
        document.querySelector('#drone-damping-val').textContent = '0.98';
      });
    }

    if (difficultySelect) {
      difficultySelect.addEventListener('change', (e) => {
        this._applyDifficulty(e.target.value);
        this.restart();
      });
    }

    if (replayPlay && replaySlider) {
      replayPlay.addEventListener('click', () => {
        this.replayPlaying = !this.replayPlaying;
        replayPlay.textContent = this.replayPlaying ? 'Pause' : 'Play';
      });
      replaySlider.addEventListener('input', (e) => {
        this.replayCursor = parseFloat(e.target.value) * this.replayDuration;
        this.replayPlaying = false;
        replayPlay.textContent = 'Play';
        this._applyReplayState();
      });
    }

    if (fpvToggle) {
      fpvToggle.addEventListener('change', (e) => {
        this.fpvEnabled = e.target.checked;
        if (this.fpvCanvas) this.fpvCanvas.style.display = this.fpvEnabled ? 'block' : 'none';
      });
    }
  }

  _applyDifficulty(level) {
    this.difficulty = level;
    const preset = this.difficultyPresets[level] || this.difficultyPresets.MEDIUM;
    this.maxSpeed = preset.maxSpeed;
    this.maxAccel = preset.maxAccel;
    this.vioDrift = preset.vioDrift;
    this.vioLatency = preset.latency;
    this.track.forEach((gate, i) => {
      gate.radius = preset.gateRadius;
      if (this.gateMeshes[i]) {
        const scale = (gate.radius / 2.5) * 1.0;
        this.gateMeshes[i].frame.scale.set(scale, scale, scale);
      }
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
    this.crashTimer = 0;
    this.replayMode = false;
    this.replayCursor = 0;
    this.replayPlaying = false;
    this.crashLabel.style.display = 'none';
    this._resetTrail();
    this._lastGateSign = null;
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
    this._updateTrail(dt);
    if (this.state === 'racing') {
      this.raceTime += dt;
    }

    if (this.state === 'crashed') {
      this.crashTimer += dt;
      this._applyCrashPhysics(dt);
      if (this.crashTimer > 0.4 && !this.replayMode) this._enterReplay(true);
      if (this.replayMode) this._applyReplayState();
      if (this.crashTimer > 2) {
        this.restart();
      }
      this._updateCamera(dt, this._getReferenceAtTime(this.raceTime));
      this._updateHUD(this._getReferenceAtTime(this.raceTime));
      return;
    }

    if (this.replayMode) {
      if (this.replayPlaying) {
        this.replayCursor += dt;
        if (this.replayCursor > this.replayDuration) this.replayCursor = this.replayDuration;
        const slider = document.querySelector('#drone-replay-slider');
        if (slider) slider.value = this.replayCursor / this.replayDuration;
        if (this.replayCursor >= this.replayDuration) this.replayPlaying = false;
      }
      this._applyReplayState();
      const reference = this._getReferenceAtTime(this.raceTime);
      this._updateCamera(dt, reference);
      this._updateHUD(reference);
      return;
    }

    const reference = this._getReferenceAtTime(this.raceTime);
    const controlOutput = this._updateControl(reference, dt);
    this._updatePhysics(controlOutput, dt);
    this._updateVIO(dt);
    this._updateGateCrossing();
    if (this._checkCrashConditions(reference)) {
      this._handleCrash('deviation');
    }
    this._updateCamera(dt, reference);
    this._updateHUD(reference);
    this._updateGateAnimations(dt);
    this._updateReplayBuffer(dt);
    this._renderFPV(dt);
  }

  _updateRaceState(dt) {
    if (this.state === 'countdown') {
      this.countdown -= dt;
      if (this.countdown <= 0) {
        this.state = 'racing';
        this.raceTime = 0;
      }
    } else if (this.state === 'finished') {
      this._enterReplay();
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
    if (this.state === 'crashed' || this.replayMode) return;
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

    this._updateCameraShake(dt);

    // Spin rotors visually with correct opposing directions
    this.rotors.forEach((rotor) => {
      const spinDir = rotor.userData.spinDir || 1;
      const spinSpeed = spinDir * 30 * dt;
      rotor.material.opacity = 0.45 + 0.15 * Math.random();
      rotor.rotation.z += spinSpeed;
    });
  }

  _updateVIO(dt) {
    // Random walk bias to emulate VIO drift
    const biasStep = new THREE.Vector3(
      (Math.random() - 0.5) * 0.01 * (this.vioDrift || 1),
      (Math.random() - 0.5) * 0.01 * (this.vioDrift || 1),
      (Math.random() - 0.5) * 0.01 * (this.vioDrift || 1),
    );
    this.vioBias.add(biasStep.multiplyScalar(dt * 4));

    const noise = () => (Math.random() - 0.5) * 0.03;
    const latencyOffset = this.trueState.velocity.clone().multiplyScalar(-(this.vioLatency || 0));
    this.estimatedState.position.copy(this.trueState.position).add(latencyOffset).add(this.vioBias).add(new THREE.Vector3(noise(), noise(), noise()));
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
        this._triggerGateAnimation((this.gateIndex + this.track.length - 1) % this.track.length);
        if (this.gateIndex === 0) {
          this.state = 'finished';
        }
      }
      this._lastGateSign = signNow;
    } else if (Math.abs(distToPlane) < 0.2 && !withinRadius) {
      this._handleCrash('missed gate');
    }
  }

  _updateCamera(dt, reference) {
    // Third-person chase camera with look-ahead
    const lookAhead = reference.direction.clone().multiplyScalar(4);
    const desiredPos = this.trueState.position.clone().add(new THREE.Vector3(0, 2, 0)).sub(reference.direction.clone().multiplyScalar(6));
    this.camera.position.lerp(desiredPos, 0.08);
    const shake = this._getCameraShake();
    this.camera.position.add(shake);
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

    this.stateLabel.textContent = `State: ${this.replayMode ? 'REPLAY MODE' : stateLabel}`;
    this.gateLabel.textContent = gateText;
    this.lapLabel.textContent = lapText;
    this.speedLabel.textContent = speedText;
    this.errorLabel.textContent = vioText;
    this.crashLabel.style.display = this.state === 'crashed' ? 'block' : 'none';
  }

  _initTrail() {
    this.trailGeometry = new THREE.BufferGeometry();
    const positions = new Float32Array(this.trailSegments * 3);
    this.trailGeometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
    const material = new THREE.LineBasicMaterial({ color: 0x93c5fd, transparent: true, opacity: 0.5, linewidth: 3 });
    this.trailLine = new THREE.Line(this.trailGeometry, material);
    this.trailLine.frustumCulled = false;
    this.scene.add(this.trailLine);
  }

  _resetTrail() {
    this.trailPoints.length = 0;
    if (this.trailGeometry) {
      const pos = this.trailGeometry.attributes.position.array;
      pos.fill(0);
      this.trailGeometry.attributes.position.needsUpdate = true;
    }
  }

  _updateTrail(dt) {
    if (!this.trailGeometry) return;
    const now = performance.now() / 1000;
    this.trailPoints.push({ time: now, position: this.trueState.position.clone() });
    while (this.trailPoints.length > 1 && now - this.trailPoints[0].time > this.trailMaxTime) {
      this.trailPoints.shift();
    }

    const positions = this.trailGeometry.attributes.position.array;
    const count = Math.min(this.trailPoints.length, this.trailSegments);
    for (let i = 0; i < count; i++) {
      const p = this.trailPoints[this.trailPoints.length - count + i];
      positions[i * 3] = p.position.x;
      positions[i * 3 + 1] = p.position.y;
      positions[i * 3 + 2] = p.position.z;
    }
    this.trailGeometry.setDrawRange(0, count);
    this.trailGeometry.attributes.position.needsUpdate = true;
    if (this.trailLine && this.trailPoints.length > 1) {
      const speed = this.trueState.velocity.length();
      this.trailLine.material.opacity = THREE.MathUtils.clamp(0.25 + speed * 0.03, 0.25, 0.85);
      this.trailLine.material.color.setHSL(0.6, 0.7, 0.6 + Math.min(speed * 0.01, 0.2));
    }
  }

  _triggerGateAnimation(index) {
    if (!this.gateAnimations[index]) return;
    const anim = this.gateAnimations[index];
    anim.intensity = 1.5;
    anim.scale = 1.2;
    anim.burstTime = 0.35;
  }

  _updateGateAnimations(dt) {
    this.gateAnimations.forEach((anim, i) => {
      if (!anim) return;
      anim.intensity = Math.max(0, anim.intensity - dt * 2.5);
      anim.scale += (1 - anim.scale) * dt * 5;
      const frame = this.gateMeshes[i].frame;
      frame.material.emissiveIntensity = 0.25 + anim.intensity;
      frame.scale.setScalar(anim.scale * ((this.track[i].radius / 2.5) * 1.0));
      if (anim.burstTime > 0) {
        this._emitGateBurst(i);
        anim.burstTime -= dt;
      }
    });
  }

  _emitGateBurst(i) {
    const frame = this.gateMeshes[i].frame;
    if (!frame.userData.burstMesh) {
      const geo = new THREE.RingGeometry(1.2, 1.35, 32);
      const mat = new THREE.MeshBasicMaterial({ color: frame.material.color, transparent: true, opacity: 0.7, side: THREE.DoubleSide });
      const mesh = new THREE.Mesh(geo, mat);
      frame.add(mesh);
      frame.userData.burstMesh = mesh;
    }
    const mesh = frame.userData.burstMesh;
    mesh.visible = true;
    mesh.scale.setScalar(mesh.scale.x * 1.05 + 0.02);
    mesh.material.opacity = Math.max(0, mesh.material.opacity - 0.04);
    if (mesh.material.opacity <= 0.01) {
      mesh.material.opacity = 0.7;
      mesh.scale.setScalar(1);
      mesh.visible = false;
    }
  }

  _handleCrash() {
    if (this.state === 'crashed') return;
    this.state = 'crashed';
    this.crashTimer = 0;
    this.replayMode = false;
    this.droneMesh.userData.spin = new THREE.Vector3((Math.random() - 0.5) * 8, (Math.random() - 0.5) * 8, (Math.random() - 0.5) * 8);
    this.trueState.angularVelocity.copy(this.droneMesh.userData.spin);
    this._enterReplay(true);
  }

  _applyCrashPhysics(dt) {
    this.trueState.velocity.add(this.gravity.clone().multiplyScalar(dt));
    this.trueState.position.add(this.trueState.velocity.clone().multiplyScalar(dt));
    this.trueState.orientation.x += this.trueState.angularVelocity.x * dt;
    this.trueState.orientation.y += this.trueState.angularVelocity.y * dt;
    this.trueState.orientation.z += this.trueState.angularVelocity.z * dt;
    if (this.trueState.position.y < 0.1) {
      this.trueState.position.y = 0.1;
      this.trueState.velocity.multiplyScalar(0.7);
    }
    this.droneMesh.position.copy(this.trueState.position);
    this.droneMesh.rotation.copy(this.trueState.orientation);
  }

  _updateCameraShake(dt) {
    const speed = this.trueState.velocity.length();
    const accel = this.trueState.velocity.clone().sub(this._prevVel || new THREE.Vector3()).length() / Math.max(dt, 1e-3);
    this._prevVel = this.trueState.velocity.clone();
    let gateBoost = 0;
    if (this.track && this.track.length) {
      const gate = this.track[this.gateIndex % this.track.length];
      const toGate = new THREE.Vector3().subVectors(this.trueState.position, gate.position);
      gateBoost = THREE.MathUtils.clamp(1 - toGate.length() / 6, 0, 1) * 0.08;
    }
    this.shakeTime += dt;
    this.shakeIntensity = THREE.MathUtils.clamp(speed * 0.005 + accel * 0.001 + gateBoost, 0, this.state === 'crashed' ? 0.35 : 0.18);
  }

  _getCameraShake() {
    const n = (t) => {
      const x = Math.sin((t + this.shakeSeed) * 12.9898) * 43758.5453;
      return (x - Math.floor(x)) * 2 - 1;
    };
    const s = this.shakeIntensity || 0;
    return new THREE.Vector3(n(this.shakeTime * 0.9) * s, n(this.shakeTime * 1.1) * s * 0.6, n(this.shakeTime * 1.3) * s);
  }

  _checkCrashConditions(reference) {
    if (this.trueState.position.y < 0.1) return true;
    if (Math.abs(this.trueState.position.x) > 40 || Math.abs(this.trueState.position.z) > 40) return true;
    const deviation = new THREE.Vector3().subVectors(this.trueState.position, reference.position).length();
    if (deviation > 2) return true;
    for (let i = 0; i < (this.columns || []).length; i++) {
      const col = this.columns[i];
      const horizontal = new THREE.Vector3(this.trueState.position.x - col.position.x, 0, this.trueState.position.z - col.position.z).length();
      if (horizontal < 0.6 && this.trueState.position.y < col.position.y + 3) {
        return true;
      }
    }
    return false;
  }

  _enterReplay(force) {
    if (!force && this.state !== 'finished' && this.state !== 'crashed') return;
    this.replayMode = true;
    this.replayCursor = this.replayDuration;
    this.replayPlaying = false;
    const slider = document.querySelector('#drone-replay-slider');
    if (slider) slider.value = 1;
  }

  _updateReplayBuffer(dt) {
    const snapshot = {
      time: this.raceTime,
      position: this.trueState.position.clone(),
      velocity: this.trueState.velocity.clone(),
      orientation: this.trueState.orientation.clone(),
    };
    this.replayBuffer.push(snapshot);
    const cutoff = this.raceTime - this.replayDuration;
    while (this.replayBuffer.length && this.replayBuffer[0].time < cutoff) this.replayBuffer.shift();
  }

  _applyReplayState() {
    if (!this.replayBuffer.length) return;
    const startTime = this.replayBuffer[0].time;
    const targetTime = startTime + this.replayCursor;
    let i = 0;
    while (i < this.replayBuffer.length - 1 && this.replayBuffer[i + 1].time < targetTime) i++;
    const a = this.replayBuffer[i];
    const b = this.replayBuffer[Math.min(i + 1, this.replayBuffer.length - 1)];
    const span = b.time - a.time || 1;
    const alpha = THREE.MathUtils.clamp((targetTime - a.time) / span, 0, 1);
    this.trueState.position.lerpVectors(a.position, b.position, alpha);
    this.trueState.velocity.lerpVectors(a.velocity, b.velocity, alpha);
    this.trueState.orientation.x = THREE.MathUtils.lerp(a.orientation.x, b.orientation.x, alpha);
    this.trueState.orientation.y = THREE.MathUtils.lerp(a.orientation.y, b.orientation.y, alpha);
    this.trueState.orientation.z = THREE.MathUtils.lerp(a.orientation.z, b.orientation.z, alpha);
    this.droneMesh.position.copy(this.trueState.position);
    this.droneMesh.rotation.copy(this.trueState.orientation);
  }

  _initFPV() {
    this.fpvCamera = new THREE.PerspectiveCamera(75, 4 / 3, 0.05, 200);
    this.fpvCanvas = document.createElement('canvas');
    this.fpvCanvas.width = 200;
    this.fpvCanvas.height = 150;
    this.fpvCanvas.style.cssText = 'position:absolute; bottom:10px; right:10px; border:1px solid rgba(255,255,255,0.2); opacity:0.9;';
    this.container.appendChild(this.fpvCanvas);
    this.fpvRenderer = new THREE.WebGLRenderer({ canvas: this.fpvCanvas, antialias: false, alpha: true });
    this.fpvRenderer.setSize(200, 150);
    this.fpvRenderer.setPixelRatio(1);
  }

  _renderFPV(dt) {
    if (!this.fpvEnabled || !this.fpvRenderer) return;
    this.fpvFrameTimer += dt;
    if (this.fpvFrameTimer < 1 / 30) return;
    this.fpvFrameTimer = 0;
    this.fpvCamera.position.copy(this.trueState.position);
    const forward = new THREE.Vector3(0, 0, 1).applyEuler(this.trueState.orientation);
    this.fpvCamera.position.add(forward.clone().multiplyScalar(0.4));
    this.fpvCamera.setRotationFromEuler(this.trueState.orientation);
    this.fpvRenderer.render(this.scene, this.fpvCamera);
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
