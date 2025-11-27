// Autonomous drone race demo with research-inspired physics + control stack.
// Exposes initDroneRaceDemo(container, options) -> { pause, resume, restart, setPausedFromVisibility }

import { Quadrotor } from '../../core/physics/drone_core_physics.js';
import { GeometricController } from '../../core/ai/drone_control.js';

class ReferenceTrajectory {
  // Min-snap placeholder implemented as Catmull-Rom splines but structured so a true solver can drop in later.
  constructor(waypoints, totalTime) {
    this.points = waypoints.map((p) => p.clone());
    this.totalTime = totalTime;
    this.segmentTime = totalTime / Math.max(1, this.points.length - 1);
  }

  _catmullRom(p0, p1, p2, p3, t) {
    const t2 = t * t;
    const t3 = t2 * t;
    return p1
      .clone()
      .multiplyScalar(2)
      .add(p2.clone().sub(p0).multiplyScalar(t))
      .add(p0.clone().multiplyScalar(2).sub(p1.clone().multiplyScalar(5)).add(p2.clone().multiplyScalar(4)).sub(p3).multiplyScalar(t2))
      .add(p0.clone().negate().add(p1.clone().multiplyScalar(3)).sub(p2.clone().multiplyScalar(3)).add(p3).multiplyScalar(t3))
      .multiplyScalar(0.5);
  }

  sample(time) {
    const clamped = Math.min(Math.max(time, 0), this.totalTime * 0.999);
    const idx = Math.floor(clamped / this.segmentTime);
    const localT = (clamped - idx * this.segmentTime) / this.segmentTime;

    const i0 = Math.max(0, idx - 1);
    const i1 = Math.max(0, idx);
    const i2 = Math.min(this.points.length - 1, idx + 1);
    const i3 = Math.min(this.points.length - 1, idx + 2);

    const p = this._catmullRom(this.points[i0], this.points[i1], this.points[i2], this.points[i3], localT);
    // Numerical derivatives for velocity/acceleration
    const eps = 0.01;
    const pFwd = this._catmullRom(this.points[i0], this.points[i1], this.points[i2], this.points[i3], Math.min(1, localT + eps));
    const pBack = this._catmullRom(this.points[i0], this.points[i1], this.points[i2], this.points[i3], Math.max(0, localT - eps));
    const vel = pFwd.clone().sub(pBack).multiplyScalar(1 / (2 * eps * this.segmentTime));
    const acc = pFwd.clone().add(pBack).sub(p.multiplyScalar(2)).multiplyScalar(1 / (eps * eps * this.segmentTime * this.segmentTime));

    const yaw = vel.lengthSq() > 1e-6 ? Math.atan2(vel.x, vel.z) : 0;
    return { position: p, velocity: vel, acceleration: acc, yaw };
  }
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
        fpvEnabled: true,
        width: container.clientWidth || 800,
        height: container.clientHeight || 450,
      },
      options,
    );

    this.physicsRate = 120;
    this.timeScale = 1.0;
    this.state = 'COUNTDOWN';
    this.userPaused = false;
    this.visibilityPaused = false;
    this.lastFrame = null;
    this.accumulator = 0;
    this._frameReq = null;

    this._initScene();
    this._initHUD();
    this._initDrone();
    this._initTrack();
    this.restart();
  }

  _initScene() {
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x0a1020);
    this.camera = new THREE.PerspectiveCamera(60, this.options.width / this.options.height, 0.1, 500);
    this.camera.position.set(0, 2.5, 8);

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

    const floor = new THREE.Mesh(
      new THREE.PlaneGeometry(120, 120),
      new THREE.MeshStandardMaterial({ color: 0x0f172a, roughness: 0.6, metalness: 0.05 }),
    );
    floor.rotation.x = -Math.PI / 2;
    floor.receiveShadow = true;
    this.scene.add(floor);

    this.trailGeometry = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3()]);
    this.trailLine = new THREE.Line(
      this.trailGeometry,
      new THREE.LineBasicMaterial({ color: 0x8be9fd, transparent: true, opacity: 0.55 }),
    );
    this.scene.add(this.trailLine);
  }

  _initHUD() {
    this.overlay = document.createElement('div');
    this.overlay.style.cssText = 'position:absolute; top:8px; left:8px; background:rgba(8,12,24,0.8); color:#e2e8f0; padding:10px; font-family:"Fira Code", monospace; border:1px solid rgba(99,102,241,0.4); border-radius:8px; backdrop-filter: blur(8px); z-index:5;';
    this.overlay.innerHTML = `
      <div style="font-size:12px; color:#a5b4fc; margin-bottom:4px;">Autonomous Drone Race</div>
      <div id="raceState" style="font-size:12px; margin-bottom:4px;">State: COUNTDOWN</div>
      <div id="raceTime" style="font-size:12px;">t = 0.00 s</div>
      <div id="raceGate" style="font-size:12px;">Gate: 0 / 0</div>
      <div id="raceSpeed" style="font-size:12px;">Speed: 0.0 m/s</div>
    `;
    this.container.style.position = 'relative';
    this.container.appendChild(this.overlay);
  }

  _initDrone() {
    this.drone = new Quadrotor();
    this.controller = new GeometricController({ mass: this.drone.params.mass });

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
    this.gates = [];
    const gateMaterial = new THREE.MeshStandardMaterial({ color: 0x8b5cf6, emissive: 0x7c3aed, emissiveIntensity: 1.5, side: THREE.DoubleSide });
    const radius = 1.1;
    const gatePositions = [];
    for (let i = 0; i < 7; i++) {
      const angle = (i / 7) * Math.PI * 2;
      const x = Math.cos(angle) * 8;
      const z = Math.sin(angle) * 8;
      const y = 2 + Math.sin(angle * 2) * 1.2;
      gatePositions.push(new THREE.Vector3(x, y, z));
    }
    this.waypoints = [new THREE.Vector3(0, 2, 10), ...gatePositions, new THREE.Vector3(0, 2, 10)];
    this.trajectory = new ReferenceTrajectory(this.waypoints, 22);

    gatePositions.forEach((pos) => {
      const gate = new THREE.Mesh(new THREE.TorusGeometry(radius, 0.05, 12, 60), gateMaterial);
      gate.position.copy(pos);
      gate.lookAt(new THREE.Vector3());
      gate.castShadow = true;
      this.scene.add(gate);
      this.gates.push(gate);
    });
  }

  restart() {
    this.raceTime = 0;
    this.state = 'COUNTDOWN';
    this.countdown = 2.0;
    this.gateIndex = 0;
    this.lastFrame = null;
    this.accumulator = 0;
    this.positionsHistory = [];

    this.drone.reset({
      position: this.waypoints[0],
      velocity: new THREE.Vector3(),
      quaternion: new THREE.Quaternion(),
      omega: new THREE.Vector3(),
    });
    this.droneMesh.position.copy(this.waypoints[0]);
    this.droneMesh.quaternion.copy(new THREE.Quaternion());
  }

  pause(user = false) {
    this.userPaused = user ? true : this.userPaused;
    this._paused = true;
    if (this._frameReq) {
      cancelAnimationFrame(this._frameReq);
      this._frameReq = null;
    }
  }

  resume(user = false) {
    this.userPaused = user ? false : this.userPaused;
    if (!this.visibilityPaused) this._paused = false;
    this.lastFrame = null;
    this._raf();
  }

  setPausedFromVisibility(visible) {
    this.visibilityPaused = !visible;
    if (visible && !this.userPaused) this.resume();
    else this.pause();
  }

  start() {
    this._raf();
  }

  _raf(timestamp) {
    if (this._paused) return;
    this._frameReq = requestAnimationFrame((t) => this._raf(t));
    if (!this.lastFrame) {
      this.lastFrame = timestamp;
      return;
    }
    const dt = ((timestamp - this.lastFrame) / 1000) * this.timeScale;
    this.lastFrame = timestamp;
    this.accumulator += dt;
    const h = 1 / this.physicsRate;
    while (this.accumulator >= h) {
      this._stepPhysics(h);
      this.accumulator -= h;
    }
    this._render();
  }

  _stepPhysics(dt) {
    if (this.state === 'COUNTDOWN') {
      this.countdown -= dt;
      if (this.countdown <= 0) {
        this.state = 'RACING';
        this.raceTime = 0;
      }
      return;
    }

    if (this.state === 'FINISHED') return;

    this.raceTime += dt;
    const desired = this.trajectory.sample(this.raceTime);
    const control = this.controller.computeControl(this.drone.getState(), desired);
    this.drone.step(control, dt);

    const state = this.drone.getState();
    this.droneMesh.position.copy(state.position);
    this.droneMesh.quaternion.copy(state.quaternion);

    this.positionsHistory.push({ t: this.raceTime, p: state.position.clone() });
    this.positionsHistory = this.positionsHistory.filter((s) => this.raceTime - s.t < 2.5);

    if (this.gateIndex < this.gates.length) {
      const dist = state.position.distanceTo(this.gates[this.gateIndex].position);
      if (dist < 1.0) this.gateIndex++;
    } else if (this.raceTime > this.trajectory.totalTime) {
      this.state = 'FINISHED';
    }
  }

  _render() {
    // Update trail
    const pts = this.positionsHistory.map((s) => s.p);
    if (pts.length >= 2) {
      this.trailGeometry.setFromPoints(pts);
      const opacities = pts.map((p, i) => (i / pts.length));
      this.trailLine.material.opacity = 0.2 + 0.5 * (pts.length / 60);
    }

    // Camera chase
    const target = this.droneMesh.position;
    const forward = new THREE.Vector3(0, 0, -1).applyQuaternion(this.droneMesh.quaternion);
    const camPos = target.clone().add(forward.clone().multiplyScalar(-4)).add(new THREE.Vector3(0, 1.2, 0));
    this.camera.position.lerp(camPos, 0.08);
    this.camera.lookAt(target);

    this.renderer.render(this.scene, this.camera);

    const speed = this.drone.getState().velocity.length();
    this.overlay.querySelector('#raceState').textContent = `State: ${this.state}`;
    this.overlay.querySelector('#raceTime').textContent = `t = ${this.raceTime.toFixed(2)} s`;
    this.overlay.querySelector('#raceGate').textContent = `Gate: ${Math.min(this.gateIndex, this.gates.length)} / ${this.gates.length}`;
    this.overlay.querySelector('#raceSpeed').textContent = `Speed: ${speed.toFixed(1)} m/s`;
  }

  destroy() {
    this.pause(true);
    if (this._frameReq) {
      cancelAnimationFrame(this._frameReq);
      this._frameReq = null;
    }
    if (this.overlay?.parentNode === this.container) this.container.removeChild(this.overlay);
    if (this.renderer?.domElement?.parentNode === this.container) {
      this.container.removeChild(this.renderer.domElement);
    }
  }
}
