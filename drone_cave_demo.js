// Autonomous cave exploration demo with LiDAR mapping and frontier-based planning.
// Exposes initDroneCaveDemo(container, options) with pause/resume/restart/setPausedFromVisibility.

import { Quadrotor } from './drone_core_physics.js';
import { GeometricController } from './drone_control.js';

class ReferenceTrajectory {
  constructor(points, totalTime) {
    this.points = points.map((p) => p.clone());
    this.totalTime = totalTime;
    this.segmentTime = totalTime / Math.max(1, this.points.length - 1);
  }

  sample(time) {
    const t = Math.min(Math.max(time, 0), this.totalTime);
    const idx = Math.min(this.points.length - 2, Math.floor(t / this.segmentTime));
    const local = (t - idx * this.segmentTime) / this.segmentTime;
    const p0 = this.points[idx];
    const p1 = this.points[idx + 1];
    const position = p0.clone().lerp(p1, local);
    const velocity = p1.clone().sub(p0).multiplyScalar(1 / this.segmentTime);
    const acceleration = new THREE.Vector3();
    const yaw = velocity.lengthSq() > 1e-6 ? Math.atan2(velocity.x, velocity.z) : 0;
    return { position, velocity, acceleration, yaw };
  }
}

class OccupancyGrid {
  constructor(resolution, size) {
    this.resolution = resolution; // meters per cell
    this.size = size; // cells per side
    this.origin = new THREE.Vector2(-((size * resolution) / 2), -((size * resolution) / 2));
    this.grid = new Float32Array(size * size).fill(0);
  }

  indexFromWorld(pos) {
    const x = Math.floor((pos.x - this.origin.x) / this.resolution);
    const y = Math.floor((pos.z - this.origin.y) / this.resolution);
    return { x, y, idx: y * this.size + x };
  }

  isInside(x, y) {
    return x >= 0 && x < this.size && y >= 0 && y < this.size;
  }

  updateCell(x, y, delta) {
    if (!this.isInside(x, y)) return;
    const idx = y * this.size + x;
    this.grid[idx] = Math.max(-4, Math.min(4, this.grid[idx] + delta));
  }

  value(x, y) {
    if (!this.isInside(x, y)) return 0;
    return this.grid[y * this.size + x];
  }

  occupancy() {
    let known = 0;
    let free = 0;
    for (let i = 0; i < this.grid.length; i++) {
      if (Math.abs(this.grid[i]) > 0.01) known++;
      if (this.grid[i] < -0.1) free++;
    }
    return { known, free, coverage: known / this.grid.length };
  }

  frontierCells() {
    const cells = [];
    for (let y = 1; y < this.size - 1; y++) {
      for (let x = 1; x < this.size - 1; x++) {
        const v = this.value(x, y);
        if (v < -0.2) {
          const unknownNbr =
            Math.abs(this.value(x + 1, y)) < 0.01 ||
            Math.abs(this.value(x - 1, y)) < 0.01 ||
            Math.abs(this.value(x, y + 1)) < 0.01 ||
            Math.abs(this.value(x, y - 1)) < 0.01;
          if (unknownNbr) cells.push({ x, y });
        }
      }
    }
    return cells;
  }
}

class LidarSimulator {
  constructor(scene, caveMesh) {
    this.scene = scene;
    this.caveMesh = caveMesh;
    this.raycaster = new THREE.Raycaster();
    this.numRays = 80;
    this.maxRange = 18;
  }

  scan(pose) {
    const hits = [];
    for (let i = 0; i < this.numRays; i++) {
      const theta = (i / this.numRays) * Math.PI * 2;
      const dir = new THREE.Vector3(Math.cos(theta), 0, Math.sin(theta)).applyQuaternion(pose.quaternion);
      this.raycaster.set(pose.position, dir);
      this.raycaster.far = this.maxRange;
      const res = this.raycaster.intersectObject(this.caveMesh, false);
      if (res.length > 0) {
        const d = res[0].distance + (Math.random() - 0.5) * 0.1;
        hits.push({ distance: d, direction: dir.clone(), point: res[0].point });
      }
    }
    return hits;
  }
}

export function initDroneCaveDemo(container, options = {}) {
  if (!container || typeof THREE === 'undefined') {
    console.warn('Cave demo requires a valid container and Three.js');
    return { pause() {}, resume() {}, restart() {}, setPausedFromVisibility() {} };
  }
  const demo = new DroneCaveDemo(container, options);
  demo.start();
  return {
    pause: () => demo.pause(true),
    resume: () => demo.resume(true),
    restart: () => demo.restart(),
    setPausedFromVisibility: (visible) => demo.setPausedFromVisibility(visible),
  };
}

class DroneCaveDemo {
  constructor(container, options) {
    this.container = container;
    this.options = Object.assign(
      {
        width: container.clientWidth || 800,
        height: container.clientHeight || 450,
        highQuality: true,
      },
      options,
    );

    this.physicsRate = 120;
    this.timeScale = 1;
    this.userPaused = false;
    this.visibilityPaused = false;
    this.lastFrame = null;
    this.accumulator = 0;

    this._initScene();
    this._initHUD();
    this._initDrone();
    this._initCave();
    this._initMapping();
    this.restart();
  }

  _initScene() {
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x05070d);
    this.camera = new THREE.PerspectiveCamera(65, this.options.width / this.options.height, 0.1, 200);
    this.camera.position.set(0, 2, 6);

    this.renderer = new THREE.WebGLRenderer({ antialias: !!this.options.highQuality });
    this.renderer.setSize(this.options.width, this.options.height);
    this.renderer.setPixelRatio(this.options.highQuality ? window.devicePixelRatio : 1);
    this.container.innerHTML = '';
    this.container.appendChild(this.renderer.domElement);

    const ambient = new THREE.AmbientLight(0x8899aa, 0.2);
    const fog = new THREE.FogExp2(0x06080f, 0.05);
    this.scene.add(ambient);
    this.scene.fog = fog;
  }

  _initHUD() {
    this.overlay = document.createElement('div');
    this.overlay.style.cssText = 'position:absolute; top:8px; right:8px; background:rgba(5,6,12,0.8); color:#e2e8f0; padding:10px; font-family:"Fira Code", monospace; border:1px solid rgba(34,197,94,0.5); border-radius:8px; backdrop-filter: blur(8px); z-index:5; text-align:right;';
    this.overlay.innerHTML = `
      <div style="font-size:12px; color:#34d399; margin-bottom:4px;">Cave Exploration</div>
      <div id="caveState" style="font-size:12px;">State: INIT</div>
      <div id="caveTime" style="font-size:12px;">t = 0.00 s</div>
      <div id="caveCoverage" style="font-size:12px;">Coverage: 0%</div>
      <div id="caveFrontiers" style="font-size:12px;">Frontiers: 0</div>
    `;
    this.container.style.position = 'relative';
    this.container.appendChild(this.overlay);

    this.minimap = document.createElement('canvas');
    this.minimap.width = 180;
    this.minimap.height = 180;
    this.minimap.style.cssText = 'position:absolute; bottom:8px; right:8px; border:1px solid rgba(34,197,94,0.6); background:#02040a;';
    this.container.appendChild(this.minimap);
  }

  _initDrone() {
    this.drone = new Quadrotor();
    this.controller = new GeometricController({ mass: this.drone.params.mass });

    const body = new THREE.Mesh(
      new THREE.ConeGeometry(0.09, 0.2, 10),
      new THREE.MeshStandardMaterial({ color: 0x22c55e, emissive: 0x16a34a, roughness: 0.3, metalness: 0.25 }),
    );
    body.rotation.x = Math.PI / 2;
    const light = new THREE.SpotLight(0x7dd3fc, 3, 12, Math.PI / 4, 0.3, 1);
    light.position.set(0, 0, 0);
    light.target.position.set(0, 0, -1);
    const rig = new THREE.Group();
    rig.add(body, light, light.target);
    this.droneMesh = rig;
    this.scene.add(this.droneMesh);
  }

  _initCave() {
    // Generate cave spline path
    const knots = [];
    for (let i = 0; i < 12; i++) {
      knots.push(new THREE.Vector3((Math.random() - 0.5) * 6, 1.5 + Math.sin(i * 0.5) * 0.4, -i * 4));
    }
    const curve = new THREE.CatmullRomCurve3(knots);
    const geometry = new THREE.TubeGeometry(curve, 200, 2.2, 12, false);
    geometry.verticesNeedUpdate = true;
    geometry.attributes.position.needsUpdate = true;
    // Add roughness with deterministic noise
    const posAttr = geometry.attributes.position;
    for (let i = 0; i < posAttr.count; i++) {
      const v = new THREE.Vector3().fromBufferAttribute(posAttr, i);
      const n = Math.sin(v.x * 0.7) * Math.sin(v.z * 0.5) * 0.15;
      v.addScaledVector(v.clone().normalize(), n);
      posAttr.setXYZ(i, v.x, v.y, v.z);
    }
    posAttr.needsUpdate = true;

    const material = new THREE.MeshStandardMaterial({
      color: 0x0f172a,
      emissive: 0x0b1f30,
      roughness: 0.9,
      metalness: 0.05,
      side: THREE.BackSide,
    });
    this.caveMesh = new THREE.Mesh(geometry, material);
    this.scene.add(this.caveMesh);

    this.artifacts = [];
    for (let i = 0; i < 4; i++) {
      const t = Math.random();
      const p = curve.getPoint(t);
      const color = new THREE.Color().setHSL(Math.random(), 0.8, 0.6);
      const obj = new THREE.Mesh(new THREE.SphereGeometry(0.2, 12, 12), new THREE.MeshStandardMaterial({ color, emissive: color }));
      obj.position.copy(p).add(new THREE.Vector3((Math.random() - 0.5) * 0.8, (Math.random() - 0.5) * 0.4, (Math.random() - 0.5) * 0.8));
      this.artifacts.push(obj);
      this.scene.add(obj);
    }

    this.lidar = new LidarSimulator(this.scene, this.caveMesh);
  }

  _initMapping() {
    this.grid = new OccupancyGrid(0.5, 120);
    this.pathWorld = [];
    this.currentPlan = null;
    this.lastLidar = 0;
    this.lastPlan = 0;
  }

  restart() {
    this.state = 'MAPPING';
    this.time = 0;
    this.lastFrame = null;
    this.accumulator = 0;
    this.drone.reset({
      position: new THREE.Vector3(0, 1.5, 2),
      velocity: new THREE.Vector3(),
      quaternion: new THREE.Quaternion(),
      omega: new THREE.Vector3(),
    });
    this.droneMesh.position.copy(this.drone.state.position);
    this.droneMesh.quaternion.copy(this.drone.state.quaternion);
    this.grid = new OccupancyGrid(0.5, 120);
    this.currentPlan = null;
    this.pathWorld = [];
  }

  pause(user = false) {
    this.userPaused = user ? true : this.userPaused;
    this._paused = true;
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
    requestAnimationFrame((t) => this._raf(t));
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

  _updateLidar(dt) {
    this.lastLidar += dt;
    if (this.lastLidar < 0.15) return;
    this.lastLidar = 0;

    const state = this.drone.getState();
    const hits = this.lidar.scan(state);
    hits.forEach((hit) => {
      const noisyPoint = hit.point.clone().add(new THREE.Vector3((Math.random() - 0.5) * 0.05, 0, (Math.random() - 0.5) * 0.05));
      const idx = this.grid.indexFromWorld(noisyPoint);
      const originIdx = this.grid.indexFromWorld(state.position);
      // Simple Bresenham-like carve free space
      const steps = 12;
      for (let i = 0; i <= steps; i++) {
        const ix = Math.floor(originIdx.x + (idx.x - originIdx.x) * (i / steps));
        const iy = Math.floor(originIdx.y + (idx.y - originIdx.y) * (i / steps));
        if (!this.grid.isInside(ix, iy)) break;
        if (i < steps) this.grid.updateCell(ix, iy, -0.35);
        else this.grid.updateCell(ix, iy, 0.8);
      }
    });
  }

  _planIfNeeded(dt) {
    this.lastPlan += dt;
    if (this.lastPlan < 1.0) return;
    this.lastPlan = 0;

    const frontiers = this.grid.frontierCells();
    if (frontiers.length === 0) return;

    const state = this.drone.getState();
    const current = this.grid.indexFromWorld(state.position);
    let best = null;
    let bestDist = Infinity;
    frontiers.forEach((f) => {
      const d = Math.hypot(f.x - current.x, f.y - current.y);
      if (d < bestDist) {
        best = f;
        bestDist = d;
      }
    });

    const path = this._astar(current, best);
    if (path.length > 0) {
      const pts = path.map((c) => new THREE.Vector3(
        this.grid.origin.x + c.x * this.grid.resolution,
        1.5,
        this.grid.origin.y + c.y * this.grid.resolution,
      ));
      this.currentPlan = {
        target: best,
        traj: new ReferenceTrajectory([state.position.clone(), ...pts], Math.max(3, pts.length * 0.4)),
        startTime: this.time,
      };
    }
  }

  _astar(start, goal) {
    const key = (c) => `${c.x},${c.y}`;
    const open = new Map();
    const closed = new Set();
    open.set(key(start), { c: start, g: 0, f: Math.hypot(goal.x - start.x, goal.y - start.y), parent: null });
    const dirs = [
      { x: 1, y: 0 },
      { x: -1, y: 0 },
      { x: 0, y: 1 },
      { x: 0, y: -1 },
    ];

    while (open.size > 0) {
      let bestKey = null;
      let bestNode = null;
      for (const [k, n] of open) {
        if (!bestNode || n.f < bestNode.f) {
          bestNode = n;
          bestKey = k;
        }
      }
      open.delete(bestKey);
      const nodeKey = key(bestNode.c);
      if (nodeKey === key(goal)) {
        const path = [];
        let n = bestNode;
        while (n) {
          path.unshift(n.c);
          n = n.parent;
        }
        return path;
      }
      closed.add(nodeKey);
      dirs.forEach((d) => {
        const nx = bestNode.c.x + d.x;
        const ny = bestNode.c.y + d.y;
        const nk = key({ x: nx, y: ny });
        if (closed.has(nk) || !this.grid.isInside(nx, ny) || this.grid.value(nx, ny) > 0.4) return;
        const g = bestNode.g + 1;
        const h = Math.hypot(goal.x - nx, goal.y - ny);
        const existing = open.get(nk);
        if (!existing || g + h < existing.f) {
          open.set(nk, { c: { x: nx, y: ny }, g, f: g + h, parent: bestNode });
        }
      });
    }
    return [];
  }

  _stepPhysics(dt) {
    this.time += dt;
    this._updateLidar(dt);
    this._planIfNeeded(dt);

    let desired;
    if (this.currentPlan) {
      const localT = (this.time - this.currentPlan.startTime);
      desired = this.currentPlan.traj.sample(localT);
      if (localT > this.currentPlan.traj.totalTime) this.currentPlan = null;
    } else {
      desired = { position: this.drone.getState().position.clone(), velocity: new THREE.Vector3(), acceleration: new THREE.Vector3(0, 0, 0), yaw: 0 };
    }

    const control = this.controller.computeControl(this.drone.getState(), desired);
    this.drone.step(control, dt);

    const state = this.drone.getState();
    this.droneMesh.position.copy(state.position);
    this.droneMesh.quaternion.copy(state.quaternion);
  }

  _render() {
    // Camera follows from behind
    const target = this.droneMesh.position;
    const forward = new THREE.Vector3(0, 0, -1).applyQuaternion(this.droneMesh.quaternion);
    const desiredCam = target.clone().add(forward.clone().multiplyScalar(-3)).add(new THREE.Vector3(0, 1.0, 0));
    this.camera.position.lerp(desiredCam, 0.1);
    this.camera.lookAt(target);
    this.renderer.render(this.scene, this.camera);

    const occ = this.grid.occupancy();
    const frontiers = this.grid.frontierCells();
    this.overlay.querySelector('#caveState').textContent = `State: ${this.state}`;
    this.overlay.querySelector('#caveTime').textContent = `t = ${this.time.toFixed(2)} s`;
    this.overlay.querySelector('#caveCoverage').textContent = `Coverage: ${(occ.coverage * 100).toFixed(1)}%`;
    this.overlay.querySelector('#caveFrontiers').textContent = `Frontiers: ${frontiers.length}`;

    this._drawMinimap(frontiers);
  }

  _drawMinimap(frontiers) {
    const ctx = this.minimap.getContext('2d');
    ctx.fillStyle = '#02040a';
    ctx.fillRect(0, 0, this.minimap.width, this.minimap.height);

    const scaleX = this.minimap.width / this.grid.size;
    const scaleY = this.minimap.height / this.grid.size;
    for (let y = 0; y < this.grid.size; y++) {
      for (let x = 0; x < this.grid.size; x++) {
        const v = this.grid.value(x, y);
        if (Math.abs(v) < 0.01) continue;
        ctx.fillStyle = v > 0 ? 'rgba(239,68,68,0.8)' : 'rgba(34,197,94,0.8)';
        ctx.fillRect(x * scaleX, y * scaleY, scaleX, scaleY);
      }
    }

    ctx.fillStyle = 'rgba(94,234,212,0.8)';
    frontiers.forEach((f) => {
      ctx.fillRect(f.x * scaleX, f.y * scaleY, scaleX, scaleY);
    });

    const state = this.drone.getState();
    const idx = this.grid.indexFromWorld(state.position);
    ctx.fillStyle = '#38bdf8';
    ctx.beginPath();
    ctx.arc(idx.x * scaleX, idx.y * scaleY, 3, 0, Math.PI * 2);
    ctx.fill();
  }
}
