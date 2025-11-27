// Autonomous cave exploration demo with LiDAR mapping, frontier planning, and Ammo.js collisions.
// Entry point: initDroneCaveDemo(container, options)

import { Quadrotor, createAmmoWorld } from '../../core/physics/drone_core_physics.js';
import { GeometricController } from '../../core/ai/drone_control.js';
import { clamp, gaussianNoise } from '../../core/utils/drone_math.js';

const ammoVec3 = (v) => new Ammo.btVector3(v.x, v.y, v.z);

class ReferenceTrajectory {
  constructor(points, totalTime) {
    this.points = points.map((p) => p.clone());
    this.totalTime = Math.max(totalTime, 0.01);
    this.segmentTime = this.totalTime / Math.max(1, this.points.length - 1);
  }

  sample(time) {
    const t = clamp(time, 0, this.totalTime);
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
    this.resolution = resolution;
    this.size = size;
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
    this.grid[idx] = clamp(this.grid[idx] + delta, -4, 4);
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
        if (this.value(x, y) < -0.2) {
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

class VIOLocalizer {
  constructor() {
    this.estimatedPose = {
      position: new THREE.Vector3(),
      quaternion: new THREE.Quaternion(),
    };
    this.velocity = new THREE.Vector3();
    this.bias = {
      position: new THREE.Vector3(),
      yaw: 0,
    };
  }

  reset(pose) {
    this.estimatedPose.position.copy(pose.position);
    this.estimatedPose.quaternion.copy(pose.quaternion);
    this.velocity.set(0, 0, 0);
    this.bias.position.set(0, 0, 0);
    this.bias.yaw = 0;
  }

  update(trueState, dt) {
    // Slow random walk bias to mimic drift
    this.bias.position.add(new THREE.Vector3(gaussianNoise(0, 0.002), 0, gaussianNoise(0, 0.002)).multiplyScalar(dt * 60));
    this.bias.yaw += gaussianNoise(0, 0.0005) * dt;

    const yawDrift = new THREE.Quaternion();
    yawDrift.setFromAxisAngle(new THREE.Vector3(0, 1, 0), this.bias.yaw);

    this.estimatedPose.position.copy(trueState.position).add(this.bias.position);
    this.estimatedPose.quaternion.copy(trueState.quaternion).premultiply(yawDrift);
    this.velocity.copy(trueState.velocity);
  }

  correctWithLandmark(truePose, weight = 0.1) {
    this.bias.position.lerp(new THREE.Vector3(), weight);
    this.bias.yaw *= 1 - weight;
    this.estimatedPose.position.lerp(truePose.position, weight);
    this.estimatedPose.quaternion.slerp(truePose.quaternion, weight);
  }
}

class LidarSimulator {
  constructor(ammoWorld) {
    this.world = ammoWorld;
    this.numRays = 24;
    this.maxRange = 18;
    this.verticalSpread = 0.2;
  }

  scan(pose) {
    if (!this.world) return [];
    const hits = [];
    const start = ammoVec3(pose.position);
    for (let i = 0; i < this.numRays; i++) {
      const theta = (i / this.numRays) * Math.PI * 2;
      const pitch = (Math.random() - 0.5) * this.verticalSpread;
      const dir = new THREE.Vector3(Math.cos(theta), pitch, Math.sin(theta)).applyQuaternion(pose.quaternion);
      const end = ammoVec3(pose.position.clone().add(dir.normalize().multiplyScalar(this.maxRange)));
      const callback = new Ammo.ClosestRayResultCallback(start, end);
      this.world.rayTest(start, end, callback);
      if (callback.hasHit()) {
        const pt = callback.get_hitPointWorld();
        const distance = pose.position.distanceTo(new THREE.Vector3(pt.x(), pt.y(), pt.z()));
        const noisy = distance + gaussianNoise(0, 0.05);
        hits.push({
          distance: noisy,
          direction: dir,
          point: new THREE.Vector3(pt.x(), pt.y(), pt.z()),
        });
      }
      Ammo.destroy(callback);
      Ammo.destroy(end);
    }
    Ammo.destroy(start);
    return hits;
  }
}

class FrontierPlanner {
  constructor(grid) {
    this.grid = grid;
    this.path = [];
  }

  plan(startCell, goalCell) {
    const open = [startCell];
    const cameFrom = new Map();
    const gScore = new Map();
    const key = (c) => `${c.x},${c.y}`;
    gScore.set(key(startCell), 0);

    const neighbors = (c) => [
      { x: c.x + 1, y: c.y },
      { x: c.x - 1, y: c.y },
      { x: c.x, y: c.y + 1 },
      { x: c.x, y: c.y - 1 },
    ].filter((n) => this.grid.isInside(n.x, n.y) && this.grid.value(n.x, n.y) < 0.2);

    while (open.length) {
      open.sort((a, b) => gScore.get(key(a)) - gScore.get(key(b)));
      const current = open.shift();
      if (current.x === goalCell.x && current.y === goalCell.y) {
        return this._reconstruct(cameFrom, current);
      }
      for (const nb of neighbors(current)) {
        const tentative = gScore.get(key(current)) + 1;
        if (!gScore.has(key(nb)) || tentative < gScore.get(key(nb))) {
          cameFrom.set(key(nb), current);
          gScore.set(key(nb), tentative);
          open.push(nb);
        }
      }
    }
    return [];
  }

  _reconstruct(cameFrom, current) {
    const path = [current];
    const key = (c) => `${c.x},${c.y}`;
    while (cameFrom.has(key(current))) {
      current = cameFrom.get(key(current));
      path.push(current);
    }
    return path.reverse();
  }
}

class CaveEnvironment {
  constructor(scene, ammoWorld) {
    this.scene = scene;
    this.world = ammoWorld;
    this.obstacles = [];
    this._buildCave();
  }

  _buildCave() {
    // Main tunnel
    const tunnelMat = new THREE.MeshStandardMaterial({ color: 0x0f172a, roughness: 0.9, metalness: 0.05 });
    const curve = new THREE.CatmullRomCurve3([
      new THREE.Vector3(0, 0, 0),
      new THREE.Vector3(0, 0, -6),
      new THREE.Vector3(4, 0, -12),
      new THREE.Vector3(6, -0.5, -18),
      new THREE.Vector3(0, 0, -24),
    ]);
    const geom = new THREE.TubeGeometry(curve, 60, 2.2, 12, false);
    const mesh = new THREE.Mesh(geom, tunnelMat);
    mesh.receiveShadow = true;
    this.scene.add(mesh);
    this.caveMesh = mesh;

    if (this.world) {
      // approximate tunnel as series of boxes for collisions
      for (let i = 0; i < 20; i++) {
        const t = i / 20;
        const center = curve.getPoint(t);
        this._addStaticBox(center, new THREE.Vector3(4, 4, 2));
      }
    }

    // Artefacts
    this.artefacts = [];
    for (let i = 0; i < 4; i++) {
      const m = new THREE.Mesh(
        new THREE.SphereGeometry(0.2, 12, 12),
        new THREE.MeshStandardMaterial({ color: 0xf97316, emissive: 0xff6b00 }),
      );
      m.position.copy(new THREE.Vector3(Math.sin(i) * 3, 0.2, -8 - i * 4));
      this.scene.add(m);
      this.artefacts.push(m);
    }

    // Debris obstacles
    for (let i = 0; i < 5; i++) {
      this.spawnObstacle(new THREE.Vector3((Math.random() - 0.5) * 4, -0.2, -4 - Math.random() * 10));
    }
  }

  _addStaticBox(center, halfExtents) {
    if (!this.world) return;
    const shape = new Ammo.btBoxShape(ammoVec3(halfExtents));
    const transform = new Ammo.btTransform();
    transform.setIdentity();
    transform.setOrigin(ammoVec3(center));
    const motionState = new Ammo.btDefaultMotionState(transform);
    const bodyInfo = new Ammo.btRigidBodyConstructionInfo(0, motionState, shape, new Ammo.btVector3(0, 0, 0));
    const body = new Ammo.btRigidBody(bodyInfo);
    this.world.addRigidBody(body);
  }

  spawnObstacle(position) {
    const geom = new THREE.DodecahedronGeometry(0.35 + Math.random() * 0.2);
    const mat = new THREE.MeshStandardMaterial({ color: 0x475569, metalness: 0.2, roughness: 0.8 });
    const mesh = new THREE.Mesh(geom, mat);
    mesh.position.copy(position);
    this.scene.add(mesh);

    let body = null;
    if (this.world) {
      const shape = new Ammo.btBoxShape(new Ammo.btVector3(0.35, 0.35, 0.35));
      const transform = new Ammo.btTransform();
      transform.setIdentity();
      transform.setOrigin(ammoVec3(position));
      const motionState = new Ammo.btDefaultMotionState(transform);
      const localInertia = new Ammo.btVector3(0, 0, 0);
      shape.calculateLocalInertia(1.5, localInertia);
      const info = new Ammo.btRigidBodyConstructionInfo(1.5, motionState, shape, localInertia);
      body = new Ammo.btRigidBody(info);
      this.world.addRigidBody(body);
    }

    this.obstacles.push({ mesh, body });
    return { mesh, body };
  }

  updateObstaclePose(obstacle, newPos) {
    obstacle.mesh.position.copy(newPos);
    if (obstacle.body) {
      const transform = obstacle.body.getWorldTransform();
      transform.setOrigin(ammoVec3(newPos));
      obstacle.body.setWorldTransform(transform);
      obstacle.body.activate();
    }
  }
}

export function initDroneCaveDemo(container, options = {}) {
  if (!container || typeof THREE === 'undefined') {
    console.warn('Cave demo requires a valid container and Three.js');
    return { pause() {}, resume() {}, restart() {}, setPausedFromVisibility() {}, destroy() {} };
  }
  const demo = new DroneCaveDemo(container, options);
  demo.start();
  return {
    pause: () => demo.pause(true),
    resume: () => demo.resume(true),
    restart: () => demo.restart(),
    setPausedFromVisibility: (visible) => demo.setPausedFromVisibility(visible),
    destroy: () => demo.destroy(),
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
    this._frameReq = null;

    this._initScene();
    this._initHUD();
    this._initWorld();
    this._initDrone();
    this._initControls();
    this._initMapping();
    this.restart();
  }

  _initScene() {
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x05070d);
    this.camera = new THREE.PerspectiveCamera(65, this.options.width / this.options.height, 0.1, 200);
    this.camera.position.set(0, 2, 6);

    this.fpvCamera = new THREE.PerspectiveCamera(75, this.options.width / this.options.height, 0.05, 50);
    this.renderer = new THREE.WebGLRenderer({ antialias: !!this.options.highQuality });
    this.renderer.setSize(this.options.width, this.options.height);
    this.renderer.setPixelRatio(this.options.highQuality ? window.devicePixelRatio : 1);
    this.container.innerHTML = '';
    this.container.appendChild(this.renderer.domElement);

    const ambient = new THREE.AmbientLight(0x8899aa, 0.18);
    const fog = new THREE.FogExp2(0x06080f, 0.05);
    this.scene.add(ambient);
    this.scene.fog = fog;

    const spot = new THREE.SpotLight(0x8bf3ff, 0.8, 25, Math.PI / 7, 0.3, 1.5);
    spot.position.set(0, 1, 0);
    this.scene.add(spot);
    this.headLight = spot;
  }

  _initHUD() {
    this.overlay = document.createElement('div');
    this.overlay.style.cssText =
      'position:absolute; top:8px; right:8px; background:rgba(5,6,12,0.82); color:#e2e8f0; padding:10px; font-family:"Fira Code", monospace; border:1px solid rgba(34,197,94,0.5); border-radius:8px; backdrop-filter: blur(8px); z-index:5; text-align:right;';
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

  _initWorld() {
    this.ammoWorld = createAmmoWorld();
    this.env = new CaveEnvironment(this.scene, this.ammoWorld);
  }

  _initDrone() {
    this.drone = new Quadrotor({ ammoWorld: this.ammoWorld });
    this.controller = new GeometricController({ mass: this.drone.params.mass });

    const body = new THREE.Mesh(
      new THREE.ConeGeometry(0.09, 0.2, 10),
      new THREE.MeshStandardMaterial({ color: 0x22c55e, emissive: 0x16a34a, roughness: 0.3, metalness: 0.25 }),
    );
    body.rotation.x = Math.PI / 2;

    const arms = new THREE.Mesh(
      new THREE.CylinderGeometry(0.02, 0.02, 0.24, 8),
      new THREE.MeshStandardMaterial({ color: 0x0ea5e9, emissive: 0x0284c7, metalness: 0.3 }),
    );
    arms.rotation.z = Math.PI / 4;

    this.droneGroup = new THREE.Group();
    this.droneGroup.add(body);
    this.droneGroup.add(arms);
    this.scene.add(this.droneGroup);
  }

  _initControls() {
    const panel = document.createElement('div');
    panel.style.cssText = 'position:absolute; top:8px; left:8px; display:flex; gap:6px; z-index:6;';
    panel.innerHTML = `
      <button data-action="start" style="padding:6px 10px;">Start</button>
      <button data-action="pause" style="padding:6px 10px;">Pause</button>
      <button data-action="reset" style="padding:6px 10px;">Reset</button>
      <button data-action="fpv" style="padding:6px 10px;">Toggle FPV</button>
      <button data-action="lidar" style="padding:6px 10px;">Toggle LiDAR</button>
      <button data-action="spawn" style="padding:6px 10px;">Spawn obstacle</button>
    `;
    this.container.appendChild(panel);
    this._controlPanel = panel;
    panel.addEventListener('click', (e) => {
      const action = e.target.dataset?.action;
      if (!action) return;
      if (action === 'start') this.resume(true);
      if (action === 'pause') this.pause(true);
      if (action === 'reset') this.restart();
      if (action === 'fpv') this.useFPV = !this.useFPV;
      if (action === 'lidar') this.showLidar = !this.showLidar;
      if (action === 'spawn') this.env.spawnObstacle(new THREE.Vector3((Math.random() - 0.5) * 3, 0, -6 - Math.random() * 8));
    });

    this.dragged = null;
    const raycaster = new THREE.Raycaster();
    const onPointerMove = (ev) => {
      if (!this.dragged) return;
      const rect = this.renderer.domElement.getBoundingClientRect();
      const ndc = new THREE.Vector2(((ev.clientX - rect.left) / rect.width) * 2 - 1, -((ev.clientY - rect.top) / rect.height) * 2 + 1);
      raycaster.setFromCamera(ndc, this.camera);
      const plane = new THREE.Plane(new THREE.Vector3(0, 1, 0), 0);
      const hit = new THREE.Vector3();
      raycaster.ray.intersectPlane(plane, hit);
      this.env.updateObstaclePose(this.dragged, hit);
    };

    this.renderer.domElement.addEventListener('pointerdown', (ev) => {
      const rect = this.renderer.domElement.getBoundingClientRect();
      const ndc = new THREE.Vector2(((ev.clientX - rect.left) / rect.width) * 2 - 1, -((ev.clientY - rect.top) / rect.height) * 2 + 1);
      raycaster.setFromCamera(ndc, this.camera);
      const intersects = raycaster.intersectObjects(this.env.obstacles.map((o) => o.mesh));
      if (intersects.length) {
        this.dragged = this.env.obstacles.find((o) => o.mesh === intersects[0].object);
        this.renderer.domElement.addEventListener('pointermove', onPointerMove);
      }
    });
    this.renderer.domElement.addEventListener('pointerup', () => {
      this.dragged = null;
      this.renderer.domElement.removeEventListener('pointermove', onPointerMove);
    });
  }

  _initMapping() {
    this.grid = new OccupancyGrid(0.3, 90);
    this.lidar = new LidarSimulator(this.ammoWorld);
    this.localizer = new VIOLocalizer();
    this.planner = new FrontierPlanner(this.grid);
    this.showLidar = true;
    this.useFPV = false;
    this.lastHits = [];
  }

  restart() {
    this.mode = 'HOVER';
    this.time = 0;
    this.drone.reset({ position: new THREE.Vector3(0, 0.4, 2) });
    this.localizer.reset({ position: new THREE.Vector3(0, 0.4, 2), quaternion: new THREE.Quaternion() });
    this.grid = new OccupancyGrid(0.3, 90);
    this._seedFreeSpace(this.drone.getState().position, 1.8);
    this.traj = null;
    this.trajTime = 0;
  }

  _seedFreeSpace(center, radius = 1.2) {
    const steps = Math.ceil(radius / this.grid.resolution);
    for (let dx = -steps; dx <= steps; dx++) {
      for (let dy = -steps; dy <= steps; dy++) {
        const wx = center.x + dx * this.grid.resolution;
        const wy = center.z + dy * this.grid.resolution;
        const cell = this.grid.indexFromWorld(new THREE.Vector3(wx, 0, wy));
        const dist = Math.hypot(dx * this.grid.resolution, dy * this.grid.resolution);
        if (dist <= radius) this.grid.updateCell(cell.x, cell.y, -0.8);
      }
    }
  }

  pause(user = false) {
    this.userPaused = user || this.userPaused;
    this.running = false;
    if (this._frameReq) {
      cancelAnimationFrame(this._frameReq);
      this._frameReq = null;
    }
  }

  resume(user = false) {
    this.userPaused = user ? false : this.userPaused;
    if (this.visibilityPaused) return;
    this.running = true;
    this.lastFrame = performance.now();
    this._frameReq = requestAnimationFrame((t) => this._loop(t));
  }

  setPausedFromVisibility(visible) {
    this.visibilityPaused = !visible;
    if (!visible) this.pause(false);
    else if (!this.userPaused) this.resume(false);
  }

  start() {
    if (this.running) return;
    this.running = true;
    this.lastFrame = performance.now();
    this._frameReq = requestAnimationFrame((t) => this._loop(t));
  }

  _loop(timestamp) {
    if (!this.running) return;
    const dt = Math.min(0.05, (timestamp - this.lastFrame) / 1000);
    this.lastFrame = timestamp;
    this.accumulator += dt * this.timeScale;
    const fixedDt = 1 / this.physicsRate;
    while (this.accumulator >= fixedDt) {
      this._stepPhysics(fixedDt);
      this.accumulator -= fixedDt;
      this.time += fixedDt;
    }
    this._render();
    this._frameReq = requestAnimationFrame((t) => this._loop(t));
  }

  _stepPhysics(dt) {
    const state = this.drone.getState();
    this.localizer.update(state, dt);

    this.lastHits = this.lidar.scan(this.localizer.estimatedPose);
    this._updateGridFromScan(this.lastHits, this.localizer.estimatedPose);

    const { coverage } = this.grid.occupancy();
    if (coverage > 0.6 && this.mode !== 'RETURN') {
      this.mode = 'RETURN';
      this.traj = new ReferenceTrajectory(
        [state.position.clone(), new THREE.Vector3(0, 0.5, 2)],
        8,
      );
      this.trajTime = 0;
    }

    if (!this.traj || this.trajTime > this.traj.totalTime) {
      const frontier = this._pickFrontier(this.localizer.estimatedPose.position);
      if (frontier) {
        const targetWorld = new THREE.Vector3(
          frontier.x * this.grid.resolution + this.grid.origin.x,
          0.5,
          frontier.y * this.grid.resolution + this.grid.origin.y,
        );
        this.traj = new ReferenceTrajectory([state.position.clone(), targetWorld], 5);
        this.trajTime = 0;
        this.mode = 'EXPLORE';
      } else {
        this.mode = 'HOVER';
      }
    }

    if (this.traj) {
      const desired = this.traj.sample(this.trajTime);
      const avoidance = this._computeAvoidance();
      desired.acceleration.add(avoidance);
      desired.yaw = Math.atan2(desired.velocity.x, desired.velocity.z);
      const control = this.controller.computeControl(state, desired);
      this.drone.step(control, dt);
      this.trajTime += dt;
    } else {
      this.drone.step({ thrust: this.drone.params.mass * 9.81, torque: new THREE.Vector3() }, dt);
    }
  }

  _pickFrontier(position) {
    const frontiers = this.grid.frontierCells();
    if (!frontiers.length) return null;
    frontiers.sort((a, b) => {
      const aw = new THREE.Vector2(a.x * this.grid.resolution + this.grid.origin.x, a.y * this.grid.resolution + this.grid.origin.y);
      const bw = new THREE.Vector2(b.x * this.grid.resolution + this.grid.origin.x, b.y * this.grid.resolution + this.grid.origin.y);
      const da = aw.distanceTo(new THREE.Vector2(position.x, position.z));
      const db = bw.distanceTo(new THREE.Vector2(position.x, position.z));
      return da - db;
    });
    return frontiers[0];
  }

  _updateGridFromScan(hits, pose) {
    for (const h of hits) {
      const hitPoint = h.point;
      const cell = this.grid.indexFromWorld(hitPoint);
      if (this.grid.isInside(cell.x, cell.y)) this.grid.updateCell(cell.x, cell.y, 1.2);

      // Carve free space along ray
      const steps = Math.ceil(h.distance / this.grid.resolution);
      const dir2d = new THREE.Vector2(h.direction.x, h.direction.z).normalize();
      for (let i = 1; i < steps; i++) {
        const pt = new THREE.Vector2(pose.position.x, pose.position.z).add(dir2d.clone().multiplyScalar(i * this.grid.resolution));
        const idx = this.grid.indexFromWorld(new THREE.Vector3(pt.x, 0, pt.y));
        this.grid.updateCell(idx.x, idx.y, -0.6);
      }
    }
  }

  _computeAvoidance() {
    const avoidance = new THREE.Vector3();
    for (const h of this.lastHits) {
      const proximity = Math.max(0, 1 - h.distance / this.lidar.maxRange);
      if (proximity > 0.35) {
        avoidance.add(h.direction.clone().multiplyScalar(-proximity * 4));
      }
    }
    return avoidance.clampLength(0, 6);
  }

  _render() {
    const state = this.drone.getState();
    this.droneGroup.position.copy(state.position);
    this.droneGroup.quaternion.copy(state.quaternion);
    this.headLight.position.copy(state.position.clone().add(new THREE.Vector3(0, 0.4, 0)));
    this.headLight.target.position.copy(state.position.clone().add(new THREE.Vector3(0, 0, -1).applyQuaternion(state.quaternion)));
    this.headLight.target.updateMatrixWorld();

    if (this.showLidar) this._renderLidar(state);

    if (this.useFPV) {
      this.fpvCamera.position.copy(state.position.clone().add(new THREE.Vector3(0, 0.05, 0)));
      this.fpvCamera.quaternion.copy(state.quaternion);
      this.renderer.render(this.scene, this.fpvCamera);
    } else {
      this.camera.position.lerp(state.position.clone().add(new THREE.Vector3(4, 2, 6)), 0.05);
      this.camera.lookAt(state.position);
      this.renderer.render(this.scene, this.camera);
    }

    this._drawMinimap(state);
    this._updateHUD(state);
  }

  _renderLidar(state) {
    if (this.lidarLines) {
      this.scene.remove(this.lidarLines);
    }
    const hits = this.lastHits || [];
    const geo = new THREE.BufferGeometry();
    const positions = new Float32Array(hits.length * 6);
    hits.forEach((h, i) => {
      positions[i * 6 + 0] = state.position.x;
      positions[i * 6 + 1] = state.position.y;
      positions[i * 6 + 2] = state.position.z;
      positions[i * 6 + 3] = h.point.x;
      positions[i * 6 + 4] = h.point.y;
      positions[i * 6 + 5] = h.point.z;
    });
    geo.setAttribute('position', new THREE.BufferAttribute(positions, 3));
    this.lidarLines = new THREE.LineSegments(
      geo,
      new THREE.LineBasicMaterial({ color: 0x22d3ee, transparent: true, opacity: 0.35 }),
    );
    this.scene.add(this.lidarLines);
  }

  _drawMinimap(state) {
    const ctx = this.minimap.getContext('2d');
    ctx.fillStyle = '#020617';
    ctx.fillRect(0, 0, this.minimap.width, this.minimap.height);

    const cellSize = this.minimap.width / this.grid.size;
    for (let y = 0; y < this.grid.size; y++) {
      for (let x = 0; x < this.grid.size; x++) {
        const v = this.grid.value(x, y);
        if (Math.abs(v) < 0.01) continue;
        ctx.fillStyle = v > 0 ? '#ef4444' : '#0ea5e9';
        ctx.globalAlpha = Math.min(Math.abs(v) / 4, 0.8);
        ctx.fillRect(x * cellSize, y * cellSize, cellSize, cellSize);
      }
    }
    ctx.globalAlpha = 1;

    // Drone
    const idx = this.grid.indexFromWorld(state.position);
    ctx.fillStyle = '#22c55e';
    ctx.beginPath();
    ctx.arc(idx.x * cellSize, idx.y * cellSize, 4, 0, Math.PI * 2);
    ctx.fill();
  }

  _updateHUD(state) {
    const coverage = this.grid.occupancy().coverage;
    const frontiers = this.grid.frontierCells().length;
    this.overlay.querySelector('#caveState').textContent = `State: ${this.mode}`;
    this.overlay.querySelector('#caveTime').textContent = `t = ${this.time.toFixed(2)} s`;
    this.overlay.querySelector('#caveCoverage').textContent = `Coverage: ${(coverage * 100).toFixed(1)}%`;
    this.overlay.querySelector('#caveFrontiers').textContent = `Frontiers: ${frontiers}`;
  }

  destroy() {
    this.pause();
    if (this._controlPanel?.parentNode === this.container) this.container.removeChild(this._controlPanel);
    if (this.overlay?.parentNode === this.container) this.container.removeChild(this.overlay);
    if (this.minimap?.parentNode === this.container) this.container.removeChild(this.minimap);
    if (this.renderer?.domElement?.parentNode === this.container) {
      this.container.removeChild(this.renderer.domElement);
    }
  }
}
