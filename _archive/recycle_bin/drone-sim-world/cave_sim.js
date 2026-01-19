import { Quadrotor, createAmmoWorld } from '../core/physics/drone_core_physics.js';
import { GeometricController } from '../core/ai/drone_control.js';
import { DroneAI } from '../core/ai/drone_ai.js';
import { ObstacleManager } from './obstacle_manager.js';
import { DraggableControls } from '../ui/draggable_controls.js';
import { OBSTACLE_TYPES } from './obstacle_presets.js';

const ammoVec3 = (v) => new Ammo.btVector3(v.x, v.y, v.z);

export class CaveSim {
  constructor(container, options = {}) {
    this.container = container;
    this.options = Object.assign(
      {
        width: container.clientWidth || 900,
        height: container.clientHeight || 520,
        shadows: true,
      },
      options,
    );

    this.physicsRate = 120;
    this.accumulator = 0;
    this.lastFrame = null;
    this.running = false;
    this._frameReq = null;
    this.simTime = 0;

    this._initScene();
    this._initWorld();
    this._initDrone();
    this._initEnvironment();
    this._initControls();
  }

  _initScene() {
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x04070e);
    this.camera = new THREE.PerspectiveCamera(60, this.options.width / this.options.height, 0.1, 120);
    this.camera.position.set(4, 3, 8);

    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderer.setSize(this.options.width, this.options.height);
    this.renderer.setPixelRatio(devicePixelRatio || 1);
    this.renderer.shadowMap.enabled = true;
    this.container.innerHTML = '';
    this.container.appendChild(this.renderer.domElement);

    const ambient = new THREE.AmbientLight(0x64748b, 0.25);
    this.scene.add(ambient);
    const dir = new THREE.DirectionalLight(0xbcd7ff, 1.4);
    dir.position.set(6, 10, 6);
    dir.castShadow = true;
    this.scene.add(dir);
  }

  _initWorld() {
    this.world = createAmmoWorld(new THREE.Vector3(0, -9.81, 0));
    this.ai = new DroneAI(this.world);
    this.obstacles = new ObstacleManager(this.scene, this.world, {
      onChange: () => this.ai.onEnvironmentChanged(),
    });
  }

  _initDrone() {
    this.drone = new Quadrotor({ ammoWorld: this.world });
    this.controller = new GeometricController({ mass: this.drone.params.mass });
    this.drone.reset({ position: new THREE.Vector3(0, 1.2, 4) });

    const body = new THREE.Mesh(
      new THREE.CapsuleGeometry(0.18, 0.3, 6, 12),
      new THREE.MeshStandardMaterial({ color: 0x22c55e, metalness: 0.3, roughness: 0.4 }),
    );
    body.castShadow = true;
    const ring = new THREE.Mesh(
      new THREE.TorusGeometry(0.28, 0.04, 6, 20),
      new THREE.MeshStandardMaterial({ color: 0x0ea5e9, emissive: 0x0284c7, roughness: 0.35 }),
    );
    ring.rotation.x = Math.PI / 2;
    this.droneMesh = new THREE.Group();
    this.droneMesh.add(body);
    this.droneMesh.add(ring);
    this.scene.add(this.droneMesh);
  }

  _initEnvironment() {
    const ground = new THREE.Mesh(
      new THREE.PlaneGeometry(40, 40, 1, 1),
      new THREE.MeshStandardMaterial({ color: 0x0b1220, roughness: 0.9 }),
    );
    ground.rotation.x = -Math.PI / 2;
    ground.receiveShadow = true;
    this.scene.add(ground);

    if (this.world) {
      const groundShape = new Ammo.btBoxShape(new Ammo.btVector3(20, 0.5, 20));
      const transform = new Ammo.btTransform();
      transform.setIdentity();
      transform.setOrigin(new Ammo.btVector3(0, -0.5, 0));
      const motionState = new Ammo.btDefaultMotionState(transform);
      const info = new Ammo.btRigidBodyConstructionInfo(0, motionState, groundShape, new Ammo.btVector3(0, 0, 0));
      const body = new Ammo.btRigidBody(info);
      this.world.addRigidBody(body);

      for (let i = 0; i < 8; i++) {
        const angle = (i / 8) * Math.PI * 2;
        const radius = 8;
        const wallPos = new THREE.Vector3(Math.cos(angle) * radius, 1.5, Math.sin(angle) * radius);
        const box = new Ammo.btBoxShape(new Ammo.btVector3(2.5, 1.5, 0.3));
        const t = new Ammo.btTransform();
        t.setIdentity();
        t.setOrigin(ammoVec3(wallPos));
        const quat = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 1, 0), -angle);
        t.setRotation(new Ammo.btQuaternion(quat.x, quat.y, quat.z, quat.w));
        const ms = new Ammo.btDefaultMotionState(t);
        const rbInfo = new Ammo.btRigidBodyConstructionInfo(0, ms, box, new Ammo.btVector3(0, 0, 0));
        this.world.addRigidBody(new Ammo.btRigidBody(rbInfo));
      }
    }
  }

  _initControls() {
    this.dragControls = new DraggableControls(this.camera, this.renderer.domElement, this.obstacles, {
      onDragEnd: () => this.ai.onEnvironmentChanged(),
    });
  }

  resetObstacles() {
    this.obstacles.clear();
    this.ai.onEnvironmentChanged();
  }

  start() {
    if (this.running) return;
    this.running = true;
    this.simTime = 0;
    this.lastFrame = performance.now();
    this._frameReq = requestAnimationFrame((t) => this._loop(t));
  }

  pause() {
    this.running = false;
    if (this._frameReq) {
      cancelAnimationFrame(this._frameReq);
      this._frameReq = null;
    }
  }

  resume() {
    if (this.running) return;
    this.running = true;
    this.simTime = this.simTime || 0;
    this.lastFrame = performance.now();
    this._frameReq = requestAnimationFrame((t) => this._loop(t));
  }

  destroy() {
    this.pause();
    if (this.dragControls) this.dragControls.dispose();
    if (this.renderer?.domElement?.parentNode === this.container) {
      this.container.removeChild(this.renderer.domElement);
    }
    this.obstacles?.clear?.();
    this.scene = null;
  }

  _loop(time) {
    if (!this.running) return;
    const dt = (time - this.lastFrame) / 1000;
    this.lastFrame = time;
    this.accumulator += dt;
    const fixed = 1 / this.physicsRate;
    while (this.accumulator >= fixed) {
      this._step(fixed);
      this.accumulator -= fixed;
    }
    this._render();
    this._frameReq = requestAnimationFrame((t) => this._loop(t));
  }

  _step(dt) {
    this.simTime += dt;
    this.drone.syncFromAmmoBody();

    const patrolPos = new THREE.Vector3(
      Math.sin(this.simTime * 0.4) * 2.5,
      1.25 + Math.sin(this.simTime * 0.8) * 0.1,
      Math.cos(this.simTime * 0.35) * 2.5,
    );
    const patrolVel = new THREE.Vector3(
      Math.cos(this.simTime * 0.4) * 2.5 * 0.4,
      Math.cos(this.simTime * 0.8) * 0.08,
      -Math.sin(this.simTime * 0.35) * 2.5 * 0.35,
    );

    const hits = this.ai.scanAndUpdate({ position: this.drone.state.position, quaternion: this.drone.state.quaternion });
    const avoidance = this._avoidanceFromHits(hits);

    const desired = {
      position: patrolPos.add(avoidance.positionOffset),
      velocity: patrolVel,
      acceleration: avoidance.accel,
      yaw: Math.atan2(patrolVel.x, patrolVel.z),
    };

    const control = this.controller.computeControl(this.drone.getState(), desired);
    this.drone.step(control, dt);

    this.obstacles.syncFromPhysics();
  }

  _render() {
    this.droneMesh.position.copy(this.drone.state.position);
    this.droneMesh.quaternion.copy(this.drone.state.quaternion);
    this.camera.lookAt(this.droneMesh.position);
    this.renderer.render(this.scene, this.camera);
  }

  spawnObstacle(type = OBSTACLE_TYPES.boulder) {
    const drop = this.drone.state.position.clone().add(new THREE.Vector3(0, 0.5, -1));
    this.obstacles.spawn(type, drop);
  }

  _avoidanceFromHits(hits) {
    const repel = new THREE.Vector3();
    for (const h of hits || []) {
      const proximity = Math.max(0, 1 - h.distance / (this.ai.lidar?.maxRange || 10));
      if (proximity > 0.25) repel.add(h.direction.clone().multiplyScalar(-proximity * 2.5));
    }
    return {
      positionOffset: repel.clampLength(0, 1.0),
      accel: repel.multiplyScalar(4).clampLength(0, 5),
    };
  }
}
