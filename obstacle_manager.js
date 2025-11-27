import { buildObstaclePreset } from './obstacle_presets.js';

const CF_KINEMATIC_OBJECT = 2;
const CF_STATIC_OBJECT = 1;

function ammoVec3(v) {
  return new Ammo.btVector3(v.x, v.y, v.z);
}

function createShape(preset) {
  switch (preset.collider?.type) {
    case 'sphere':
      return new Ammo.btSphereShape(preset.collider.radius);
    case 'box':
      return new Ammo.btBoxShape(ammoVec3(preset.collider.halfExtents));
    case 'hull':
    default: {
      const geom = preset.geometry.clone();
      const pos = geom.attributes.position;
      const hull = new Ammo.btConvexHullShape();
      for (let i = 0; i < pos.count; i++) {
        hull.addPoint(new Ammo.btVector3(pos.getX(i), pos.getY(i), pos.getZ(i)), i === pos.count - 1);
      }
      return hull;
    }
  }
}

export class ObstacleManager {
  constructor(scene, ammoWorld, { onChange } = {}) {
    this.scene = scene;
    this.world = ammoWorld;
    this.obstacles = [];
    this.onChange = onChange;
    this._id = 0;
  }

  spawn(type, position = new THREE.Vector3(), overrides = {}) {
    const preset = buildObstaclePreset(type, overrides);
    const mesh = new THREE.Mesh(preset.geometry, preset.material);
    mesh.castShadow = true;
    mesh.receiveShadow = true;
    mesh.position.copy(position);
    this.scene.add(mesh);

    let body = null;
    if (this.world) {
      const shape = createShape(preset);
      const transform = new Ammo.btTransform();
      transform.setIdentity();
      transform.setOrigin(ammoVec3(position));
      const motionState = new Ammo.btDefaultMotionState(transform);
      const localInertia = new Ammo.btVector3(0, 0, 0);
      const mass = Math.max(0.8, preset.mass || 1.5);
      shape.calculateLocalInertia(mass, localInertia);
      const info = new Ammo.btRigidBodyConstructionInfo(mass, motionState, shape, localInertia);
      info.set_m_friction(0.9);
      info.set_m_restitution(0.2);
      const rb = new Ammo.btRigidBody(info);
      rb.setDamping(0.2, 0.85);
      rb.setActivationState(4);
      this.world.addRigidBody(rb);
      body = rb;
    }

    const obstacle = { id: this._id++, mesh, body, preset, kinematic: false };
    this.obstacles.push(obstacle);
    this._notify();
    return obstacle;
  }

  getInteractableMeshes() {
    return this.obstacles.map((o) => o.mesh);
  }

  setKinematic(obstacle, enabled) {
    if (!obstacle?.body) return;
    obstacle.kinematic = enabled;
    const body = obstacle.body;
    const flags = body.getCollisionFlags();
    if (enabled) {
      body.setCollisionFlags(flags | CF_KINEMATIC_OBJECT);
      body.setActivationState(4);
      body.setMassProps(0, new Ammo.btVector3(0, 0, 0));
    } else {
      body.setCollisionFlags(flags & ~CF_KINEMATIC_OBJECT & ~CF_STATIC_OBJECT);
      const shape = body.getCollisionShape();
      const inertia = new Ammo.btVector3(0, 0, 0);
      shape.calculateLocalInertia(obstacle.preset.mass || 1.5, inertia);
      body.setMassProps(obstacle.preset.mass || 1.5, inertia);
      body.activate();
    }
  }

  updatePose(obstacle, position, quaternion = null) {
    if (!obstacle) return;
    obstacle.mesh.position.copy(position);
    if (quaternion) obstacle.mesh.quaternion.copy(quaternion);

    if (obstacle.body) {
      const transform = obstacle.body.getWorldTransform();
      transform.setOrigin(ammoVec3(position));
      if (quaternion) {
        transform.setRotation(
          new Ammo.btQuaternion(quaternion.x, quaternion.y, quaternion.z, quaternion.w),
        );
      }
      obstacle.body.setWorldTransform(transform);
      obstacle.body.activate();
    }
  }

  syncFromPhysics() {
    if (!this.world) return;
    for (const obstacle of this.obstacles) {
      if (obstacle.kinematic || !obstacle.body) continue;
      const transform = obstacle.body.getWorldTransform();
      const origin = transform.getOrigin();
      const rot = transform.getRotation();
      obstacle.mesh.position.set(origin.x(), origin.y(), origin.z());
      obstacle.mesh.quaternion.set(rot.x(), rot.y(), rot.z(), rot.w());
    }
  }

  _notify() {
    if (typeof this.onChange === 'function') this.onChange();
  }
}
