import { DronePhysicsEngine } from '../physics/drone_physics_engine.js';
import { CRAZYFLIE_PARAMS } from '../physics/params_crazyflie.js';
import { RaceController } from './race_controller.js';
import { buildRaceTrack, ReferenceTrajectory } from './track.js';
import { TimeOptimalTrajectory } from './trajectory_optimizer.js';
import { RaceUIController } from './race_ui_controller.js';
import { createUnifiedHUD, updateUnifiedHUD } from './unified_hud.js';

function clamp(x, min, max) {
  return Math.min(Math.max(x, min), max);
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
        width: container.clientWidth || 800,
        height: container.clientHeight || 450,
      },
      options,
    );

    this.floorHeight = options.floorHeight ?? 0.05;

    this.physicsRate = 240;
    this.timeScale = 1.0;
    this.state = 'COUNTDOWN';
    this.userPaused = true; // Start paused
    this.visibilityPaused = false;
    this.lastFrame = null;
    this.accumulator = 0;
    this._frameReq = null;
    this.debugEnabled = false;
    this.cameraMode = 'chase';
    this.controllerMode = 'geometric'; // 'geometric' or 'time-optimal'
    this.stabilizationTime = 1.5; // Hold position for 1.5 seconds before starting trajectory
    this.elapsedStabilizationTime = 0;
    this.ethControllerLogShown = false;

    this._initScene();
    this._initDrone();
    this._initTrack();
    this._initHUD();
    this._bindInputs();
    this._initUIController();
    this.restart();
  }

  _initScene() {
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x0a1020);
    const aspect = this.options.width / this.options.height;
    
    // Perspective camera for third-person view
    this.chaseCamera = new THREE.PerspectiveCamera(70, aspect, 0.05, 500);
    this.chaseCamera.position.set(-2, 0.8, 1.5);
    this.chaseCamera.up.set(0, 0, 1); // +Z is up
    this.chaseCamera.lookAt(new THREE.Vector3(0, 0, 2.5));
    
    this.overheadCamera = new THREE.PerspectiveCamera(60, aspect, 0.05, 200);
    this.overheadCamera.position.set(0, 0, 26); // Look down from +Z
    this.overheadCamera.up.set(0, 1, 0); // Up vector in +Y direction
    this.overheadCamera.lookAt(new THREE.Vector3(0, 0, 0));
    this.activeCamera = this.cameraMode === 'overhead' ? this.overheadCamera : this.chaseCamera;

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

    const floorGeo = new THREE.PlaneGeometry(200, 200);
    const floorMat = new THREE.MeshStandardMaterial({ color: 0x20252b, roughness: 0.8, metalness: 0.1 });
    const floor = new THREE.Mesh(floorGeo, floorMat);
    floor.rotation.y = 0; // Floor is X-Y plane (Z is up)
    floor.position.z = this.floorHeight;
    floor.receiveShadow = true;
    this.scene.add(floor);

    this.trailGeometry = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3()]);
    this.trailLine = new THREE.Line(
      this.trailGeometry,
      new THREE.LineBasicMaterial({ color: 0x8be9fd, transparent: true, opacity: 0.55 }),
    );
    this.scene.add(this.trailLine);
  }

  _initDrone() {
    this.params = CRAZYFLIE_PARAMS;
    this.drone = new DronePhysicsEngine(this.params);

    // Initialize controller types (ETH-based)
    this.geometricController = new RaceController(this.params);
    this.timeOptimalController = new RaceController(this.params); // Will use time-optimal trajectory
    
    // Set active controller
    this.controller = this.geometricController;
    // Create body matching stationary demo (+Z is up, X is forward)
    const bodyGeometry = new THREE.BoxGeometry(0.28, 0.28, 0.08);
    const topMat = new THREE.MeshStandardMaterial({ color: 0xff0000, metalness: 0.35, roughness: 0.45 });
    const bottomMat = new THREE.MeshStandardMaterial({ color: 0x0000ff, metalness: 0.35, roughness: 0.45 });
    const sideMat = new THREE.MeshStandardMaterial({ color: 0x22d3ee, metalness: 0.35, roughness: 0.45 });
    const bodyMaterials = [sideMat, sideMat, topMat, bottomMat, sideMat, sideMat];
    const body = new THREE.Mesh(bodyGeometry, bodyMaterials);
    body.castShadow = true;
    body.receiveShadow = true;
    
    const armMat = new THREE.MeshStandardMaterial({ color: 0x0ea5e9, metalness: 0.25, roughness: 0.6 });
    const arm = new THREE.Mesh(new THREE.BoxGeometry(0.8, 0.12, 0.05), armMat); // Arm along X (forward)
    arm.castShadow = true;
    arm.receiveShadow = true;

    this.droneMesh = new THREE.Group();
    this.droneMesh.add(body, arm);

    // Add colored motor rotors (ETH Zürich X-configuration)
    const rotorGeo = new THREE.RingGeometry(0.09, 0.14, 16);
    const rotorColors = [0xff0000, 0x00ff00, 0x0000ff, 0xffff00];
    const rotorPositions = [
      new THREE.Vector3(0.28, 0.28, 0.06),   // Motor 0 - Front-Left: +X (front), +Y (left)
      new THREE.Vector3(0.28, -0.28, 0.06),  // Motor 1 - Front-Right: +X (front), -Y (right)
      new THREE.Vector3(-0.28, -0.28, 0.06), // Motor 2 - Back-Right: -X (back), -Y (right)
      new THREE.Vector3(-0.28, 0.28, 0.06),  // Motor 3 - Back-Left: -X (back), +Y (left)
    ];
    rotorPositions.forEach((p, i) => {
      const rotorMat = new THREE.MeshStandardMaterial({ color: rotorColors[i], emissive: rotorColors[i], emissiveIntensity: 0.5, metalness: 0.6, roughness: 0.4 });
      const rTop = new THREE.Mesh(rotorGeo, rotorMat);
      rTop.position.copy(p);
      rTop.position.z += 0.01;
      rTop.castShadow = true;
      this.droneMesh.add(rTop);
      const rBottom = new THREE.Mesh(rotorGeo, rotorMat);
      rBottom.position.copy(p);
      rBottom.position.z -= 0.01;
      rBottom.castShadow = true;
      this.droneMesh.add(rBottom);
    });

    this.droneMesh.position.copy(new THREE.Vector3(-15, 0, 2.4)); // Start on the platform (same as waypoint)
    this.scene.add(this.droneMesh);
    
  }

  _initTrack() {
    const { gates, waypoints } = buildRaceTrack(this.scene);
    this.gates = gates;
    this.waypoints = waypoints;
    
    // Create both trajectory types
    this.standardTrajectory = new ReferenceTrajectory(waypoints, 50); // Conservative 50s
    this.timeOptimalTrajectory = new TimeOptimalTrajectory(waypoints, {
      maxVelocity: 12.0,
      maxAcceleration: 20.0,
      aggressiveness: 0.85,
    });

    // Set trajectories on controllers
    this.geometricController.setTrajectory(this.standardTrajectory);
    this.timeOptimalController.setTrajectory(this.timeOptimalTrajectory);
    
    this.trajectory = this.standardTrajectory;
  }

  _initHUD() {
    // HUD will be created by RaceUIController and inserted into controls panel
    // We'll get a reference to it after initialization
    this.unifiedHUD = null;
  }

  _initUIController() {
    this.uiController = new RaceUIController(this, this.container);
  }

  _bindInputs() {
    window.addEventListener('keydown', (e) => {
      if (e.key === 'd' || e.key === 'D') {
        this.debugEnabled = !this.debugEnabled;
      }
      if (e.key === 'c' || e.key === 'C') {
        if (this.cameraMode === 'chase') {
          this.cameraMode = 'overhead';
          this.activeCamera = this.overheadCamera;
        } else {
          this.cameraMode = 'chase';
          this.activeCamera = this.chaseCamera;
        }
      }
      if (e.key === 'm' || e.key === 'M') {
        this._cycleControllerMode();
      }
    });
  }

  _setControllerMode(mode) {
    this.controllerMode = mode === 'time-optimal' ? 'time-optimal' : 'geometric';
    
    // Switch controller and trajectory
    if (this.controllerMode === 'geometric') {
      this.controller = this.geometricController;
      this.trajectory = this.standardTrajectory;
    } else if (this.controllerMode === 'time-optimal') {
      this.controller = this.timeOptimalController;
      this.trajectory = this.timeOptimalTrajectory;
    }
    
    console.log('[Controller] Switched to', this.controllerMode);
    this.restart();
  }

  _cycleControllerMode() {
    const modes = ['geometric', 'time-optimal'];
    const currentIdx = modes.indexOf(this.controllerMode);
    const nextIdx = (currentIdx + 1) % modes.length;
    this._setControllerMode(modes[nextIdx]);
  }

  _logEthControllerActive() {
    if (this.ethControllerLogShown) return;
    console.log('[CTRL] ETH controller active');
    this.ethControllerLogShown = true;
  }

  _cycleCameraMode() {
    if (this.cameraMode === 'chase') {
      this.cameraMode = 'overhead';
      this.activeCamera = this.overheadCamera;
    } else {
      this.cameraMode = 'chase';
      this.activeCamera = this.chaseCamera;
    }
    console.log('[Camera] Switched to', this.cameraMode);
  }

  restart() {
    // Position drone on starting platform with zero orientation and zero velocity (+Z is up)
    const droneState = {
      position: new THREE.Vector3(-15, 0, 2.4),
      velocity: new THREE.Vector3(0, 0, 0),
      orientation: new THREE.Quaternion(0, 0, 0, 1), // Identity quaternion - no rotation
      angularVelocity: new THREE.Vector3(0, 0, 0),
    };
    this.drone.reset(droneState);
    this.geometricController.reset();
    this.timeOptimalController.reset();
    this.state = 'STABILIZING'; // Hold position first
    this.elapsedStabilizationTime = 0;
    this.ethControllerLogShown = false;
    const state = this.drone.getState();
    this.trailGeometry.setFromPoints([state.position.clone(), state.position.clone()]);
    // Sync mesh position with physics state
    this.droneMesh.position.copy(state.position);
    this.droneMesh.quaternion.copy(state.orientation);
  }

  pause(user = false) {
    if (user) this.userPaused = true;
    if (this._frameReq) cancelAnimationFrame(this._frameReq);
    this._frameReq = null;
  }

  resume(user = false) {
    if (user) this.userPaused = false;
    // Start the loop if not already running, even when paused (we'll skip physics but still render)
    if (!this._frameReq && !this.visibilityPaused) {
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

  start() {
    // Render once immediately to show the initial scene even when paused
    this._render();
    // Then try to resume the animation loop
    this.resume(false);
  }

  destroy() {
    this.pause(false);
    this.container.innerHTML = '';
  }

  _loop(timestamp) {
    if (this.lastFrame === null) this.lastFrame = timestamp;
    const dt = ((timestamp - this.lastFrame) / 1000) * this.timeScale;
    this.lastFrame = timestamp;

    const fixedDt = 1 / this.physicsRate;
    this.accumulator += dt;
    while (this.accumulator >= fixedDt) {
      if (!this.userPaused && !this.visibilityPaused) {
        this._stepPhysics(fixedDt);
      }
      this.accumulator -= fixedDt;
    }

    this._render();
    this._frameReq = requestAnimationFrame((t) => this._loop(t));
  }

  _stepPhysics(dt) {
    if (this.state === 'STABILIZING') {
      // During stabilization, hold drone at starting position
      this.elapsedStabilizationTime += dt;
      
      // Get drone state and compute control to hold position
      const state = this.drone.getState();
      const hoverTarget = {
        position: new THREE.Vector3(0, 0, 2.4),
        velocity: new THREE.Vector3(0, 0, 0),
        acceleration: new THREE.Vector3(0, 0, 0),
        yaw: 0,
      };
      const result = this.controller.computeControl(state, hoverTarget, dt);

      // Apply motor commands and step physics
      this._logEthControllerActive();
      this.drone.applyMotorCommands(result.motorCommands[0], result.motorCommands[1], result.motorCommands[2], result.motorCommands[3]);
      this.drone.step(dt);
      
      // After stabilization time, switch to running
      if (this.elapsedStabilizationTime >= this.stabilizationTime) {
        this.state = 'RUNNING';
        this.controller.reset();
      }
      return; // Skip rest of physics during stabilization
    }

    if (this.state !== 'RUNNING') return;

    // Update controller time and get target
    this.controller.update(dt);
    const target = this.controller.getTarget();
    
    // Get drone state and compute control
    const state = this.drone.getState();
    const result = this.controller.computeControl(state, target, dt);
    
    // Store result for visualization and debug
    this.lastControlResult = result;
    
    // Apply motor commands and step physics
    this._logEthControllerActive();
    this.drone.applyMotorCommands(result.motorCommands[0], result.motorCommands[1], result.motorCommands[2], result.motorCommands[3]);
    this.drone.step(dt);

    // Check if drone reached the last gate
    const dronePos = this.drone.state.position;
    const lastGatePos = this.waypoints[this.waypoints.length - 1];
    const gateThreshold = 2.0; // Distance threshold to consider gate reached
    
    if (dronePos.distanceTo(lastGatePos) < gateThreshold) {
      this.state = 'FINISHED';
    }

    // Collision detection with platform and ground
    const pos = this.drone.state.position;
    const droneRadius = 0.2;
    
    // Platform collision (green pad at z=2.4, x-y plane, now at x=-15)
    const platformPos = new THREE.Vector3(-15, 0, 2.4);
    const platformSize = new THREE.Vector3(3, 3, 0.2);
    if (pos.x > platformPos.x - platformSize.x / 2 && pos.x < platformPos.x + platformSize.x / 2 &&
        pos.y > platformPos.y - platformSize.y / 2 && pos.y < platformPos.y + platformSize.y / 2 &&
        pos.z < platformPos.z + platformSize.z / 2 + droneRadius && pos.z > platformPos.z - droneRadius) {
      // Prevent drone from falling through platform
      this.drone.state.position.z = platformPos.z + platformSize.z / 2 + droneRadius;
      // Dampen downward velocity
      if (this.drone.state.velocity.z < 0) {
        this.drone.state.velocity.z *= 0.5;
      }
    }
    
    // Ground collision
    if (pos.z < this.floorHeight + droneRadius) {
      this.drone.state.position.z = this.floorHeight + droneRadius;
      if (this.drone.state.velocity.z < 0) {
        this.drone.state.velocity.z *= 0.3;
      }
    }
    
    // Gate ring collision detection (torus in Y-Z plane, normal along +X)
    // Rings are vertical donuts; drone should pass through the hole without hitting the tube
    // DISABLED: Torus collision causing issues - drone can pass through freely
    /*
    this.gates.forEach((gate, idx) => {
      const gateWorldPos = gate.getWorldPosition(new THREE.Vector3());
      const torusRadius = 1.5;        // Major radius (distance from center to ring center)
      const tubeRadius = 0.08;        // Minor radius (thickness of the tube)
      const droneRadius = 0.2;
      
      // For a ring in Y-Z plane centered at gateWorldPos:
      // Distance from ring to drone = distance of drone to the circle in Y-Z plane
      const relPos = pos.clone().sub(gateWorldPos);
      
      // Distance in Y-Z plane (perpendicular to X-axis flight direction)
      const yzDist = Math.sqrt(relPos.y * relPos.y + relPos.z * relPos.z);
      
      // Distance from drone to ring surface in Y-Z plane
      const distToRing = Math.abs(yzDist - torusRadius);
      
      // Only collide if drone hits the actual ring tube (within tubeRadius + droneRadius)
      if (distToRing < tubeRadius + droneRadius && yzDist > 0.1) {
        // Push drone away from ring center (in Y-Z plane)
        const yzDirection = new THREE.Vector3(0, relPos.y, relPos.z).normalize();
        const pushDistance = (tubeRadius + droneRadius - distToRing) * 1.1;
        
        // Only push outward (away from ring center)
        if (yzDist < torusRadius) {
          // Inside ring - push outward
          this.drone.state.position.add(yzDirection.multiplyScalar(-pushDistance));
        } else {
          // Outside ring - push outward
          this.drone.state.position.add(yzDirection.multiplyScalar(pushDistance));
        }
        
        // Dampen velocity component in Y-Z plane
        const velYZ = new THREE.Vector3(0, this.drone.state.velocity.y, this.drone.state.velocity.z);
        this.drone.state.velocity.add(velYZ.multiplyScalar(-0.3));
      }
    });
    */

    // Check gate crossing
    const vel = this.drone.state.velocity.length();
    this.controller.updateGateIndex(pos, this.waypoints, 1.5);

    // Check if completed all gates
    if (this.controller.getGateIndex() >= this.gates.length) {
      this.state = 'COMPLETE';
    }
  }

  _render() {
    if (this.cameraMode === 'chase') {
      // Third-person camera follows drone from behind - very close view
      // Offset: behind the drone (negative X since +X is flight direction), slightly to the right (positive Y), and above
      const offset = new THREE.Vector3(-0.5, 0.2, 0.5);
      const targetPos = this.drone.state.position.clone().add(offset);
      this.chaseCamera.position.lerp(targetPos, 0.1);
      
      // Look at a point slightly ahead of the drone for better perspective
      const lookAheadDist = 2;
      const lookTarget = this.drone.state.position.clone().add(new THREE.Vector3(lookAheadDist, 0, 0.2));
      this.chaseCamera.lookAt(lookTarget);
      this.activeCamera = this.chaseCamera;
    } else {
      this.overheadCamera.position.z = 26; // Look down from above (+Z)
      this.overheadCamera.lookAt(new THREE.Vector3(0, 0, 0));
      this.activeCamera = this.overheadCamera;
    }
    this.droneMesh.position.copy(this.drone.state.position);
    this.droneMesh.quaternion.copy(this.drone.state.orientation);

    const points = this.trailGeometry.getAttribute('position');
    points.setXYZ(0, this.drone.state.position.x, this.drone.state.position.y, this.drone.state.position.z);
    points.setXYZ(1, this.drone.state.position.x, this.drone.state.position.y, this.drone.state.position.z);
    points.needsUpdate = true;

    // Update unified HUD every frame (including when paused)
    this._updateUnifiedHUD();
    this.renderer.render(this.scene, this.activeCamera || this.chaseCamera);
  }

  _updateUnifiedHUD() {
    if (!this.unifiedHUD) return;
    
    const st = this.drone.state;
    const vel = st.velocity.length();
    const acc = st.acceleration ? st.acceleration.length() : 0;
    
    // Compute tilt angle from orientation
    const euler = new THREE.Euler().setFromQuaternion(st.orientationQuat || st.orientation);
    const tiltAngle = THREE.MathUtils.radToDeg(Math.sqrt(euler.x * euler.x + euler.y * euler.y));
    
    // Compute tracking error
    const target = this.controller.getTarget();
    const trackingError = st.position.distanceTo(target.position);
    
    // Prepare unified HUD data
    const hudData = {
      // Race status
      state: this.state,
      time: this.controller.simTime,
      gateIdx: this.controller.getGateIndex(),
      gateTotal: this.gates.length,
      speed: vel,
      
      // Controller info
      controllerMode: this.controllerMode,
      cameraMode: this.cameraMode,
      trackingError,
      
      // Performance metrics
      velocity: vel,
      acceleration: acc,
      tiltAngle,
      altitude: st.position.z,
      
      // Debug
      debugEnabled: this.debugEnabled,
    };
    
    // Add time-optimal data
    if (this.controllerMode === 'time-optimal') {
      const metrics = this.timeOptimalTrajectory.getMetrics();
      hudData.timeOptData = {
        totalDuration: metrics.totalDuration,
        avgSpeed: metrics.avgSpeed,
        maxSpeed: metrics.maxSpeed,
        aggressiveness: metrics.aggressiveness,
      };
    }
    
    // Add debug data
    if (this.debugEnabled) {
      const rpm = st.motorRPM || [0, 0, 0, 0];
      const desired = this.drone.desiredOrientationQuat || new THREE.Quaternion();
      const qErr = desired.clone().multiply((st.orientationQuat || st.orientation).clone().invert());
      const attErrMag = 2 * new THREE.Vector3(qErr.x, qErr.y, qErr.z).length();
      
      hudData.debugData = {
        position: st.position,
        velocity: st.velocity,
        velocityMag: vel,
        motorRPM: rpm,
        thrust: st.totalThrust,
        attitude: {
          roll: THREE.MathUtils.radToDeg(euler.x),
          pitch: THREE.MathUtils.radToDeg(euler.y),
          yaw: THREE.MathUtils.radToDeg(euler.z),
        },
        attitudeError: attErrMag,
      };
    }
    
    updateUnifiedHUD(this.unifiedHUD, hudData);
  }
}

// Auto-initialize the drone race demo when the page loads
if (document.readyState === 'loading') {
  document.addEventListener('DOMContentLoaded', () => {
    const container = document.getElementById('drone-race-demo');
    if (container) {
      initDroneRaceDemo(container, {
        width: container.clientWidth || 800,
        height: container.clientHeight || 520,
        enableShadows: true,
        highQuality: true,
      });
    }
  });
} else {
  // DOM is already loaded
  const container = document.getElementById('drone-race-demo');
  if (container) {
    initDroneRaceDemo(container, {
      width: container.clientWidth || 800,
      height: container.clientHeight || 520,
      enableShadows: true,
      highQuality: true,
    });
  }
}
