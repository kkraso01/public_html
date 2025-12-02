import { DronePhysicsEngine } from '../physics/drone_physics_engine.js';
import { CRAZYFLIE_PARAMS } from '../physics/params_crazyflie.js';
import { EthController } from '../stationary/eth_controller.js';
import { CaveSim } from './cave_sim.js';
import { Lidar } from './lidar.js';
import { OccupancyGrid, updateGridFromLidar, chooseFrontierTarget, planPath } from './mapping.js';

function clamp(x, min, max) {
  return Math.min(Math.max(x, min), max);
}

export function initDroneCaveDemo(container, options = {}) {
  if (!container || typeof THREE === 'undefined') {
    console.warn('Drone cave demo requires a valid container and Three.js');
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
      { width: container.clientWidth || 900, height: container.clientHeight || 520, shadows: true },
      options,
    );
    this.floorHeight = options.floorHeight ?? 0.05;
    this.physicsRate = 200;
    this.timeScale = 1;
    this.userPaused = false;
    this.visibilityPaused = false;
    this.lastFrame = null;
    this.accumulator = 0;
    this._frameReq = null;
    this.debugEnabled = false;
    this.cameraMode = 'chase'; // Default to chase camera like drone race

    this._initScene();
    this._initSim();
    this._initDrone();
    this._initHUD();
    this._bindInputs();
    this.restart();
  }

  _initScene() {
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x04070e);
    const aspect = this.options.width / this.options.height;
    
    // Chase camera (follows drone from behind) - like race demo
    this.chaseCamera = new THREE.PerspectiveCamera(70, aspect, 0.05, 200);
    this.chaseCamera.position.set(-1.2, 0, 1.5); // Z-up: behind and above
    this.chaseCamera.up.set(0, 0, 1); // +Z is up
    
    // Overhead camera (top-down view, Z-up)
    this.overheadCamera = new THREE.PerspectiveCamera(60, aspect, 0.05, 200);
    this.overheadCamera.position.set(0, 0, 12); // Lower for better view
    this.overheadCamera.up.set(0, 1, 0); // Y is up in camera space
    this.overheadCamera.lookAt(new THREE.Vector3(0, 0, 0));
    
    this.activeCamera = this.cameraMode === 'overhead' ? this.overheadCamera : this.chaseCamera;

    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderer.setSize(this.options.width, this.options.height);
    this.renderer.setPixelRatio(devicePixelRatio || 1);
    this.renderer.shadowMap.enabled = true;
    this.container.innerHTML = '';
    this.container.appendChild(this.renderer.domElement);

    const ambient = new THREE.AmbientLight(0x64748b, 0.25);
    this.scene.add(ambient);
    
    // Directional light from above (Z-up)
    const dir = new THREE.DirectionalLight(0xbcd7ff, 1.4);
    dir.position.set(6, 6, 10); // Light from above
    dir.castShadow = true;
    this.scene.add(dir);

    // Floor at Z=0 (XY plane) - dark brown/grey
    const floorGeo = new THREE.PlaneGeometry(200, 200);
    const floorMat = new THREE.MeshStandardMaterial({ color: 0x2d2520, roughness: 0.9, metalness: 0.05 });
    const floor = new THREE.Mesh(floorGeo, floorMat);
    floor.position.z = this.floorHeight; // Z-up: floor at low Z
    floor.receiveShadow = true;
    this.scene.add(floor);
    
    // Boundary walls (Z-up: vertical walls in XY plane) - dark orange/brown
    const wallMat = new THREE.MeshStandardMaterial({ color: 0x4a3626, roughness: 0.85, metalness: 0.05 });
    const wallHeight = 8;
    
    const wallNorth = new THREE.Mesh(new THREE.BoxGeometry(60, 0.6, wallHeight), wallMat);
    wallNorth.position.set(0, -30, wallHeight / 2);
    
    const wallSouth = new THREE.Mesh(new THREE.BoxGeometry(60, 0.6, wallHeight), wallMat);
    wallSouth.position.set(0, 30, wallHeight / 2);
    
    const wallEast = new THREE.Mesh(new THREE.BoxGeometry(0.6, 60, wallHeight), wallMat);
    wallEast.position.set(30, 0, wallHeight / 2);
    
    const wallWest = new THREE.Mesh(new THREE.BoxGeometry(0.6, 60, wallHeight), wallMat);
    wallWest.position.set(-30, 0, wallHeight / 2);
    
    [wallNorth, wallSouth, wallEast, wallWest].forEach((w) => {
      w.receiveShadow = true;
      w.castShadow = true;
      this.scene.add(w);
    });
  }

  _initSim() {
    this.cave = new CaveSim(this.scene);
    this.grid = new OccupancyGrid();
    this.lidar = new Lidar();
    
    // Lidar ray visualization
    this.lidarRayGroup = new THREE.Group();
    this.scene.add(this.lidarRayGroup);
  }

  _initDrone() {
    this.params = CRAZYFLIE_PARAMS;
    this.drone = new DronePhysicsEngine(this.params);
    this.drone.reset({ position: new THREE.Vector3(0, 0, 1.4) }); // Z-up: start at 1.4m altitude

    // Initialize ETH controller with cave exploration gains
    this.controller = new EthController(this.params);
    
    // Moderate position gains for smooth cave navigation
    this.controller.Kp.set(5.0, 5.0, 8.0);
    this.controller.Kd.set(3.5, 3.5, 5.0);
    this.controller.Ki.set(0.1, 0.1, 0.2);
    
    // Conservative attitude gains for obstacle avoidance
    this.controller.KR.set(0.03, 0.03, 0.15);
    this.controller.Komega.set(0.015, 0.015, 0.06);
    
    // Limit aggressiveness in confined spaces
    this.controller.maxAcc = 8.0;
    this.controller.maxTiltAngle = 45 * Math.PI / 180;

    // Create body matching race/stationary demo (+Z is up, X is forward)
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

    this.scene.add(this.droneMesh);
  }

  _initHUD() {
    this.overlay = document.createElement('div');
    this.overlay.style.cssText =
      'position:absolute; top:8px; left:8px; background:rgba(6,9,18,0.85); color:#d6e3ff; padding:10px; font-family:"Fira Code", monospace; border:1px solid rgba(59,130,246,0.5); border-radius:8px; z-index:5;';
    this.overlay.innerHTML = `
      <div style="font-size:12px; color:#7dd3fc;">Cave Explorer</div>
      <div id="caveState" style="font-size:12px; margin-top:6px;">Exploring</div>
      <div id="caveCoverage" style="font-size:12px;">Coverage: 0%</div>
      <div id="caveFrontiers" style="font-size:12px;">Frontiers: 0</div>
      <div id="caveTarget" style="font-size:12px;">Target: none</div>
    `;
    this.container.style.position = 'relative';
    this.container.appendChild(this.overlay);

    this.cameraLabel = document.createElement('div');
    this.cameraLabel.style.cssText = 'font-size:12px; color:#cbd5f5; margin-top:6px;';
    this.cameraLabel.textContent = `Camera: ${this.cameraMode.toUpperCase()}`;
    this.overlay.appendChild(this.cameraLabel);

    this.debugPanel = document.createElement('div');
    this.debugPanel.style.cssText =
      'position:absolute; top:8px; right:8px; background:rgba(6,8,18,0.85); color:#e0f2fe; padding:10px; font-family:"Fira Code", monospace; border:1px solid rgba(34,211,238,0.35); border-radius:8px; z-index:6; display:none; min-width:220px;';
    this.debugPanel.innerHTML = `
      <div style="font-size:12px; color:#67e8f9; margin-bottom:4px;">Debug</div>
      <div id="dbgPos" style="font-size:12px;">Pos: 0, 0, 0</div>
      <div id="dbgAlt" style="font-size:12px;">Alt: 0.00 m</div>
      <div id="dbgVel" style="font-size:12px;">Vel: 0.00 m/s (0,0,0)</div>
      <div id="dbgRPM" style="font-size:12px;">RPM: 0 | 0 | 0 | 0</div>
      <div id="dbgThrust" style="font-size:12px;">Thrust: 0.0 N</div>
      <div id="dbgAtt" style="font-size:12px;">R/P/Y: 0 / 0 / 0</div>
      <div id="dbgAttErr" style="font-size:12px;">Att Err: 0.00</div>
      <div id="dbgFrontier" style="font-size:12px;">Frontiers: 0</div>
    `;
    this.container.appendChild(this.debugPanel);
  }

  _bindInputs() {
    window.addEventListener('keydown', (e) => {
      if (e.key === 'd' || e.key === 'D') {
        this.debugEnabled = !this.debugEnabled;
        if (this.debugPanel) this.debugPanel.style.display = this.debugEnabled ? 'block' : 'none';
      }
      if (e.key === 'c' || e.key === 'C') {
        if (this.cameraMode === 'chase') {
          this.cameraMode = 'overhead';
          this.activeCamera = this.overheadCamera;
        } else {
          this.cameraMode = 'chase';
          this.activeCamera = this.chaseCamera;
        }
        if (this.cameraLabel) this.cameraLabel.textContent = `Camera: ${this.cameraMode.toUpperCase()}`;
      }
    });
  }

  restart() {
    this.drone.reset({ position: new THREE.Vector3(0, 0, 1.4), orientation: new THREE.Quaternion() }); // Z-up: altitude 1.4m
    this.controller.reset();
    this.grid = new OccupancyGrid();
    this.target = new THREE.Vector3(0, 0, 1.4); // Z-up target
    this.path = [];
    this.pathIndex = 0;
    this.lastHits = [];
    this.frontierInfo = null;
    this.simTime = 0;
    this.state = 'EXPLORING';
    this.stuckTimer = 0;
    this.lastExploredPos = new THREE.Vector3(0, 0, 1.4);
    this.explorationTimeout = 5.0; // Seconds before considering stuck
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

  start() {
    this.resume(false);
  }

  destroy() {
    this.pause(false);
    this.container.innerHTML = '';
  }

  _loop(timestamp) {
    if (this.userPaused || this.visibilityPaused) return;
    if (this.lastFrame === null) this.lastFrame = timestamp;
    const dt = ((timestamp - this.lastFrame) / 1000) * this.timeScale;
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

    // Stuck detection - check if drone hasn't moved much
    const distMoved = this.drone.state.position.distanceTo(this.lastExploredPos);
    if (distMoved > 1.0) {
      // Made progress, reset stuck timer
      this.stuckTimer = 0;
      this.lastExploredPos.copy(this.drone.state.position);
    } else {
      this.stuckTimer += dt;
    }
    
    // Lidar scan at 10 Hz (faster for better responsiveness)
    if (this.simTime % 0.1 < dt) {
      this.lastHits = this.lidar.scan(this.cave, this.drone.state);
      
      updateGridFromLidar(this.grid, this.drone.state, this.lastHits);
      const coverage = this._coverage();
      this.overlay.querySelector('#caveCoverage').textContent = `Coverage: ${(coverage * 100).toFixed(1)}%`;
      
      // Check if stuck - pick random exploration target
      if (this.stuckTimer > this.explorationTimeout) {
        this.state = 'RANDOM_EXPLORATION';
        // Pick random point within cave bounds, away from current position
        const randomTarget = this._generateRandomExplorationPoint();
        this.frontierInfo = { point: randomTarget, clusterCount: 0, targetSize: 0 };
        this.path = [];
        this.pathIndex = 0;
        this.stuckTimer = 0;
      } else {
        // Normal frontier-based exploration
        this.frontierInfo = chooseFrontierTarget(this.grid, this.drone.state.position);
      }
      
      if (this.frontierInfo?.point) {
        // Ensure frontier target has safe altitude
        const minAltitude = this.floorHeight + 0.8;
        this.frontierInfo.point.z = Math.max(minAltitude, this.frontierInfo.point.z);
        
        const newPath = planPath(this.grid, this.drone.state.position, this.frontierInfo.point);
        if (newPath.length) {
          // Ensure all waypoints maintain safe altitude
          this.path = newPath.map(wp => {
            wp.z = Math.max(minAltitude, wp.z);
            return wp;
          });
          this.pathIndex = 0;
          this.state = 'EXPLORING';
        } else if (this.state !== 'RANDOM_EXPLORATION') {
          // Path planning failed, try direct approach
          this.path = [this.frontierInfo.point];
          this.pathIndex = 0;
        }
      } else if (this.state !== 'RANDOM_EXPLORATION') {
        // No frontiers found, pick random exploration point
        this.state = 'RANDOM_EXPLORATION';
        const randomTarget = this._generateRandomExplorationPoint();
        this.frontierInfo = { point: randomTarget, clusterCount: 0, targetSize: 0 };
        this.path = [randomTarget];
        this.pathIndex = 0;
      }
      
      if (this.frontierInfo) {
        this.overlay.querySelector('#caveFrontiers').textContent = `Frontiers: ${this.frontierInfo.clusterCount}`;
        this.overlay.querySelector('#caveTarget').textContent =
          `Target: (${this.frontierInfo.point.x.toFixed(1)}, ${this.frontierInfo.point.y.toFixed(1)}, ${this.frontierInfo.point.z.toFixed(1)})`;
      }
    }

    // Path following with obstacle avoidance
    const waypoint = this.path[this.pathIndex] || this.frontierInfo?.point || this.target;
    if (waypoint) {
      this.target.copy(waypoint);
      if (this.drone.state.position.distanceTo(waypoint) < 0.7 && this.pathIndex < this.path.length - 1) {
        this.pathIndex += 1;
      }
    }

    // Ensure target maintains safe altitude (Z-up convention)
    const targetPos = this.target.clone();
    const minAltitude = this.floorHeight + 0.8; // Floor + safety margin
    const maxAltitude = 4.0;
    targetPos.z = Math.max(minAltitude, Math.min(targetPos.z, maxAltitude)); // Keep between safe altitude and ceiling
    
    // Compute reactive obstacle avoidance force
    const avoidance = this._avoidanceFromHits();
    
    // Build target for controller
    const state = this.drone.getState();
    const targetObj = {
      position: targetPos,
      velocity: new THREE.Vector3(0, 0, 0),
      acceleration: avoidance.multiplyScalar(1.5), // Stronger avoidance response
      yaw: Math.atan2(targetPos.y - state.position.y, targetPos.x - state.position.x), // XY plane heading
    };
    
    // Compute control using ETH controller
    const result = this.controller.computeControl(state, targetObj, dt);
    this.drone.applyMotorCommands(result.motorCommands[0], result.motorCommands[1], result.motorCommands[2], result.motorCommands[3]);
    this.drone.step(dt);

    // Collision detection and response
    const postState = this.drone.getState();
    const minSafeAltitude = this.floorHeight + 0.5; // Safety margin above floor
    
    // Check floor collision
    if (postState.position.z < minSafeAltitude) {
      // Stop downward motion and push to safe altitude
      this.drone.state.v_W.z = Math.max(0, this.drone.state.v_W.z); // Stop falling
      this.drone.state.p_W.z = minSafeAltitude; // Teleport to safe altitude
      this.drone.state.v_W.multiplyScalar(0.7); // Dampen horizontal velocity
    }
    
    // Check obstacle collision
    if (this.cave.collide(postState.position)) {
      // Bounce back and push upward (+Z)
      this.drone.state.v_W.multiplyScalar(-0.3);
      this.drone.state.p_W.add(new THREE.Vector3(0, 0, 0.15)); // Push up in Z
    }

    this.overlay.querySelector('#caveState').textContent = `Exploring → (${this.target.x.toFixed(1)}, ${this.target.y.toFixed(1)}, ${this.target.z.toFixed(1)})`;
  }

  _coverage() {
    let known = 0;
    for (let i = 0; i < this.grid.grid.length; i++) {
      if (Math.abs(this.grid.grid[i]) > 0.01) known++;
    }
    return known / this.grid.grid.length;
  }

  _render() {
    if (this.cameraMode === 'chase') {
      // Third-person camera follows drone from behind and rotates with it
      // Offset in body frame: behind (negative X), centered (Y=0), slightly above (positive Z)
      const offsetBody = new THREE.Vector3(-1.2, 0, 0.6);
      
      // Transform offset from body frame to world frame using drone's orientation
      const offsetWorld = offsetBody.clone().applyQuaternion(this.drone.state.orientation);
      const targetPos = this.drone.state.position.clone().add(offsetWorld);
      this.chaseCamera.position.lerp(targetPos, 0.12); // Smooth follow
      
      // Look at a point ahead of the drone in its body frame
      const lookAheadBody = new THREE.Vector3(1.5, 0, 0.2); // Ahead (X), centered (Y), slightly up (Z)
      const lookAheadWorld = lookAheadBody.clone().applyQuaternion(this.drone.state.orientation);
      const lookTarget = this.drone.state.position.clone().add(lookAheadWorld);
      this.chaseCamera.lookAt(lookTarget);
      this.activeCamera = this.chaseCamera;
    } else {
      // Overhead view: Z position looking down
      this.overheadCamera.position.set(0, 0, 12); // Z-up: camera above (lowered for better view)
      this.overheadCamera.lookAt(new THREE.Vector3(0, 0, 0));
      this.activeCamera = this.overheadCamera;
    }
    this.droneMesh.position.copy(this.drone.state.position);
    this.droneMesh.quaternion.copy(this.drone.state.orientation);
    this._updateLidarVisualization(); // Update every frame for smooth display
    this._updateDebugPanel();
    this.renderer.render(this.scene, this.activeCamera || this.overheadCamera);
  }

  _updateDebugPanel() {
    if (!this.debugEnabled || !this.debugPanel) return;
    const st = this.drone.state;
    const velMag = st.velocity.length();
    const rpm = st.motorRPM || [0, 0, 0, 0];
    const euler = new THREE.Euler().setFromQuaternion(st.orientationQuat || st.orientation);
    const desired = this.drone.desiredOrientationQuat || new THREE.Quaternion();
    const qErr = desired.clone().multiply((st.orientationQuat || st.orientation).clone().invert());
    const attErrMag = 2 * new THREE.Vector3(qErr.x, qErr.y, qErr.z).length();

    this.debugPanel.querySelector('#dbgPos').textContent =
      `Pos: ${st.position.x.toFixed(2)}, ${st.position.y.toFixed(2)}, ${st.position.z.toFixed(2)}`;
    this.debugPanel.querySelector('#dbgAlt').textContent = `Alt: ${st.position.z.toFixed(2)} m`; // Z-up: altitude is Z
    this.debugPanel.querySelector('#dbgVel').textContent =
      `Vel: ${velMag.toFixed(2)} m/s (${st.velocity.x.toFixed(2)}, ${st.velocity.y.toFixed(2)}, ${st.velocity.z.toFixed(2)})`;
    this.debugPanel.querySelector('#dbgRPM').textContent = `RPM: ${rpm.map((r) => r.toFixed(0)).join(' | ')}`;
    this.debugPanel.querySelector('#dbgThrust').textContent = `Thrust: ${st.totalThrust.toFixed(2)} N`;
    this.debugPanel.querySelector('#dbgAtt').textContent =
      `R/P/Y: ${(THREE.MathUtils.radToDeg(euler.x)).toFixed(1)} / ${(THREE.MathUtils.radToDeg(euler.y)).toFixed(1)} / ${(THREE.MathUtils.radToDeg(euler.z)).toFixed(1)}`;
    this.debugPanel.querySelector('#dbgAttErr').textContent = `Att Err: ${attErrMag.toFixed(3)}`;
    if (this.frontierInfo) {
      this.debugPanel.querySelector('#dbgFrontier').textContent =
        `Frontiers: ${this.frontierInfo.clusterCount} | Cluster size ${this.frontierInfo.targetSize}`;
    }
  }

  _avoidanceFromHits() {
    if (!this.lastHits || !this.lastHits.length) return new THREE.Vector3();
    
    // Compute repulsive force from nearby obstacles
    const avoidance = this.lastHits.reduce((acc, h) => {
      // Stronger repulsion for closer obstacles
      const strength = Math.pow(Math.max(0, 1 - h.distance / this.lidar.maxRange), 2);
      return acc.add(h.direction.clone().multiplyScalar(-strength * 5.0));
    }, new THREE.Vector3());
    
    return avoidance.clampLength(0, 3.5);
  }
  
  _generateRandomExplorationPoint() {
    // Generate random point within cave bounds, away from current position
    const minAltitude = this.floorHeight + 0.8;
    const maxAltitude = 3.5;
    const caveBounds = 10; // Stay within reasonable bounds
    
    let x, y, attempts = 0;
    const currentPos2d = new THREE.Vector2(this.drone.state.position.x, this.drone.state.position.y);
    
    // Find point at least 3m away from current position
    do {
      x = (Math.random() - 0.5) * caveBounds * 2;
      y = (Math.random() - 0.5) * caveBounds * 2;
      attempts++;
    } while (new THREE.Vector2(x, y).distanceTo(currentPos2d) < 3.0 && attempts < 20);
    
    const z = minAltitude + Math.random() * (maxAltitude - minAltitude);
    return new THREE.Vector3(x, y, z);
  }
  
  _updateLidarVisualization() {
    // Clear previous visualization
    this.lidarRayGroup.clear();
    
    if (!this.lastHits || !this.lastHits.length) return;
    
    const dronePos = this.drone.state.position;
    
    // Separate hits from misses
    const actualHits = this.lastHits.filter(h => h.hit);
    const misses = this.lastHits.filter(h => !h.hit);
    
    // Group actual hits by proximity zones
    const closeHits = actualHits.filter(h => h.distance < 2.5);
    const midHits = actualHits.filter(h => h.distance >= 2.5 && h.distance < 5.0);
    const farHits = actualHits.filter(h => h.distance >= 5.0);
    
    // Draw detection cone for actual hits
    const drawCone = (hits, color, opacity) => {
      if (hits.length === 0) return;
      
      // Create point cloud for the hit area
      const points = hits.map(h => h.point.clone());
      const geometry = new THREE.BufferGeometry().setFromPoints(points);
      const material = new THREE.PointsMaterial({ 
        color: color,
        size: 0.15,
        opacity: opacity,
        transparent: true,
        sizeAttenuation: true,
      });
      const pointCloud = new THREE.Points(geometry, material);
      this.lidarRayGroup.add(pointCloud);
      
      // Draw ALL rays to show full cone structure
      for (let i = 0; i < hits.length; i++) {
        const hit = hits[i];
        const lineMat = new THREE.LineBasicMaterial({ 
          color: color,
          opacity: opacity * 0.5,
          transparent: true,
        });
        const linePoints = [dronePos.clone(), hit.point.clone()];
        const lineGeo = new THREE.BufferGeometry().setFromPoints(linePoints);
        const line = new THREE.Line(lineGeo, lineMat);
        this.lidarRayGroup.add(line);
      }
    };
    
    // Draw misses (rays that go to max range) in dim cyan to show cone shape
    if (misses.length > 0) {
      for (let i = 0; i < misses.length; i++) {
        const miss = misses[i];
        const lineMat = new THREE.LineBasicMaterial({ 
          color: 0x00ffff,
          opacity: 0.15,
          transparent: true,
        });
        const linePoints = [dronePos.clone(), miss.point.clone()];
        const lineGeo = new THREE.BufferGeometry().setFromPoints(linePoints);
        const line = new THREE.Line(lineGeo, lineMat);
        this.lidarRayGroup.add(line);
      }
    }
    
    // Red for danger zone (<2.5m), yellow for caution (2.5-5m), green for safe (>5m)
    drawCone(closeHits, 0xff3333, 0.8);
    drawCone(midHits, 0xffaa00, 0.6);
    drawCone(farHits, 0x33ff33, 0.4);
  }
}
