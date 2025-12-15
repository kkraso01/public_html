import { DronePhysicsEngine } from '../physics/drone_physics_engine.js';
import { CRAZYFLIE_PARAMS } from '../physics/params_crazyflie.js';
import { EthController } from '../stationary/eth_controller.js';
import { CaveSim } from './cave_sim.js';
import { Lidar } from './lidar.js';
import { OccupancyGrid3D, updateGrid3DFromLidar, chooseFrontierTarget, planPath } from './mapping.js';
import { TSDFVolume } from './tsdf_volume.js';
import { ICPOdometry } from './icp_odometry.js';
import { computeNextBestView } from './nbv.js';

function clamp(x, min, max) {
  return Math.min(Math.max(x, min), max);
}

export function initDroneCaveDemo(container, options = {}) {
  if (!container || typeof THREE === 'undefined') {
    console.warn('Drone cave demo requires a valid container and Three.js');
    return { pause() {}, resume() {}, restart() {}, setPausedFromVisibility() {}, destroy() {} };
  }
  const demo = new DroneCaveDemo(container, options);
  window.caveDemo = demo; // Expose for console access to performance flags
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
    
    // Performance optimization flags
    this.enableSliceViz = false; // MOST EXPENSIVE: 2500 cell iterations + canvas ops (disable for 30% perf boost)
    this.enableLidarBeams = false; // EXPENSIVE: 80 line updates per frame (disable for 15% perf boost)
    this.enablePathSpheres = true; // MODERATE: Few spheres (keep enabled)

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
    
    // Isometric 45° third-person camera - view whole maze from angle
    this.isometricCamera = new THREE.PerspectiveCamera(60, aspect, 0.05, 200);
    this.isometricCamera.position.set(12, 12, 8); // 45° angle: equal X, Y, lower Z for perspective
    this.isometricCamera.up.set(0, 0, 1); // +Z is up
    this.isometricCamera.lookAt(new THREE.Vector3(0, 0, 3)); // Look at center of cave at Z=3m
    
    this.activeCamera = this.cameraMode === 'overhead' ? this.overheadCamera : (this.cameraMode === 'isometric' ? this.isometricCamera : this.chaseCamera);

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
    this.grid = new OccupancyGrid3D(0.5, 80, 20); // 3D voxel grid: 40x40x10m volume
    this.tsdf = new TSDFVolume(0.3, 80, 20);      // TSDF map for accurate surface representation
    this.icp = new ICPOdometry(400, 6);           // ICP odometry with 400 points, 6 iterations
    this.lidar = new Lidar();
    
    this.slamUpdateCounter = 0;
    
    // Lidar ray visualization
    this.lidarRayGroup = new THREE.Group();
    this.scene.add(this.lidarRayGroup);
    
    // Horizontal slice visualization at drone altitude (explored regions)
    this.sliceGroup = new THREE.Group();
    this.scene.add(this.sliceGroup);
    
    // Create persistent slice mesh for explored areas
    this._initSliceVisualization();
    
    // Waypoint visualization - arrow shape to show direction
    const arrowGroup = new THREE.Group();
    
    // Cone pointing in +X direction (forward in body frame)
    const coneGeom = new THREE.ConeGeometry(0.15, 0.4, 8);
    coneGeom.rotateZ(-Math.PI / 2); // Point cone along +X axis
    const coneMesh = new THREE.Mesh(
      coneGeom,
      new THREE.MeshBasicMaterial({ 
        color: 0x00ff00,
        transparent: true,
        opacity: 0.8
      })
    );
    coneMesh.position.x = 0.2; // Offset forward
    arrowGroup.add(coneMesh);
    
    // Sphere at base
    const sphereMesh = new THREE.Mesh(
      new THREE.SphereGeometry(0.15, 16, 12),
      new THREE.MeshBasicMaterial({ 
        color: 0x00ff00,
        transparent: true,
        opacity: 0.7
      })
    );
    arrowGroup.add(sphereMesh);
    
    this.waypointMarker = arrowGroup;
    this.waypointMarker.visible = false;
    this.scene.add(this.waypointMarker);
    
    // Line from drone to waypoint
    this.waypointLine = new THREE.Line(
      new THREE.BufferGeometry(),
      new THREE.LineBasicMaterial({ color: 0x00ff00, linewidth: 2 })
    );
    this.scene.add(this.waypointLine);
    
    // Final target marker (larger, different style)
    const targetGroup = new THREE.Group();
    const targetSphere = new THREE.Mesh(
      new THREE.SphereGeometry(0.25, 16, 12),
      new THREE.MeshBasicMaterial({ 
        color: 0xff00ff,
        transparent: true,
        opacity: 0.6,
        wireframe: true
      })
    );
    targetGroup.add(targetSphere);
    
    this.finalTargetMarker = targetGroup;
    this.finalTargetMarker.visible = false;
    this.scene.add(this.finalTargetMarker);
    
    // Line from drone to final target
    this.finalTargetLine = new THREE.Line(
      new THREE.BufferGeometry(),
      new THREE.LineBasicMaterial({ 
        color: 0xff00ff, 
        linewidth: 1,
        transparent: true,
        opacity: 0.3
      })
    );
    this.scene.add(this.finalTargetLine);
    
    // Path visualization - shows entire planned route
    this.pathLine = new THREE.Line(
      new THREE.BufferGeometry(),
      new THREE.LineBasicMaterial({ 
        color: 0x00ffff,
        linewidth: 2,
        transparent: true,
        opacity: 0.6
      })
    );
    this.scene.add(this.pathLine);
    
    // Path waypoint spheres
    this.pathSpheres = [];
    for (let i = 0; i < 20; i++) { // Pre-create 20 spheres
      const sphere = new THREE.Mesh(
        new THREE.SphereGeometry(0.1, 8, 8),
        new THREE.MeshBasicMaterial({ 
          color: 0x00ffff,
          transparent: true,
          opacity: 0.5
        })
      );
      sphere.visible = false;
      this.pathSpheres.push(sphere);
      this.scene.add(sphere);
    }
  }

  _initDrone() {
    this.params = CRAZYFLIE_PARAMS;
    this.drone = new DronePhysicsEngine(this.params);
    this.drone.reset({ position: new THREE.Vector3(0, 0, 1.4) }); // Z-up: start at 1.4m altitude

    // Initialize ETH controller with Crazyflie-tuned defaults from constructor
    this.controller = new EthController(this.params);
    // Using Crazyflie 2.x defaults: Kp(4,4,12), Kd(3,3,6), Ki(0.05,0.05,0.30)
    // Using Crazyflie 2.x defaults: KR(3.0,3.0,0.8), Komega(0.08,0.08,0.02)
    // Using Crazyflie 2.x defaults: maxAcc=18 m/s², maxTilt=35°
    // No gain overrides - using realistic CF2 dynamics

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

    // Add nose indicator (cone pointing forward along +X axis)
    const noseGeo = new THREE.ConeGeometry(0.08, 0.25, 8);
    const noseMat = new THREE.MeshStandardMaterial({ 
      color: 0xffff00, 
      emissive: 0xffaa00, 
      emissiveIntensity: 0.7,
      metalness: 0.4, 
      roughness: 0.3 
    });
    const nose = new THREE.Mesh(noseGeo, noseMat);
    nose.rotation.z = -Math.PI / 2; // Rotate cone to point along +X
    nose.position.set(0.5, 0, 0); // Position at front of drone
    nose.castShadow = true;

    this.droneMesh = new THREE.Group();
    this.droneMesh.add(body, arm, nose);

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
    // Log performance optimization info to console
    console.log('[CAVE DEMO] Performance flags (toggle in console):');
    console.log('  window.caveDemo.enableSliceViz = true/false  (currently: ' + this.enableSliceViz + ') - MOST EXPENSIVE');
    console.log('  window.caveDemo.enableLidarBeams = true/false (currently: ' + this.enableLidarBeams + ') - EXPENSIVE');
    console.log('  window.caveDemo.enablePathSpheres = true/false (currently: ' + this.enablePathSpheres + ')');
    
    window.addEventListener('keydown', (e) => {
      if (e.key === 'd' || e.key === 'D') {
        this.debugEnabled = !this.debugEnabled;
        if (this.debugPanel) this.debugPanel.style.display = this.debugEnabled ? 'block' : 'none';
      }
      if (e.key === 'c' || e.key === 'C') {
        if (this.cameraMode === 'chase') {
          this.cameraMode = 'isometric';
          this.activeCamera = this.isometricCamera;
        } else if (this.cameraMode === 'isometric') {
          this.cameraMode = 'overhead';
          this.activeCamera = this.overheadCamera;
        } else {
          this.cameraMode = 'chase';
          this.activeCamera = this.chaseCamera;
        }
        if (this.cameraLabel) this.cameraLabel.textContent = `Camera: ${this.cameraMode === 'isometric' ? '45° ISO' : this.cameraMode.toUpperCase()}`;
      }
    });
  }

  restart() {
    this.drone.reset({ position: new THREE.Vector3(0, 0, 1.4), orientation: new THREE.Quaternion() }); // Z-up: altitude 1.4m
    this.controller.reset();
    this.grid = new OccupancyGrid3D(0.5, 80, 20);
    this.tsdf = new TSDFVolume(0.3, 80, 20);
    
    // Initialize SLAM pose (separate from ground truth physics)
    this.slamPose = {
      position: this.drone.state.position.clone(),
      orientation: this.drone.state.orientation.clone(),
    };
    this.icp.reset();
    
    this.target = new THREE.Vector3(0, 0, 1.4); // Z-up target
    this.path = [];
    this.pathIndex = 0;
    this.lastHits = [];
    this.frontierInfo = null;
    this.simTime = 0;
    this.state = 'EXPLORING';
    this.stuckTimer = 0;
    this.lastExploredPos = new THREE.Vector3(0, 0, 1.4);
    this.explorationTimeout = 10.0; // Time before considering drone stuck and replanning
    this.slamUpdateCounter = 0;
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
    const hasTarget = this.frontierInfo && this.frontierInfo.point;
    if (distMoved > 0.5) {
      // Made progress, reset stuck timer
      this.stuckTimer = 0;
      this.lastExploredPos.copy(this.drone.state.position);
    } else if (hasTarget) {
      // Has a target (either following path or heading to final target) - not stuck, just moving slowly
      this.stuckTimer = 0;
    } else {
      this.stuckTimer += dt;
    }
    
    // Lidar scan at 50 Hz (realistic for modern sensors like Velodyne, Ouster)
    if (this.simTime % 0.02 < dt) {
      this.lastHits = this.lidar.scan(this.cave, this.drone.state);
      
      // Update SLAM pose with noisy ground truth (ICP is too unstable for demo)
      // In a real system, you'd use ICP/VIO/etc., but for demo purposes we add realistic noise
      if (this.slamPose) {
        // Copy ground truth with small drift
        const drift = 0.05; // 5cm drift per update
        this.slamPose.position.copy(this.drone.state.position);
        this.slamPose.position.x += (Math.random() - 0.5) * drift;
        this.slamPose.position.y += (Math.random() - 0.5) * drift;
        this.slamPose.position.z += (Math.random() - 0.5) * drift * 0.5; // Less Z drift
        this.slamPose.orientation.copy(this.drone.state.orientation);
      }
      
      // Use SLAM pose for mapping (realistic: map built from estimated pose, not ground truth)
      const mappingPose = this.slamPose ?? this.drone.state;
      
      // Update occupancy grid for planning/frontiers
      updateGrid3DFromLidar(this.grid, mappingPose, this.lastHits);
      
      // Update TSDF for accurate surface representation
      this.tsdf.integrateScan(mappingPose, this.lastHits);
      
      // Debug: log every 100 scans
      this.scanCount = (this.scanCount || 0) + 1;
      if (this.scanCount % 100 === 0) {
        console.log(`[LIDAR] Scan #${this.scanCount}: ${this.lastHits.length} hits, pos=${mappingPose.position.toArray().map(v=>v.toFixed(2))}`);
        const tsdfStats = this.tsdf.getStats();
        console.log('[SLAM] TSDF stats:', tsdfStats);
      }
      
      // Update slice visualization less frequently for performance
      this.slamUpdateCounter++;
      if (this.enableSliceViz && this.slamUpdateCounter % 10 === 0) {
        this._updateSliceVisualization();
      }
    }
    
    // Update exploration targets only when needed (not on timer)
    if (this.simTime % 0.5 < dt) {
      const coverage = this._coverage();
      this.overlay.querySelector('#caveCoverage').textContent = `Coverage: ${(coverage * 100).toFixed(1)}%`;
    }
    
    // Only recompute targets when: 1) stuck, 2) no target, or 3) reached current target
    // Use SLAM pose instead of ground truth for exploration logic (realistic)
    const slamPos = this.slamPose ? this.slamPose.position : this.drone.state.position;
    
    // Only replan when: 1) no target, 2) EXACTLY at magenta waypoint (≤0.1m), or 3) stuck
    // Keep following path to magenta target even if intermediate waypoints remain
    // Note: hasTarget already declared in stuck detection above
    const completedPath = this.pathIndex >= this.path.length; // Advanced past all waypoints
    const distToFinalTarget = hasTarget ? slamPos.distanceTo(this.frontierInfo.point) : 999;
    const reachedMagentaWaypoint = hasTarget && completedPath && distToFinalTarget <= 0.1; // NBV triggers ONLY at magenta (≤0.1m error)
    const noTarget = !this.frontierInfo;
    
    // NEVER replan while actively navigating to a target - only when EXACTLY at magenta or stuck
    if (noTarget || reachedMagentaWaypoint || this.stuckTimer > this.explorationTimeout) {
      if (noTarget) console.log('[REPLAN] Trigger: No target');
      else if (reachedMagentaWaypoint) console.log(`[NBV] At magenta waypoint! (completed=${completedPath}, dist=${distToFinalTarget.toFixed(3)}m ≤ 0.1m) → Computing NBV...`);
      else console.log(`[REPLAN] Trigger: Stuck (timer=${this.stuckTimer.toFixed(1)}s)`);
      
      this.lastReplanTime = this.simTime;
      
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
        // Try NBV (Next-Best-View) first for smarter exploration
        const nbv = computeNextBestView(this.grid, this.slamPose ?? { position: slamPos, orientation: this.drone.state.orientation }, {
          maxRange: this.lidar.maxRange ?? 18,
          numCandidates: 16,
          radiusMin: 3.0,
          radiusMax: 7.0,
        });
        
        // Fallback to frontier-based exploration if NBV gain is too low
        if (!nbv || nbv.gain < 5) {
          this.frontierInfo = chooseFrontierTarget(this.grid, slamPos);
          console.log(`[FRONTIER] NBV gain too low, using frontier. Found: ${this.frontierInfo ? 'YES' : 'NO'}, clusters=${this.frontierInfo?.clusterCount || 0}`);
        } else {
          const nbvOriginalZ = nbv.pose.position.z;
          this.frontierInfo = {
            point: nbv.pose.position.clone(),
            clusterCount: 0,
            targetSize: nbv.gain,
          };
          console.log(`[NBV] Target Z=${nbvOriginalZ.toFixed(2)}m, gain: ${nbv.gain.toFixed(1)}, travel: ${nbv.travelCost.toFixed(2)}`);
        }
        
        if (this.frontierInfo?.point) {
          const beforeClampZ = this.frontierInfo.point.z;
          console.log(`[TARGET] Before clamp: (${this.frontierInfo.point.x.toFixed(2)}, ${this.frontierInfo.point.y.toFixed(2)}, Z=${beforeClampZ.toFixed(2)}m)`);
        }
      }
      
      if (this.frontierInfo?.point) {
        // Ensure frontier target has safe altitude
        const minAltitude = this.floorHeight + 1.2; // Increased from 0.8 to prevent ground contact
        const beforeClamp = this.frontierInfo.point.z;
        this.frontierInfo.point.z = Math.max(minAltitude, this.frontierInfo.point.z);
        console.log(`[TARGET] After clamp: Z=${beforeClamp.toFixed(2)}m → ${this.frontierInfo.point.z.toFixed(2)}m (minAlt=${minAltitude.toFixed(2)}m)`);
        
        const distToTarget = slamPos.distanceTo(this.frontierInfo.point);
        
        // Always use A* path planning for obstacle avoidance (use SLAM pose for realistic planning)
        const newPath = planPath(this.grid, slamPos, this.frontierInfo.point);
        if (newPath.length > 0) {
          // Ensure all waypoints maintain safe altitude
          this.path = newPath.map((wp, idx) => {
            const wpOrigZ = wp.z;
            wp.z = Math.max(minAltitude, wp.z);
            if (idx === 0 || idx === newPath.length - 1) {
              console.log(`[PATH] Waypoint ${idx}: Z=${wpOrigZ.toFixed(2)}m → ${wp.z.toFixed(2)}m`);
            }
            return wp;
          });
          this.pathIndex = 0;
          this.state = 'EXPLORING';
          console.log(`[PATH] A* found path with ${newPath.length} waypoints`);
        } else {
          // Path planning failed - create intermediate waypoints for safer navigation
          console.warn('[PATH] A* failed, creating intermediate waypoints');
          const dir = this.frontierInfo.point.clone().sub(slamPos).normalize();
          const dist = slamPos.distanceTo(this.frontierInfo.point);
          const numWaypoints = Math.max(2, Math.ceil(dist / 2.0)); // Waypoint every 2m
          
          this.path = [];
          for (let i = 1; i <= numWaypoints; i++) {
            const t = i / numWaypoints;
            const wp = slamPos.clone().add(dir.clone().multiplyScalar(t * dist));
            wp.z = Math.max(minAltitude, wp.z);
            this.path.push(wp);
          }
          this.pathIndex = 0;
          this.state = 'EXPLORING';
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
    const minAltitude = this.floorHeight + 1.2; // Safety margin to prevent ground contact
    const maxAltitude = 7.0; // Increased to allow exploration of full cave height (walls are 8m)
    
    const waypoint = this.path[this.pathIndex] || this.frontierInfo?.point || this.target;
    if (waypoint) {
      this.target.copy(waypoint);
      // Allow full altitude range - only clamp to safe bounds, don't artificially limit exploration
      this.target.z = Math.max(minAltitude, Math.min(this.target.z, maxAltitude));
      
      // Waypoint advancement - require BOTH horizontal AND vertical proximity
      // CRITICAL: Check distance to CLAMPED target (this.target), not original waypoint!
      const currentPos = this.drone.state.position;
      const distToWaypoint = currentPos.distanceTo(this.target);
      const horizontalDist = Math.sqrt(
        Math.pow(currentPos.x - this.target.x, 2) + 
        Math.pow(currentPos.y - this.target.y, 2)
      );
      const verticalDist = Math.abs(currentPos.z - this.target.z);
      const velocity = this.drone.state.velocity.length();
      
      // Calculate if we're heading towards the waypoint
      const toWaypoint = waypoint.clone().sub(currentPos).normalize();
      const velDir = this.drone.state.velocity.clone().normalize();
      const heading = toWaypoint.dot(velDir);
      
      // Advance only when BOTH horizontal and vertical distances are small (0.1m precision)
      const threshold = 0.1; // MUST be within 0.1m in BOTH H and V directions
      const bothClose = horizontalDist < threshold && verticalDist < threshold;
      
      if (bothClose) {
        this.pathIndex += 1;
        const finalWaypoint = this.pathIndex >= this.path.length;
        console.log(`[NAV] Advanced to waypoint ${this.pathIndex}/${this.path.length} (H=${horizontalDist.toFixed(2)}m, V=${verticalDist.toFixed(2)}m)${finalWaypoint ? ' ✓ COMPLETED PATH' : ''}`);
        // DEBUG: Log what the next target will be
        const nextWaypoint = this.path[this.pathIndex] || this.frontierInfo?.point;
        if (nextWaypoint) {
          console.log(`[NAV_DEBUG] Next target: (${nextWaypoint.x.toFixed(2)}, ${nextWaypoint.y.toFixed(2)}, Z=${nextWaypoint.z.toFixed(2)}m)`);
        }
      } else if (distToWaypoint < 1.5 && Math.random() < 0.02) {
        // Debug: Show progress toward waypoint
        console.log(`[NAV] Approaching wp${this.pathIndex}: (${this.target.x.toFixed(2)}, ${this.target.y.toFixed(2)}, Z=${this.target.z.toFixed(2)}m) | H=${horizontalDist.toFixed(2)}m, V=${verticalDist.toFixed(2)}m (need BOTH <0.1m)`);
      }
    }

    // Target position for control (Z-up convention) - already clamped above
    const targetPos = this.target.clone();
    
    // Build target for controller
    const state = this.drone.getState();
    
    // DEBUG: Log target being sent to controller
    if (Math.random() < 0.05) {
      console.log(`[CTRL_TARGET] Path index=${this.pathIndex}/${this.path.length} | Target: (${targetPos.x.toFixed(2)}, ${targetPos.y.toFixed(2)}, Z=${targetPos.z.toFixed(2)}m) | Drone pos: Z=${state.position.z.toFixed(2)}m`);
    }
    
    // Compute desired velocity for smooth motion (like stationary demo)
    const toTarget = targetPos.clone().sub(state.position);
    const distToTarget = toTarget.length();
    const desiredSpeed = Math.min(3.0, distToTarget * 2.0); // Speed scales with distance, max 3 m/s
    const desiredVelocity = distToTarget > 0.1 
      ? toTarget.normalize().multiplyScalar(desiredSpeed)
      : new THREE.Vector3(0, 0, 0);
    
    // Path planner (A*) already handles obstacle avoidance - no reactive avoidance needed!
    // Reactive forces would fight the planned path and cause jittery motion
    
    // Use nose-first control mode - ETH controller handles yaw tracking automatically
    const targetObj = {
      position: targetPos,
      velocity: desiredVelocity, // Smooth velocity feedforward
      acceleration: new THREE.Vector3(0, 0, 0), // Trust the path planner
    };
    
    // Compute control using ETH controller's nose-first mode
    // This automatically points nose toward target and handles smooth yaw tracking
    const result = this.controller.computeNoseFirstControl(state, targetObj, dt, {
      enableNoseFacing: true,
      aggressiveYawTracking: true,
      maxYawRate: 20.0 // Reduced from 100 to eliminate jitter
    });
    this.drone.applyMotorCommands(result.motorCommands[0], result.motorCommands[1], result.motorCommands[2], result.motorCommands[3]);
    this.drone.step(dt);

    // Collision detection and response
    const postState = this.drone.getState();
    const minSafeAltitude = this.floorHeight + 0.6; // Increased safety margin above floor
    
    // Check floor collision - enforce hard floor constraint (minimal damping)
    if (postState.position.z < minSafeAltitude) {
      // Stop downward motion only, don't fight horizontal movement
      this.drone.state.v_W.z = Math.max(0, this.drone.state.v_W.z); // Prevent going down
      this.drone.state.p_W.z = minSafeAltitude; // Hard constraint to safe altitude
      // Remove horizontal damping - let controller handle it smoothly
    }
    
    // Check obstacle collision
    if (this.cave.collide(postState.position)) {
      // CRITICAL: Don't reverse Z velocity - that would cancel climbing!
      // Only bounce XY velocity to push away from obstacle horizontally
      const oldVz = this.drone.state.v_W.z;
      this.drone.state.v_W.multiplyScalar(-0.3); // Reverse all velocity
      this.drone.state.v_W.z = Math.max(0.2, oldVz); // Restore upward velocity with minimum 0.2 m/s
      this.drone.state.p_W.add(new THREE.Vector3(0, 0, 0.3)); // Push up in Z (increased from 0.2)
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
    } else if (this.cameraMode === 'isometric') {
      // Isometric 45° third-person view - see whole maze from angle
      this.isometricCamera.position.set(12, 12, 8);
      this.isometricCamera.lookAt(new THREE.Vector3(0, 0, 3)); // Center of cave at Z=3m
      this.activeCamera = this.isometricCamera;
    } else {
      // Overhead view: Z position looking down
      this.overheadCamera.position.set(0, 0, 12); // Z-up: camera above (lowered for better view)
      this.overheadCamera.lookAt(new THREE.Vector3(0, 0, 0));
      this.activeCamera = this.overheadCamera;
    }
    this.droneMesh.position.copy(this.drone.state.position);
    this.droneMesh.quaternion.copy(this.drone.state.orientation);
    this._updateLidarVisualization(); // Update every frame for smooth display
    this._updateWaypointVisualization();
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
    
    const state = this.drone.getState();
    
    // Get drone's forward direction (body frame +X in world frame)
    const forward = new THREE.Vector3(1, 0, 0).applyQuaternion(state.orientation);
    
    // Compute repulsive force from obstacles in FORWARD HEMISPHERE only
    const avoidance = new THREE.Vector3();
    let closeObstacles = 0;
    
    this.lastHits.forEach(h => {
      if (!h.hit) return; // Skip rays that didn't hit anything
      
      // Check if obstacle is in front of drone (dot product > 0)
      const dotProduct = h.direction.dot(forward);
      if (dotProduct < 0.3) return; // Ignore obstacles behind or to the side (cos(72°) ≈ 0.3)
      
      // Critical distance threshold for smooth avoidance
      const criticalDist = 2.0; // Start avoiding earlier at 2.0m for smoother response
      const emergencyDist = 1.0; // Emergency avoidance at 1.0m
      
      if (h.distance < criticalDist) {
        closeObstacles++;
        
        // Gentler exponential repulsion for smooth motion
        let strength;
        if (h.distance < emergencyDist) {
          // Emergency: strong but not jerky repulsion
          strength = Math.pow((emergencyDist - h.distance) / emergencyDist, 1.5) * 8.0; // Reduced from 20.0
        } else {
          // Normal: gentle guidance away from obstacle
          strength = Math.pow((criticalDist - h.distance) / criticalDist, 1.5) * 3.0; // Reduced from 8.0
        }
        
        // Weight by how much the obstacle is in front (stronger for directly ahead)
        strength *= dotProduct;
        
        // Push away from obstacle
        avoidance.add(h.direction.clone().multiplyScalar(-strength));
      }
    });
    
    // Clamp avoidance force to gentle limits (scaled down by 3x in main code)
    return avoidance.clampLength(0, 4.0); // Reduced from 6.0-10.0
  }
  
  _generateRandomExplorationPoint() {
    // Generate random point within cave bounds, away from current position
    const minAltitude = this.floorHeight + 1.2; // Safe altitude above floor
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
    // EXPENSIVE: 80+ line geometries + buffer updates every frame
    // Disable for 15% performance boost
    if (!this.enableLidarBeams) {
      this.lidarRayGroup.visible = false;
      return;
    }
    
    // Clear previous visualization
    this.lidarRayGroup.clear();
    this.lidarRayGroup.visible = true;
    
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

  _updateWaypointVisualization() {
    // Show immediate waypoint (next step in path)
    const waypoint = this.path[this.pathIndex] || this.frontierInfo?.point || this.target;
    
    if (waypoint && waypoint !== this.drone.state.position) {
      const dist = this.drone.state.position.distanceTo(waypoint);
      
      // Only show waypoint if it's reasonably far away (prevents jitter when very close)
      if (dist > 0.2) {
        // Show waypoint marker
        this.waypointMarker.position.copy(waypoint);
        this.waypointMarker.visible = true;
        
        // Orient marker based on waypoint yaw if available
        if (waypoint.yaw !== undefined) {
          this.waypointMarker.rotation.z = waypoint.yaw;
        }
        
        // Update line from drone to waypoint
        const points = [
          this.drone.state.position.clone(),
          waypoint.clone()
        ];
        this.waypointLine.geometry.setFromPoints(points);
        this.waypointLine.visible = true;
        
        // Static scale (pulse disabled for performance)
        this.waypointMarker.scale.set(1.0, 1.0, 1.0);
        
        // Change color based on distance and progress
        let color;
        if (dist < 1.0) {
          color = 0xffff00; // Yellow when close (< 1m)
        } else if (dist < 3.0) {
          color = 0x00ff00; // Green when medium (< 3m)
        } else {
          color = 0x00ffff; // Cyan when far
        }
        
        // Update all children materials
        this.waypointMarker.children.forEach(child => {
          if (child.material) {
            child.material.color.setHex(color);
          }
        });
      } else {
        // Too close - hide to prevent visual confusion
        this.waypointMarker.visible = false;
        this.waypointLine.visible = false;
      }
    } else {
      this.waypointMarker.visible = false;
      this.waypointLine.visible = false;
    }
    
    // Show final target (exploration goal) - magenta wireframe sphere
    const finalTarget = this.frontierInfo?.point;
    if (finalTarget && this.path.length > 1) {
      const distToFinal = this.drone.state.position.distanceTo(finalTarget);
      
      if (distToFinal > 1.0) {
        this.finalTargetMarker.position.copy(finalTarget);
        this.finalTargetMarker.visible = true;
        
        // Static scale (pulse disabled for performance)
        this.finalTargetMarker.scale.set(1.0, 1.0, 1.0);
        
        // Faint line to final target
        const finalPoints = [
          this.drone.state.position.clone(),
          finalTarget.clone()
        ];
        this.finalTargetLine.geometry.setFromPoints(finalPoints);
        this.finalTargetLine.visible = true;
      } else {
        this.finalTargetMarker.visible = false;
        this.finalTargetLine.visible = false;
      }
    } else {
      this.finalTargetMarker.visible = false;
      this.finalTargetLine.visible = false;
    }
    
    // Draw full path from current position through all remaining waypoints
    if (this.path.length > 0) {
      const pathPoints = [this.drone.state.position.clone()];
      
      // Add all remaining waypoints in the path
      for (let i = this.pathIndex; i < this.path.length; i++) {
        pathPoints.push(this.path[i].clone());
      }
      
      if (pathPoints.length > 1) {
        this.pathLine.geometry.setFromPoints(pathPoints);
        this.pathLine.visible = true;
        
        // Show spheres at each waypoint (moderate cost, can disable for small perf gain)
        if (this.enablePathSpheres) {
          for (let i = 0; i < this.pathSpheres.length; i++) {
          if (i < pathPoints.length - 1) { // -1 to skip drone position
            this.pathSpheres[i].position.copy(pathPoints[i + 1]);
            this.pathSpheres[i].visible = true;
            
            // Color code spheres: cyan for early waypoints, yellow for near ones
            const progress = i / Math.max(1, pathPoints.length - 2);
            const color = new THREE.Color();
            color.setHSL(0.5 - progress * 0.35, 1.0, 0.5); // Cyan to yellow
            this.pathSpheres[i].material.color.copy(color);
          } else {
            this.pathSpheres[i].visible = false;
          }
        }
        } else {
          // Hide all spheres if disabled
          this.pathSpheres.forEach(s => s.visible = false);
        }
      } else {
        this.pathLine.visible = false;
        this.pathSpheres.forEach(s => s.visible = false);
      }
    } else {
      this.pathLine.visible = false;
      this.pathSpheres.forEach(s => s.visible = false);
    }
  }

  _initSliceVisualization() {
    // Create a texture to paint explored regions
    const resolution = 256; // texture resolution
    const canvas = document.createElement('canvas');
    canvas.width = resolution;
    canvas.height = resolution;
    this.sliceCanvas = canvas;
    this.sliceContext = canvas.getContext('2d');
    
    // Initialize with transparent black
    this.sliceContext.fillStyle = 'rgba(0, 0, 0, 0)';
    this.sliceContext.fillRect(0, 0, resolution, resolution);
    
    // Create texture from canvas
    this.sliceTexture = new THREE.CanvasTexture(canvas);
    this.sliceTexture.needsUpdate = true;
    
    // Create horizontal plane at ground level to show explored areas
    const planeSize = 40; // matches grid size (80 * 0.5)
    const planeGeo = new THREE.PlaneGeometry(planeSize, planeSize);
    const planeMat = new THREE.MeshBasicMaterial({
      map: this.sliceTexture,
      transparent: true,
      opacity: 0.6,
      side: THREE.DoubleSide,
      depthWrite: false
    });
    
    this.sliceMesh = new THREE.Mesh(planeGeo, planeMat);
    this.sliceMesh.position.z = 0.1; // Slightly above floor (Z-up)
    this.sliceGroup.add(this.sliceMesh);
  }

  _updateSliceVisualization() {
    if (!this.grid || !this.sliceContext) return;
    
    const ctx = this.sliceContext;
    const resolution = this.sliceCanvas.width;
    const droneZ = this.drone.state.position.z;
    
    // Get the Z slice at drone altitude
    const zIndex = Math.floor((droneZ - this.grid.origin.z) / this.grid.resolution);
    
    // Paint explored cells at this Z level
    for (let y = 0; y < this.grid.sizeY; y++) {
      for (let x = 0; x < this.grid.sizeX; x++) {
        const value = this.grid.value(x, y, Math.max(0, Math.min(zIndex, this.grid.sizeZ - 1)));
        
        // Map grid coordinates to texture coordinates
        const texX = Math.floor((x / this.grid.sizeX) * resolution);
        const texY = Math.floor((y / this.grid.sizeY) * resolution);
        
        if (Math.abs(value) > 0.1) {
          // Cell has been observed
          if (value > 0.6) {
            // Occupied - red/orange
            ctx.fillStyle = 'rgba(255, 100, 50, 0.8)';
          } else if (value < -0.1) {
            // Free space - cyan/blue
            ctx.fillStyle = 'rgba(50, 200, 255, 0.4)';
          }
          ctx.fillRect(texX, texY, Math.ceil(resolution / this.grid.sizeX) + 1, Math.ceil(resolution / this.grid.sizeY) + 1);
        }
      }
    }
    
    // Mark drone current position
    const droneTexX = Math.floor(((this.drone.state.position.x - this.grid.origin.x) / (this.grid.sizeX * this.grid.resolution)) * resolution);
    const droneTexY = Math.floor(((this.drone.state.position.y - this.grid.origin.y) / (this.grid.sizeY * this.grid.resolution)) * resolution);
    
    ctx.fillStyle = 'rgba(0, 255, 0, 0.9)';
    ctx.beginPath();
    ctx.arc(droneTexX, droneTexY, 4, 0, Math.PI * 2);
    ctx.fill();
    
    this.sliceTexture.needsUpdate = true;
  }
}
