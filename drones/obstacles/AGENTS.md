# AI Agent Instructions — Obstacle Avoidance Demo

## Overview
Interactive obstacle avoidance demonstration featuring **real-time physics simulation** of falling obstacles and **reactive collision avoidance** for hovering drone. Emphasizes safe navigation in dynamic environments.

## Purpose
- **Dynamic Obstacles**: Physics-based falling objects (spheres, boxes)
- **Collision Avoidance**: Simple reactive controller (potential fields)
- **Interactive Demo**: Drag-and-drop obstacle placement
- **Educational**: Visualize safety margins, collision zones

## Architecture

### System Overview
```
┌───────────────┐
│ User Input    │ → Drag obstacles, drop from height
└───────┬───────┘   Mouse interaction (3D picking)
        │
        ▼
┌───────────────┐
│ Obstacle Mgr  │ → Track all obstacles
│               │   Physics simulation (gravity, bounce)
└───────┬───────┘   Collision detection (sphere-sphere)
        │
        ▼
┌───────────────┐
│ Avoidance     │ → Compute repulsive forces
│ Controller    │   Maintain safe distance
└───────┬───────┘   Blend with hover controller
        │
        ▼
┌───────────────┐
│ Hover Control │ → Return to home position
│               │   Altitude hold
└───────┬───────┘   Gentle convergence
        │
        ▼
┌───────────────┐
│ Motor Alloc   │ → SE(3) mixing (simple PID)
└───────────────┘   Crazyflie physics
```

## Files and Components

### Main Demo
**`drone_obstacle_demo.js`** (407 lines)
- **Lines 1-100**: Initialization, scene setup, obstacle manager
- **Lines 100-200**: Draggable controls (mouse interaction)
- **Lines 200-300**: Physics integration, collision avoidance
- **Lines 300-407**: Rendering (drone, obstacles, safety spheres)

**Key Features**:
- Overhead camera (top-down view)
- Drag obstacles to reposition
- Click to drop new obstacles
- Visualize safety radius (transparent sphere)
- Pause/resume physics

### Obstacle Management
**`obstacle_manager.js`**

**Obstacle Types**:
```javascript
SPHERE: {
  radius: 0.3,
  mass: 0.5,
  color: 0xff0000
}

BOX: {
  size: [0.4, 0.4, 0.4],
  mass: 0.8,
  color: 0x0000ff
}

CYLINDER: {
  radius: 0.2,
  height: 0.6,
  mass: 0.6,
  color: 0x00ff00
}
```

**Physics Integration**:
```javascript
// Simple Euler integration
obstacle.velocity.z += -gravity * dt;
obstacle.position.add(obstacle.velocity.clone().multiplyScalar(dt));

// Ground collision
if (obstacle.position.z < obstacle.radius) {
  obstacle.position.z = obstacle.radius;
  obstacle.velocity.z *= -0.5; // Bounce with damping
}
```

**Collision Detection**:
```javascript
// Sphere-sphere (obstacle vs drone)
const distance = obstacle.position.distanceTo(drone.position);
if (distance < obstacle.radius + drone.radius) {
  return COLLISION;
}
```

### Obstacle Simulation
**`obstacle_sim.js`**

**Preset Scenes**:
```javascript
'sparse':   3 obstacles, wide spacing
'dense':    10 obstacles, tight packing
'falling':  Obstacles drop from ceiling
'static':   Fixed positions (no physics)
```

**Spawning**:
```javascript
spawnObstacle(type, position) {
  const obstacle = createObstacle(type);
  obstacle.position.copy(position);
  obstacle.velocity.set(0, 0, 0); // Initially at rest
  obstacles.push(obstacle);
}
```

**Cleanup**:
```javascript
// Remove obstacles that fall out of bounds
obstacles = obstacles.filter(obs => obs.position.z > -10);
```

### Drag-and-Drop Interface
**`draggable_controls.js`**

**Mouse Picking**:
```javascript
// Raycast from camera through mouse position
const raycaster = new THREE.Raycaster();
raycaster.setFromCamera(mouse, camera);

const intersects = raycaster.intersectObjects(obstacles);
if (intersects.length > 0) {
  selectedObstacle = intersects[0].object;
}
```

**Dragging**:
```javascript
// Project mouse ray onto horizontal plane at obstacle height
const plane = new THREE.Plane(new Vector3(0, 0, 1), -obstacle.position.z);
const intersection = raycaster.ray.intersectPlane(plane);

if (intersection) {
  obstacle.position.x = intersection.x;
  obstacle.position.y = intersection.y;
}
```

**Dropping**:
```javascript
// On mouse release, give obstacle initial downward velocity
if (wasAboveGround) {
  obstacle.velocity.z = -1.0; // m/s
}
```

### Collision Avoidance Controller

**Algorithm**: Artificial Potential Field
```javascript
// Attractive force toward home position
F_attract = -K_attract * (drone_pos - home_pos)

// Repulsive force from each obstacle
for (obstacle in obstacles) {
  distance = ||drone_pos - obstacle_pos||
  
  if (distance < safeDistance) {
    direction = (drone_pos - obstacle_pos).normalize()
    magnitude = K_repel * (1/distance - 1/safeDistance)
    F_repel += direction * magnitude
  }
}

// Combine forces
F_total = F_attract + F_repel

// Convert to desired velocity
v_desired = F_total / damping
```

**Parameters**:
```javascript
K_attract = 2.0        // Pull toward home
K_repel = 5.0          // Push away from obstacles
safeDistance = 1.5     // meters (drone radius + margin)
damping = 0.5          // Velocity scaling
```

**Integration with Hover Controller**:
```javascript
// Blend avoidance with hover
const target_pos = home_pos + avoidance_offset;
const ctrl = hoverController.compute(state, target_pos, dt);
```

### Stationary Controller (Legacy)
**`stationary_controller.js`** (used for hover)

Simple PID controller (not ETH SE(3)):
```javascript
// Position error
e_p = target_pos - current_pos

// PID output
u = Kp * e_p + Kd * (target_vel - current_vel) + Ki * ∫e_p

// Convert to thrust
thrust = mass * (u.z + gravity)

// Convert to motor commands (simplified allocation)
motor_commands = allocate(thrust, u.x, u.y, yaw_error)
```

**Note**: This is **not** the ETH controller; it's a simpler PID for demonstration purposes.

## Interactive Features

### Mouse Controls
- **Left Click + Drag**: Move obstacle horizontally
- **Right Click**: Drop new obstacle at cursor
- **Mouse Wheel**: Zoom camera
- **Middle Drag**: Pan camera

### Keyboard Shortcuts
- **Space**: Pause/Resume physics
- **R**: Reset scene (clear obstacles)
- **1-3**: Spawn sphere/box/cylinder at center
- **C**: Toggle camera mode (overhead/angled)

### HUD Display
```
Obstacles: 5
Drone Status: AVOIDING
Distance to Nearest: 1.2m
Safe: YES ✓
```

## Performance Considerations

### Obstacle Limit
- **Recommended**: <20 obstacles for 60 FPS
- **Maximum**: 50 obstacles (starts to lag)
- **Bottleneck**: Collision detection (O(n) per frame)

### Optimization
```javascript
// Spatial hashing for collision detection
const grid = new SpatialHashGrid(cellSize = 2.0);
for (obstacle in obstacles) {
  grid.insert(obstacle);
}

// Only check nearby obstacles
const nearby = grid.query(drone.position, radius = safeDistance);
```

### Visual Quality
```javascript
// High quality
obstacles.castShadow = true;
obstacles.segments = 32;

// Performance mode
obstacles.castShadow = false;
obstacles.segments = 16;
```

## Common Tasks for AI Agents

### Easy
- Add new obstacle shapes (torus, pyramid)
- Change avoidance parameters (safe distance, gains)
- Modify obstacle colors/materials
- Add particle effects on collision

### Medium
- Implement trajectory prediction (avoid future collisions)
- Add moving obstacles (constant velocity)
- Create obstacle patterns (swarm, orbit)
- Implement 3D picking (drag vertically)

### Hard
- Multi-drone avoidance (inter-drone collision)
- Learning-based avoidance (neural network)
- Optimal path planning (A* with dynamic obstacles)
- Soft-body obstacles (deformable)

## Known Issues & Solutions

### Issue #1: Drone Gets Trapped Between Obstacles
**Symptom**: Oscillates without escaping  
**Cause**: Repulsive forces cancel out  
**Fix**: Add random perturbation:
```javascript
if (stuck_count > 100) {
  F_total += randomVector() * 0.5;
}
```

### Issue #2: Obstacles Penetrate Ground
**Symptom**: Objects clip through floor  
**Cause**: Large timestep misses collision  
**Fix**: Reduce physics timestep:
```javascript
const dt = 1 / 200; // Instead of 1/120
```

### Issue #3: Drag Feels Laggy
**Symptom**: Obstacle lags behind cursor  
**Cause**: Raycasting computed only once per render frame  
**Fix**: Smooth interpolation:
```javascript
obstacle.position.lerp(targetPosition, 0.3);
```

### Issue #4: Drone Vibrates Near Obstacles
**Symptom**: Jittery motion when close to safe distance  
**Cause**: Discontinuous repulsive force  
**Fix**: Use smooth potential (Gaussian instead of 1/r):
```javascript
magnitude = K_repel * exp(-distance² / (2*σ²));
```

### Issue #5: Obstacles Disappear After Drop
**Symptom**: Object vanishes when released  
**Cause**: Velocity becomes NaN (division by zero)  
**Fix**: Clamp velocity:
```javascript
obstacle.velocity.clampLength(0, 10.0); // Max 10 m/s
```

## Testing Checklist

- [ ] Drone hovers stably with no obstacles
- [ ] Drone avoids single obstacle (all sides)
- [ ] Drone navigates through obstacle field
- [ ] Drag-and-drop works smoothly
- [ ] Obstacles bounce realistically
- [ ] No penetration (obstacles or ground)
- [ ] HUD updates in real-time
- [ ] Reset button clears all obstacles

## Coordinate System

### Scene Layout
```
        +Y (forward)
         ↑
         │
    ─────┼────→ +X (right)
         │
         ●  (origin, home position)
        
+Z: up (out of screen in overhead view)
```

### Drone Home Position
```javascript
homePosition = new Vector3(0, 0, 1.5); // 1.5m altitude
```

### Obstacle Spawn Height
```javascript
spawnHeight = 3.0; // meters (above drone)
```

## Physics Parameters

### Gravity
```javascript
gravity = 9.81; // m/s² (standard Earth gravity)
```

### Obstacle Properties
```javascript
// Restitution (bounciness)
restitution = 0.5; // 50% energy retained on bounce

// Air drag (simplified)
dragCoeff = 0.1; // Linear velocity damping
```

### Collision Response
```javascript
// Elastic collision (conserve momentum)
v1_after = ((m1 - m2)*v1 + 2*m2*v2) / (m1 + m2);
v2_after = ((m2 - m1)*v2 + 2*m1*v1) / (m1 + m2);

// Apply restitution
v1_after *= restitution;
v2_after *= restitution;
```

## Debugging Tips

### Visualize Forces
```javascript
// Draw arrow for repulsive force
const arrow = new THREE.ArrowHelper(
  F_repel.normalize(),
  drone.position,
  F_repel.length() * 0.5,
  0xff0000
);
scene.add(arrow);
```

### Log Collision Events
```javascript
console.log(`[COLLISION] Obstacle ${id} at distance ${dist.toFixed(2)}m`);
```

### Show Safe Zone
```javascript
// Transparent sphere around drone
const safeZone = new THREE.Mesh(
  new THREE.SphereGeometry(safeDistance, 32, 32),
  new THREE.MeshBasicMaterial({
    color: 0x00ff00,
    transparent: true,
    opacity: 0.2
  })
);
drone.add(safeZone);
```

## Dependencies

- `../physics/drone_physics_engine.js` — Dynamics
- `../stationary/stationary_controller.js` — Simple PID hover
- `Three.js` — Rendering, raycasting
- No external physics engine (custom implementation)

## Future Enhancements

### Planned Features
- [ ] Multi-drone cooperation (formation flying)
- [ ] Obstacle prediction (Kalman filter)
- [ ] Path recording/playback
- [ ] VR mode (immersive avoidance)
- [ ] Realistic sensor simulation (lidar, camera)

### Research Directions
- Machine learning for obstacle avoidance
- Optimal reciprocal collision avoidance (ORCA)
- Swarm intelligence (flocking behavior)
- Non-cooperative agents (adversarial obstacles)

## References

- **Potential Fields**: Khatib, "Real-Time Obstacle Avoidance for Manipulators and Mobile Robots" (1986)
- **Reactive Navigation**: Borenstein & Koren, "The Vector Field Histogram" (1991)
- **Dynamic Obstacles**: Fiorini & Shiller, "Motion Planning in Dynamic Environments" (1998)
- **Collision Avoidance**: Berg et al., "Reciprocal n-Body Collision Avoidance" (2011)

---

**Last Updated**: Dec 2025  
**Maintainer**: Konstantin Krasovitskiy  
**Status**: Demo (educational purposes)
