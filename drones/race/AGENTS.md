# AI Agent Instructions — Drone Racing Demo

## Overview
High-speed autonomous racing demo featuring **aggressive trajectory optimization**, **model predictive control (MPC)**, and **time-optimal gate navigation**. The drone races through a custom track at maximum speed while maintaining stability.

## Purpose
- **Racing Performance**: Minimum lap times through gate sequence
- **Trajectory Optimization**: Minimum-snap splines with time optimization
- **Advanced Control**: MPC with explicit thrust/tilt constraints
- **Real-Time Telemetry**: Live HUD with lap times, velocities, control effort

## Architecture

### Control Pipeline
```
┌──────────────┐
│ Track Setup  │ → Gate positions (3D torus models)
└──────┬───────┘   Reference trajectory (minimum-snap)
       │
       ▼
┌──────────────┐
│  Optimizer   │ → Time-optimal trajectory (minimum time)
│              │   Waypoint timing (aggressive but feasible)
└──────┬───────┘
       │
       ▼
┌──────────────┐
│  MPC / Race  │ → Tracking controller (feedforward + feedback)
│  Controller  │   Thrust vectoring (explicit tilt constraints)
└──────┬───────┘   Yaw control (nose-first racing)
       │
       ▼
┌──────────────┐
│   Physics    │ → Crazyflie dynamics (X-configuration)
│              │   Motor allocation (SE(3) mixer)
└──────────────┘   Collision detection (gate pass-through)
```

## Files and Components

### Main Demo
**`drone_race_demo.js`** (637 lines)
- **Lines 1-100**: Initialization, scene setup, track generation
- **Lines 100-200**: State machine (COUNTDOWN → RACING → FINISHED)
- **Lines 200-350**: Physics integration, gate detection
- **Lines 350-500**: Trajectory following, MPC updates
- **Lines 500-637**: Rendering (gates, trail, HUD, camera)

**State Machine**:
```javascript
COUNTDOWN:  3-2-1 timer, drone at start
RACING:     Active flight, lap timing
FINISHED:   Crossed finish, show results
CRASHED:    Collision detected (future)
```

### Track Generation
**`track.js`**
- **Gate Layouts**: Predefined sequences (loop, slalom, helix)
- **Gate Models**: 3D torus with customizable radius
- **Reference Trajectory**: Minimum-snap spline through gate centers
- **Velocity Profile**: Constant-speed or time-optimal

**Track Presets**:
```javascript
'simple':   Straight line (3 gates, testing)
'loop':     Vertical loop (5 gates, beginner)
'slalom':   S-curve (7 gates, intermediate)
'helix':    Spiral climb (9 gates, advanced)
'figure8':  Horizontal figure-8 (12 gates, expert)
```

**Example**:
```javascript
const track = buildRaceTrack('helix', {
  gateRadius: 0.8,  // meters
  numGates: 9,
  spacing: 3.0,     // meters between gates
  rotationRate: Math.PI / 4 // rad per gate
});
```

### Trajectory Optimization
**`trajectory_optimizer.js`**

#### Minimum-Snap Spline
**Objective**: Minimize ∫(d⁴r/dt⁴)² dt (smoothness)  
**Constraints**:
- Pass through waypoints
- Continuous up to 3rd derivative (jerk)
- Initial/final velocity = 0

**Implementation**:
```javascript
// Solve for polynomial coefficients:
// r(t) = a₀ + a₁t + a₂t² + a₃t³ + a₄t⁴ + a₅t⁵
// Minimize snap = |d⁴r/dt⁴|²
```

**Output**: Polynomial spline segments (5th order)

#### Time Optimization
**Objective**: Minimize total time T  
**Constraints**:
- Max velocity: 3.0 m/s (Crazyflie limit)
- Max acceleration: 15 m/s² (realistic for racing)
- Max jerk: 50 m/s³ (motor response limit)

**Algorithm**: Binary search on segment times
```python
while T_max - T_min > ε:
    T_mid = (T_max + T_min) / 2
    if trajectory_feasible(T_mid):
        T_max = T_mid  # Try faster
    else:
        T_min = T_mid  # Too aggressive
```

### Controllers

#### `race_controller.js` — Feedforward + SE(3)
**Type**: Geometric tracking controller with aggressive gains  
**Features**:
- Feedforward acceleration from trajectory
- High-bandwidth attitude loop (KR = [8, 8, 2])
- Nose-first yaw tracking
- Velocity prediction (lookahead 50ms)

**Gains** (tuned for racing):
```javascript
Kp = [6.0, 6.0, 10.0]  // Higher than hover
Kd = [4.0, 4.0, 5.0]   // Aggressive damping
Ki = [0.0, 0.0, 0.0]   // No integral (causes lag)
```

#### `mpc_controller.js` — Model Predictive Control
**Type**: Quadratic program with receding horizon  
**Horizon**: N = 15 steps (150ms @ 100Hz)  
**Update Rate**: 100 Hz (10ms intervals)

**Cost Function**:
```
J = Σ[||p - p_ref||²_Q + ||v - v_ref||²_R + ||u - u_hover||²_S]
```
Where:
- Q = position weight (high)
- R = velocity weight (medium)
- S = control effort weight (small)

**Constraints**:
```javascript
// Thrust limits
T_min ≤ T ≤ T_max  // [0, 4×maxThrustPerMotor]

// Tilt angle
||thrust_horizontal|| / thrust_vertical ≤ tan(θ_max)  // θ_max = 40°

// Input rates
|dT/dt| ≤ dT_max
```

**Solver**: Gradient descent with Armijo line search

### Gate Detection
**`gate_model.js`**

**Gate Representation**:
```javascript
gate = {
  position: Vector3,      // Center
  normal: Vector3,        // Facing direction
  radius: 0.8,            // Torus major radius
  tubeRadius: 0.1,        // Torus minor radius
  mesh: THREE.Mesh        // Visual model
}
```

**Pass Detection**:
```javascript
// Check if drone trajectory crossed gate plane
const prevSide = (prev_pos - gate_pos) · gate_normal;
const currSide = (curr_pos - gate_pos) · gate_normal;

if (prevSide * currSide < 0) { // Crossed plane
  const intersection = lineIntersection(prev_pos, curr_pos, gate);
  if (distance(intersection, gate_pos) < gate_radius) {
    return PASSED;
  }
}
```

**Collision** (not implemented yet):
Check if drone hit torus surface

### HUD and Telemetry
**`unified_hud.js`**

**Elements**:
- Lap timer (current / best lap)
- Speed gauge (m/s, visual bar)
- Gate counter (current / total)
- Altitude indicator
- Control effort (thrust %)
- State telemetry (position, velocity, attitude)

**Canvas Overlay**:
- 2D HUD drawn on transparent canvas
- Updates at 60 Hz (decoupled from physics)
- Color-coded (green = good, yellow = warning, red = danger)

**Example**:
```javascript
Speed: 2.4 m/s  [████████░░] 80%
Gate:  5/12     Next: 15.2m
Alt:   2.1m     Target: 2.5m
Thrust: 65%     [██████░░░░]
```

### Camera Control
**`race_ui_controller.js`**

**Modes**:
1. **Chase**: Follows drone from behind (default)
   - Offset: -3m back, +1m up
   - Smooth interpolation (lerp factor = 0.1)
2. **Orbit**: Circles track center
   - Radius: 10m
   - Height: 5m
   - Rotation: 0.01 rad/frame
3. **Onboard**: First-person view
   - Mounted on drone nose
   - Minimal lag (direct copy)
4. **Free**: Manual control
   - Mouse drag to rotate
   - Scroll to zoom

**Smooth Following**:
```javascript
// Chase camera update
const targetPos = drone.position.clone()
  .add(drone.forward.clone().multiplyScalar(-3))
  .add(new Vector3(0, 0, 1));

camera.position.lerp(targetPos, 0.1);
camera.lookAt(drone.position);
```

## Performance Optimization

### Rendering
```javascript
// High quality (default)
shadows: true
antialiasing: true
gateDetail: 64 segments
trailLength: 500 points

// Performance mode
shadows: false
antialiasing: false
gateDetail: 32 segments
trailLength: 100 points
```

### Physics
```javascript
// High fidelity
physicsRate: 240 Hz
controllerRate: 240 Hz

// Performance
physicsRate: 120 Hz
controllerRate: 120 Hz
```

### Known Bottlenecks
1. **MPC Solver**: QP iteration (~2ms per solve)
   - **Fix**: Use race_controller instead (faster)
2. **Trail Rendering**: 500 points × 60 FPS
   - **Fix**: Reduce to 100 points
3. **Shadow Maps**: 2048×2048 for all gates
   - **Fix**: Disable shadows or reduce resolution

## Common Tasks for AI Agents

### Easy
- Add new track preset
- Adjust camera parameters
- Change gate visual style
- Modify HUD layout

### Medium
- Implement collision detection (gate impact)
- Add wind disturbances
- Create AI opponent (shadow drone)
- Record/replay lap ghosts

### Hard
- Multi-drone racing (avoid collisions)
- Learning-based trajectory optimization
- Real-time track generation (procedural)
- VR camera mode (stereoscopic)

## Known Issues & Solutions

### Issue #1: Drone Overshoots Gates at High Speed
**Symptom**: Passes gate but far from center  
**Cause**: Feedforward not strong enough  
**Fix**: Increase lookahead distance:
```javascript
const lookahead = 0.1; // 100ms prediction
const v_pred = v + a * lookahead;
```

### Issue #2: Oscillations on Tight Turns
**Symptom**: Wobbles when changing direction rapidly  
**Cause**: Derivative kick from waypoint switching  
**Fix**: Pre-filter trajectory with moving average:
```javascript
trajectory_smooth[i] = 0.25*traj[i-1] + 0.5*traj[i] + 0.25*traj[i+1];
```

### Issue #3: Yaw Lags Behind Trajectory
**Symptom**: Drone flies sideways through gates  
**Cause**: Yaw gains too conservative  
**Fix**: Enable aggressive yaw mode:
```javascript
options.aggressiveYawTracking = true;
options.maxYawRate = 50.0; // rad/s (very fast)
```

### Issue #4: MPC Violates Thrust Limits
**Symptom**: Motors saturate, control degrades  
**Cause**: QP solver doesn't respect inequality constraints  
**Fix**: Add barrier function to cost:
```javascript
if (T > 0.95 * T_max) {
  cost += 1000 * (T - 0.95*T_max)²;
}
```

### Issue #5: Gate Detection Misses Pass at High Speed
**Symptom**: Flies through gate but doesn't register  
**Cause**: Physics timestep too coarse (missed crossing)  
**Fix**: Increase physics rate to 300 Hz or use swept collision

## Testing Checklist

- [ ] Completes track without missing gates
- [ ] Lap time under 15 seconds (for 'loop' track)
- [ ] No motor saturation (thrust <95% of max)
- [ ] Smooth yaw transitions (no oscillations)
- [ ] Camera follows drone without jitter
- [ ] HUD updates in real-time
- [ ] Restart button resets to countdown
- [ ] All gates render correctly (no z-fighting)

## Coordinate System Notes

### Track Coordinates
- **Start Gate**: (0, 0, 1.5) facing +X
- **Subsequent Gates**: Positioned by preset
- **Finish Line**: Same as start (lap detection)

### Gate Normal Convention
```javascript
// Normal points in "forward" direction (drone should approach from behind)
gate.normal = new Vector3(1, 0, 0); // For start gate
```

### Trajectory Parameterization
```javascript
// Spline parameter: s ∈ [0, 1] maps to physical time
t_physical = s * T_total
position = spline.evaluate(s)
velocity = spline.derivative(s) / T_total
```

## Physics Integration

### Fixed Timestep Loop
```javascript
const dt = 1 / this.physicsRate; // 240 Hz → 4.17ms
while (accumulator >= dt) {
  // 1. Get reference from trajectory
  const ref = trajectory.evaluate(t);
  
  // 2. Compute control
  const ctrl = controller.compute(state, ref, dt);
  
  // 3. Step physics
  physics.applyMotorCommands(...ctrl.motorCommands);
  physics.step(dt);
  
  // 4. Check gate crossing
  checkGatePass(state, prevState);
  
  accumulator -= dt;
  t += dt;
}
```

### Collision Handling
Currently only floor collision:
```javascript
if (state.position.z < floorHeight + 0.3) {
  state.velocity.z = 0;
  state.position.z = floorHeight + 0.3;
}
```

Gate collision **not implemented** (drone is ghost).

## Debugging Tips

### Visualize Trajectory
```javascript
// Enable trajectory path rendering
demo.showReferencePath = true;

// Color-code by velocity
const speedColorMap = (v) => {
  if (v < 1.0) return 0x00ff00; // Green (slow)
  if (v < 2.0) return 0xffff00; // Yellow (medium)
  return 0xff0000;              // Red (fast)
};
```

### Monitor Control Effort
```javascript
// In browser console
setInterval(() => {
  const thrust = demo.lastControl.thrust;
  const thrustPct = (thrust / demo.physics.maxTotalThrust) * 100;
  console.log(`Thrust: ${thrustPct.toFixed(1)}%`);
}, 100);
```

### Log Gate Times
```javascript
// Add to gate pass detection
console.log(`Gate ${gateIdx} passed at t=${elapsedTime.toFixed(2)}s`);
```

## Dependencies

- `../physics/` — Dynamics, motor model
- `../stationary/eth_controller.js` — Base SE(3) controller
- `Three.js` — Rendering, 3D math
- No external optimization libraries (custom QP solver)

## References

- **Minimum Snap**: Mellinger & Kumar, "Minimum Snap Trajectory Generation and Control for Quadrotors" (ICRA 2011)
- **MPC for Quadrotors**: Kamel et al., "Model Predictive Control for Trajectory Tracking of Unmanned Aerial Vehicles" (ICRA 2017)
- **Time Optimization**: Richter et al., "Polynomial Trajectory Planning for Aggressive Quadrotor Flight" (ISRR 2013)
- **Drone Racing**: Kaufmann et al., "Deep Drone Acrobatics" (RSS 2020)

---

**Last Updated**: Dec 2025  
**Maintainer**: Konstantin Krasovitskiy  
**Status**: Production (racing competitions)
