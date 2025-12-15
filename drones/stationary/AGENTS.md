# AI Agent Instructions — Stationary Hover Demo

## Overview
Interactive demo for testing quadrotor **hover stability** and **waypoint tracking** with multiple controller options. Features real-time gain tuning, trajectory modes, and detailed telemetry visualization.

## Purpose
- **Educational**: Visualize PID control loops, attitude stabilization, motor allocation
- **Research**: Test ETH geometric controller, iLQR-MPC, and classical cascaded controllers
- **Debugging**: Real-time plots, 3D motor visualization, state telemetry

## Architecture

### Main Components
```
stationary_test.html         → HTML page with controls
stationary_test.js           → Demo orchestration, UI, rendering
eth_controller.js            → ETH geometric SE(3) controller (PRIMARY)
mpc_controller.js            → Model Predictive Control (EXPERIMENTAL)
stationary_controller.js     → Legacy PID controller (DEPRECATED)
```

### Coordinate System (CRITICAL)
- **World Frame**: +Z UP (ETH convention)
- **Body Frame**: +X forward, +Y left, +Z up
- **Motor Layout**: Crazyflie X-configuration (see physics/AGENTS.md)
- **Gravity Compensation**: `a_cmd.z += gravity` in outer loop

## Controller Hierarchy

### 🏆 ETH Controller (eth_controller.js) — PRIMARY
**Status**: Production-ready, most stable  
**Type**: Cascaded geometric SE(3) controller  
**Reference**: Lee 2010, Mellinger 2011, ETH Flying Machine Arena

#### Control Architecture
```
Outer Loop (Position/Velocity):
  input:  p_ref, v_ref, a_ref, yaw_ref
  output: desired acceleration a_des
  
  a_des = Kp*(p_ref - p) + Kd*(v_ref - v) + Ki*∫error + a_ff + [0,0,g]
  
Inner Loop (Attitude):
  input:  a_des, yaw_des
  output: motor commands [u₀, u₁, u₂, u₃]
  
  1. Compute desired thrust: T_des = m * ||a_des||
  2. Compute desired attitude: R_des from a_des and yaw
  3. Attitude error: e_R = vee(R_des^T R - R^T R_des) / 2
  4. Torque command: τ = -KR*e_R - Kω*e_ω + ω × Jω
  5. Motor allocation: solve [T, τ] → [u₀,u₁,u₂,u₃]
```

#### Default Gains (Proven Stable)
```javascript
// Position loop
Kp = [4.0, 4.0, 8.0]   // Higher on Z for altitude priority
Kd = [3.0, 3.0, 4.0]   // Critical damping
Ki = [0.05, 0.05, 0.20] // Gentle integral, Z higher

// Attitude loop
KR = [4.5, 4.5, 1.0]      // Roll/pitch aggressive, yaw smooth
Kω = [0.10, 0.10, 0.05]   // Angular rate damping
```

#### Motor Allocation (X-Configuration)
```javascript
// Inverse allocation solving:
//   [1   1   1   1 ] [f₀]   [S ]   where S  = T/kF
//   [1  -1  -1   1 ] [f₁] = [Tx]         Tx = τ_x√2/(L·kF)
//   [1   1  -1  -1 ] [f₂]   [Ty]         Ty = τ_y√2/(L·kF)
//   [-1  1  -1   1 ] [f₃]   [Tz]         Tz = τ_z/kM

// Solution:
f₀ = ¼(S + Tx + Ty - Tz)  // Front-Left  (CW)
f₁ = ¼(S - Tx + Ty + Tz)  // Front-Right (CCW)
f₂ = ¼(S - Tx - Ty - Tz)  // Back-Right  (CW)
f₃ = ¼(S + Tx - Ty + Tz)  // Back-Left   (CCW)
```

#### Saturation Handling
```javascript
// Priority: attitude stability > yaw control
if (anyMotorOverMax || anyMotorNegative) {
  // Try dropping yaw first
  [T₀,T₁,T₂,T₃] = solveThrusts(τ_z = 0);
  
  if (stillSaturated) {
    // Scale all motors proportionally
    scale = T_max / max(T₀,T₁,T₂,T₃);
    [T₀,T₁,T₂,T₃] *= scale;
  }
}
```

#### Recent Bug Fixes (Dec 2025)
**❌ FIXED: Artificial Thrust Limit Bug**
```javascript
// OLD (WRONG - caused permanent saturation):
const hoverThrustPerMotor = (mass * gravity) / 4;
this.maxThrustPerMotor = hoverThrustPerMotor * 2.0;

// NEW (CORRECT - uses actual physics limits):
this.maxThrustPerMotor = params.maxThrustPerMotor ??
                          (this.kF * this.omegaMax * this.omegaMax);
```
**Problem**: Old code limited thrust to 2x hover (~0.137N), but physics allows ~0.18N  
**Result**: Motors saturated, drone couldn't descend (always climbing)  
**Solution**: Use actual physics envelope from motor model

**❌ FIXED: Premature Thrust Clamp**
```javascript
// OLD (WRONG - destroys altitude control):
thrust_des = clamp(thrust_des, 0, T_total_max);

// NEW (CORRECT - no early clamping):
// Let thrust_des have full range; saturation handled per-motor later
```
**Problem**: Clamping total thrust early prevented proper descent  
**Result**: When target below current position, thrust stayed maxed  
**Solution**: Remove early clamp, handle saturation in allocation

### MPC Controller (mpc_controller.js) — EXPERIMENTAL
**Status**: Work-in-progress, not used in main demo  
**Type**: Model Predictive Control with trajectory prediction  
**Horizon**: 10-20 timesteps  
**Update Rate**: Lower than ETH (more computational)

### Legacy PID Controller (stationary_controller.js) — DEPRECATED
**Status**: Kept for reference, not recommended  
**Issues**: Gimbal lock, poor transient response, no feedforward

## Demo Features

### Trajectory Modes
1. **Hover**: Fixed position hold
2. **Circle**: Horizontal circular trajectory
3. **Figure-8**: Lissajous pattern (tests rapid direction changes)
4. **Waypoint**: User-defined 3D path following
5. **Manual**: Direct position control via sliders

### Camera Modes
- **Chase**: Follows drone from behind (default)
- **Orbit**: Circles around drone
- **Free**: Manual camera control (mouse drag)
- **Top-Down**: Overhead view

### HUD Elements
- Real-time state telemetry (position, velocity, attitude)
- Motor thrust visualization (4 colored bars)
- Controller gains (live-adjustable sliders)
- Performance metrics (tracking error, control effort)
- 3D motor force vectors on drone model

### Interactive Controls
- **Gain Sliders**: Real-time PID/SE(3) gain tuning
- **Trajectory Selector**: Switch modes on-the-fly
- **Reset Button**: Restart simulation
- **Pause/Resume**: Freeze physics
- **Camera Controls**: Mouse/keyboard navigation

## File Structure

### `stationary_test.js` (Main Demo)
- **Lines 1-100**: Initialization, scene setup
- **Lines 100-300**: Physics integration loop
- **Lines 300-500**: Rendering (drone, motors, trails, HUD)
- **Lines 500-700**: UI controls (sliders, buttons)
- **Lines 700-900**: Trajectory generators
- **Lines 900-1062**: Utilities, cleanup

### `eth_controller.js` (Production Controller)
- **Lines 1-50**: Constructor, parameter initialization
- **Lines 50-100**: Gain update methods
- **Lines 100-220**: Outer loop (position → acceleration)
- **Lines 220-400**: Inner loop (acceleration → motor commands)
- **Lines 400-630**: Nose-first mode, utilities

## Common Tasks for AI Agents

### Easy
- Add new trajectory pattern (modify trajectory generators)
- Adjust default gains for different flight styles
- Add new HUD telemetry fields
- Change drone visual appearance

### Medium
- Implement obstacle avoidance
- Add wind disturbance visualization
- Create trajectory recording/playback
- Implement formation flight (multi-drone)

### Hard
- Add real-time trajectory optimization
- Implement adaptive gain scheduling
- Create machine learning gain tuner
- Add full 6-DOF collision detection

## Known Issues & Workarounds

### Issue #1: Oscillations at Waypoint
**Symptom**: Drone oscillates when reaching waypoint  
**Cause**: Constant gains too aggressive near target  
**Fix**: Gain scaling already implemented (lines 118-120 in eth_controller.js)
```javascript
if (posErrorMag < 0.15) {
  gainScale = Math.max(0.1, posErrorMag / 0.15);
}
```

### Issue #2: Slow Yaw Response
**Symptom**: Drone takes time to rotate to desired heading  
**Cause**: Conservative yaw gains for stability  
**Fix**: Increase `KR.z` and `Kω.z` (use sliders), but watch for oscillations

### Issue #3: Altitude Drift in Circle Mode
**Symptom**: Z slowly increases/decreases during horizontal flight  
**Cause**: Insufficient integral gain on Z  
**Fix**: Increase `Ki.z` from 0.20 to 0.30 (use slider)

## Testing Checklist

Before committing changes to controller:

- [ ] Hover at (0,0,1) for 10 seconds without drift
- [ ] Circle trajectory completes without oscillations
- [ ] Figure-8 maintains altitude throughout
- [ ] Waypoint approach is smooth (no overshoot)
- [ ] Manual position changes respond in <0.5s
- [ ] All motors stay in [0,1] range (no saturation spam)
- [ ] Reset button returns to stable hover
- [ ] Gain sliders update in real-time without crashes

## Physics Integration Notes

### Timestep
- **Physics Rate**: 240 Hz (dt = 1/240 ≈ 4.17ms)
- **Render Rate**: 60 Hz (16.67ms)
- **Controller Rate**: Same as physics (240 Hz)

### Integration Method
Fixed timestep with accumulator:
```javascript
const dt = 1.0 / this.physicsRate;
this.accumulator += frameTime;
while (this.accumulator >= dt) {
  // 1. Update controller
  // 2. Step physics
  // 3. Check collisions
  this.accumulator -= dt;
}
```

### Ground Collision
```javascript
const minSafeAltitude = floorHeight + 0.6; // 0.65m default
if (state.position.z < minSafeAltitude) {
  state.velocity.z = Math.max(0, state.velocity.z); // Kill downward velocity
  state.position.z = minSafeAltitude; // Hard clamp
}
```

## Performance Optimization

### For Low-End Devices
```javascript
// Reduce visual complexity
options.shadows = false;
options.highQuality = false;

// Reduce physics rate
this.physicsRate = 120; // Half rate

// Simplify drone model
useLowPolyDrone = true;
```

### For High-End Devices
```javascript
// Enable all features
options.shadows = true;
options.highQuality = true;
this.physicsRate = 300; // Even higher fidelity
options.motionBlur = true;
```

## Dependencies

- `../physics/drone_physics_engine.js` — Core dynamics
- `../physics/params_crazyflie.js` — Hardware constants
- `Three.js` (global) — 3D rendering
- No external control libraries — all control code is self-contained

## Debugging Tips

### Enable Debug Logging
Uncomment debug logs in `eth_controller.js`:
```javascript
// Line 142: Acceleration command
// Line 182: Desired b1 vector
// Line 202: Yaw tracking
// Line 237: Thrust calculation
// Line 474: Nose-first control
```

### Monitor Motor Saturation
Watch for console spam:
```
[ETH_ALLOC] Motor saturation detected
```
If frequent, either:
- Reduce trajectory aggressiveness
- Increase `maxAcc` limit
- Check for thrust limit bugs (see "Recent Bug Fixes")

### Visualize Control Signals
Use HUD motor bars — should be:
- **Green**: Normal operation (0.3-0.7)
- **Yellow**: High thrust (0.7-0.9)
- **Red**: Saturated (>0.9 or <0.1)

## References

- **Lee 2010**: "Geometric Tracking Control of a Quadrotor UAV on SE(3)"
- **Mellinger 2011**: "Minimum Snap Trajectory Generation and Control for Quadrotors"
- **Faessler 2017**: "Differential Flatness of Quadrotor Dynamics Subject to Rotor Drag"
- **ETH Zurich ASL**: https://asl.ethz.ch/
- **Crazyflie Docs**: https://www.bitcraze.io/documentation/

---

**Last Updated**: Dec 2025  
**Maintainer**: Konstantin Krasovitskiy  
**Status**: Production (actively maintained)
