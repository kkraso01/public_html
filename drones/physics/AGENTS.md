# AI Agent Instructions — Drone Physics Engine

## Overview
This folder contains the **shared physics engine** used by all drone simulations. It implements high-fidelity Crazyflie 2.x quadrotor dynamics with correct X-configuration motor allocation.

## Critical Physics Conventions (MUST FOLLOW)

### **Coordinate System: +Z UP (ETH Standard)**
- **World Frame**: Right-handed with +Z pointing UP
  - +X: arbitrary horizontal direction
  - +Y: perpendicular to X in horizontal plane
  - +Z: **UPWARD** (altitude increases with Z)
- **Body Frame**: Right-handed attached to quadrotor
  - +X: forward (nose direction)
  - +Y: left (port side)
  - +Z: up (thrust direction, perpendicular to rotor plane)
- **Gravity**: Acts in **-Z direction** (downward): `F_gravity = (0, 0, -m*g)`
- **Thrust**: Acts in **+Z direction** (upward in body frame)

### **Motor Configuration: Crazyflie 2.x X-Frame**
```
Motor layout (X-configuration):
     0 (CW)          1 (CCW)
       ╲            ╱
        ╲          ╱
         ╲  +X ↑ ╱
          ╲    ╱
     +Y ← ●  (center of mass)
          ╱    ╲
         ╱      ╲
        ╱        ╲
     3 (CCW)      2 (CW)

Motor 0: Front-Left  (CW)  at angle 135°
Motor 1: Front-Right (CCW) at angle 45°
Motor 2: Back-Right  (CW)  at angle -45° (315°)
Motor 3: Back-Left   (CCW) at angle -135° (225°)
```

### **Torque Equations (CRITICAL - DO NOT CHANGE)**
```javascript
// X-configuration with arm length L, motors at 45° angles
// Effective moment arm = L/√2 in both X and Y directions

// Roll torque (about +X axis, left vs right):
τ_x = (L/√2) * kF * ((ω₀² + ω₃²) - (ω₁² + ω₂²))

// Pitch torque (about +Y axis, front vs back):
τ_y = (L/√2) * kF * ((ω₀² + ω₁²) - (ω₂² + ω₃²))

// Yaw torque (about +Z axis, CW=-1, CCW=+1):
τ_z = kM * (-ω₀² + ω₁² - ω₂² + ω₃²)

// Total thrust (all motors):
T_total = kF * (ω₀² + ω₁² + ω₂² + ω₃²)
```

## Files in This Folder

### `drone_physics_engine.js` (Core Physics)
- **Purpose**: High-fidelity 6-DOF quadrotor dynamics
- **Key Features**:
  - First-order motor lag model
  - Correct X-configuration torque generation
  - Quaternion-based attitude representation
  - Aerodynamic drag (linear model)
  - Coriolis/gyroscopic effects
- **State Vector**:
  ```javascript
  state = {
    p_W: Vector3,        // position in world frame
    v_W: Vector3,        // velocity in world frame
    q_WB: Quaternion,    // orientation (body → world)
    omega_B: Vector3,    // angular velocity in body frame
    motorSpeeds: [ω₀, ω₁, ω₂, ω₃]  // rad/s
  }
  ```
- **Critical Methods**:
  - `step(dt)`: Integrates dynamics forward by dt seconds
  - `applyMotorCommands(u0, u1, u2, u3)`: Commands in [0,1] range
  - `getState()`: Returns cloned state for external use

### `motor_model.js` (Motor Dynamics)
- **Purpose**: First-order lag model for motor response
- **Time Constant**: τ = 0.015s (realistic for Crazyflie coreless motors)
- **Transfer Function**: `ω(s) / ω_cmd(s) = 1 / (τs + 1)`
- **Implementation**: `dω/dt = (ω_cmd - ω) / τ`

### `params_crazyflie.js` (Physical Constants)
- **Purpose**: Real Crazyflie 2.x hardware specifications
- **Key Parameters**:
  - `mass = 0.028 kg` (28 grams)
  - `armLength = 0.046 m` (92mm prop-to-prop diagonal / 2)
  - `thrustCoeff = 2.88e-8 N·s²` (kF)
  - `torqueCoeff = 7.24e-10 N·m·s²` (kM)
  - `omegaMax = 2500 rad/s` (~24k RPM)
  - `inertia: {Jxx: 1.4e-5, Jyy: 1.4e-5, Jzz: 2.17e-5} kg·m²`
- **DO NOT MODIFY** these values without physical justification

## Common Bugs and How to Avoid Them

### ❌ Bug #1: Wrong Gravity Sign
```javascript
// WRONG (makes drone fall up):
F_gravity = new THREE.Vector3(0, 0, +mass * gravity);

// CORRECT:
F_gravity = new THREE.Vector3(0, 0, -mass * gravity);
```

### ❌ Bug #2: Wrong Motor Spin Directions in Yaw Torque
```javascript
// WRONG (yaw control won't work):
τ_z = kM * (+ω₀² - ω₁² + ω₂² - ω₃²);  // flipped signs

// CORRECT (matches Crazyflie X-config):
τ_z = kM * (-ω₀² + ω₁² - ω₂² + ω₃²);
```

### ❌ Bug #3: Forgetting √2 in X-Configuration Torques
```javascript
// WRONG (torque will be 1.41x too large):
τ_x = L * kF * ((ω₀² + ω₃²) - (ω₁² + ω₂²));

// CORRECT (arms at 45° → effective moment arm = L/√2):
τ_x = (L / Math.sqrt(2)) * kF * ((ω₀² + ω₃²) - (ω₁² + ω₂²));
```

### ❌ Bug #4: Using NED Instead of ENU
This engine uses **ENU** (+Z up), NOT NED (+Z down). If integrating with NED code:
- Invert Z coordinates: `z_NED = -z_ENU`
- Invert Z velocities: `vz_NED = -vz_ENU`
- Rotate quaternion by 180° around X or Y axis

## Integration with Controllers

Controllers (ETH, MPC, etc.) must:
1. **Use +Z up convention** in all calculations
2. **Add gravity compensation** to Z-axis acceleration: `a_cmd.z += gravity`
3. **Match motor allocation** exactly (motor indices, spin directions, torque signs)
4. **Respect thrust limits**: `0 ≤ T_i ≤ kF * ω_max²` per motor
5. **Use correct quaternion convention**: `q_WB` rotates body → world

## Testing and Validation

### Hover Test (Most Basic)
```javascript
// At hover, motors should be equal:
const hoverThrust = mass * gravity / 4; // per motor
const hoverOmega = Math.sqrt(hoverThrust / kF);
// Should be ~1633 rad/s for Crazyflie (realistic)
```

### Torque Sign Test
```javascript
// Increase motor 0 only → should roll LEFT and yaw CW
applyMotorCommands(0.6, 0.5, 0.5, 0.5);
// Expected: τ_x > 0 (roll left), τ_z < 0 (yaw CW)

// Increase motors 0,1 (front) → should pitch UP
applyMotorCommands(0.6, 0.6, 0.5, 0.5);
// Expected: τ_y > 0 (nose up)
```

### Gravity Test
```javascript
// Zero thrust → should fall at g
applyMotorCommands(0, 0, 0, 0);
step(0.1);
// Expected: v_W.z ≈ -0.981 m/s (falling down)
```

## Performance Considerations

- **Integration Rate**: Use dt ≤ 5ms (200+ Hz) for stability
- **Motor Lag**: Already modeled, don't add artificial delays
- **Drag**: Current model is simple; for high-speed flight, consider quadratic drag
- **Quaternion Normalization**: Already handled in `step()`, no need to normalize externally

## Dependencies

- **Three.js**: Used for Vector3, Matrix3, Quaternion math
- **ES6 Modules**: All files use `import/export` syntax
- **No external physics libraries**: Self-contained implementation

## Coordinate Transform Cheat Sheet

```javascript
// Body to world (thrust vector):
F_world = F_body.clone().applyMatrix3(R_BW);
// where R_BW = rotationMatrix(q_WB)

// World to body (wind/disturbance):
F_body = F_world.clone().applyMatrix3(R_WB);
// where R_WB = R_BW.transpose()

// Extract Euler angles (for visualization only, NOT control):
const euler = new THREE.Euler().setFromQuaternion(q_WB, 'XYZ');
const roll = euler.x, pitch = euler.y, yaw = euler.z;
```

## Common Tasks for AI Agents

### Easy Tasks
- Adjust drag coefficients for different environments
- Add ground effect model near floor
- Implement wind disturbances
- Add telemetry/logging

### Medium Tasks
- Implement propwash/downwash effects
- Add rotor inflow dynamics
- Model battery voltage sag
- Implement motor saturation nonlinearities

### Hard Tasks
- Switch to full rotor blade element theory
- Add flexible body dynamics
- Implement ground contact/collision
- Multi-drone aerodynamic interference

## What NOT to Do

- ❌ **DO NOT** change motor spin directions without updating controllers
- ❌ **DO NOT** switch to NED without updating all dependent code
- ❌ **DO NOT** modify torque equations without physical justification
- ❌ **DO NOT** use Euler angles for integration (quaternion drift)
- ❌ **DO NOT** assume hover thrust = total max thrust / 4 (that's only at hover!)

## References

- Crazyflie 2.x Hardware: https://www.bitcraze.io/products/crazyflie-2-1/
- ETH Flying Machine Arena: https://flyingmachinearena.org/
- Mellinger, D. "Minimum Snap Trajectory Generation and Control for Quadrotors" (2011)
- Lee, T. "Geometric Tracking Control of a Quadrotor UAV on SE(3)" (2010)

---

**Last Updated**: Dec 2025  
**Maintainer**: Konstantin Krasovitskiy  
**Status**: Production (used by all drone demos)
