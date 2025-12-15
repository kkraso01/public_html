# Stationary Demo Analysis

## 1. Overview of the Stationary Demo
The stationary hover demo instantiates a Crazyflie-parameterized quadrotor physics engine, drives it with an ETH-style cascaded SE(3) controller, and visualizes the hover in a Three.js scene. The demo initializes the physics engine, controller, and rendering assets, then advances simulation at a fixed 240 Hz step while rendering frames; when paused, single-step integration is available for inspection.【F:drones/stationary/stationary_test.js†L5-L92】【F:drones/stationary/stationary_test.js†L707-L791】 The control loop builds a target (position, velocity, yaw), asks the controller for motor commands and desired attitude, applies commands through the physics integrator (which models motor lag, thrust/torque, and rigid-body dynamics), then updates on-screen diagnostics (HUD, thrust arrows) before the next frame.【F:drones/stationary/stationary_test.js†L724-L858】【F:drones/physics/drone_physics_engine.js†L86-L190】 Default targets hover near z=0.6 m but user interaction can retarget in XY without interrupting the loop.【F:drones/stationary/stationary_test.js†L52-L77】【F:drones/stationary/stationary_test.js†L221-L243】

## 2. File-by-File Deep Analysis
### 2.1 physics/drone_physics_engine.js
Purpose: Simulates quadrotor dynamics using Crazyflie parameters, handling motor lag, thrust/torque computation, rigid-body integration, and state export.【F:drones/physics/drone_physics_engine.js†L1-L190】 Key elements:
- `clamp01` bounds normalized motor inputs.【F:drones/physics/drone_physics_engine.js†L4-L6】
- `quatDerivative(q, omega)` computes quaternion derivative for body-to-world orientation (q_WB) using right multiplication by body-frame angular velocity; scales by 0.5.【F:drones/physics/drone_physics_engine.js†L8-L21】
- Constructor sets parameters, diagonal inertia matrices, motor model, and state containers.【F:drones/physics/drone_physics_engine.js†L25-L63】
- `reset` assigns initial state vectors/quaternion, resets motors, and exposes aliases for compatibility.【F:drones/physics/drone_physics_engine.js†L65-L80】
- `applyMotorCommands` maps normalized [0,1] inputs to commanded rotor speeds using motor limits.【F:drones/physics/drone_physics_engine.js†L86-L90】
- `step(dt)` updates motor dynamics, computes thrust/torque via Crazyflie X layout, applies drag, integrates translational and rotational dynamics, and integrates quaternion then normalizes.【F:drones/physics/drone_physics_engine.js†L92-L176】 Torque mapping uses square-root-of-two arm projection, and yaw torque reflects CW/CCW signs.【F:drones/physics/drone_physics_engine.js†L117-L132】 Rotational dynamics include Coriolis cross term; translational uses body-to-world rotation without transpose.【F:drones/physics/drone_physics_engine.js†L141-L168】
- `getState` returns copies of position, velocity, orientation quaternion, angular velocity, and motor speeds.【F:drones/physics/drone_physics_engine.js†L179-L189】
Influence: Defines how motor commands translate to forces/torques and subsequent state evolution, so controller effectiveness depends on these mappings and integration choices.

### 2.2 physics/motor_model.js
Purpose: First-order actuator model converting commanded rotor speeds to actual speeds with time constant and clamping to min/max.【F:drones/physics/motor_model.js†L1-L35】 Key functions:
- Constructor sets limits and initializes arrays.【F:drones/physics/motor_model.js†L3-L10】
- `reset` zeroes speeds/commands.【F:drones/physics/motor_model.js†L12-L15】
- `step(dt, omegaCmd)` applies exponential approach to commanded speed per rotor and clamps.【F:drones/physics/motor_model.js†L17-L25】
- `getSpeeds` returns a copy; `_clampOmega` guards non-finite inputs.【F:drones/physics/motor_model.js†L27-L35】
Influence: Introduces motor lag and saturation, affecting responsiveness and stability of inner loop.

### 2.3 physics/params_crazyflie.js
Purpose: Provides physical parameters for Crazyflie 2.x including mass, inertia, thrust/torque coefficients, drag, motor limits, and gravity; precomputes max thrust per motor.【F:drones/physics/params_crazyflie.js†L1-L28】 Influence: Shared constants across controller and physics for consistent modeling; mismatches here would mis-scale control.

### 2.4 stationary/eth_controller.js
Purpose: Implements cascaded ETH/Lee SE(3) controller (outer PID in world frame, inner geometric attitude control, motor allocation) tuned for Crazyflie-scale vehicle.【F:drones/stationary/eth_controller.js†L1-L324】 Key structures:
- Constructor stores vehicle/motor constants, initializes gains, integral limits, acceleration/tilt caps, and inertia vector.【F:drones/stationary/eth_controller.js†L3-L36】
- `reset` clears position integral.【F:drones/stationary/eth_controller.js†L39-L41】
- Gain update helpers accept scalars, vectors, or objects for position and attitude loops.【F:drones/stationary/eth_controller.js†L43-L81】
- `computeControl(state, target, dt)` pipeline:
  - Extracts position/velocity and target references; computes position and velocity errors, updates integral with clamping.【F:drones/stationary/eth_controller.js†L83-L100】
  - Builds acceleration command including gravity feedforward (+Z up) and optional feedforward acceleration; clamps magnitude and horizontal component for tilt limits.【F:drones/stationary/eth_controller.js†L102-L132】
  - Converts acceleration to desired body frame via differential flatness: compute thrust direction b3 (normalize and ensure upward), heading vector b1_ref, orthogonal b2 = b3 x b1_ref, and b1 = b2 x b3, then construct quaternion q_des from basis (Three.js columns).【F:drones/stationary/eth_controller.js†L134-L178】
  - Collective thrust thrust_des = mass * (a_cmd dot b3) clamped to 0–4*T_max.【F:drones/stationary/eth_controller.js†L185-L189】
  - Attitude error: rotation matrices from current and desired quaternions, skew = R_des^T R - R^T R_des; vee maps skew to 3-vector; angular velocity error subtracts desired (zero).【F:drones/stationary/eth_controller.js†L191-L218】
  - Control law tau = -K_R * e_R - K_omega * e_omega + omega x (J omega); torques then clamped per axis.【F:drones/stationary/eth_controller.js†L227-L242】
  - Mixer: Inverse allocation for Crazyflie X using square-root-of-two geometry; solves thrusts per motor, handles saturation prioritizing attitude over yaw, clamps to motor thrust limits, converts thrust to normalized motor commands based on omega min/max.【F:drones/stationary/eth_controller.js†L244-L320】
Influence: Defines control actions and mapping to motors; any sign or frame mismatch here affects stability.

### 2.5 stationary/stationary_controller.js
Purpose: Thin wrapper around EthController to provide default hover target and additional diagnostics for visualization.【F:drones/stationary/stationary_controller.js†L1-L81】 Key behaviors:
- Holds EthController instance and forwards gain updates; `toggleFOPID` is inert for compatibility.【F:drones/stationary/stationary_controller.js†L4-L26】
- `compute` constructs default hover target if none, calls EthController, then extracts actual and desired thrust directions from quaternions (using third column of rotation matrices) to compute thrust misalignment angle and attach diagnostics to result.【F:drones/stationary/stationary_controller.js†L28-L79】 Influence: Diagnostics support HUD and logging; default target influences hover altitude.

### 2.6 stationary/stationary_test.js
Purpose: Demo harness setting up Three.js scene, physics engine, controller, UI, event handling, simulation loop, and rendering for stationary hover test.【F:drones/stationary/stationary_test.js†L5-L194】【F:drones/stationary/stationary_test.js†L707-L858】 Key elements:
- Exports `initStationaryHoverDemo` that instantiates `StationaryHoverDemo`, provides pause/resume/restart APIs, and ensures Three.js presence.【F:drones/stationary/stationary_test.js†L5-L21】
- Constructor sets simulation rate (240 Hz), initializes Crazyflie parameters, target at (0,0,0.6), controller with tuned gains, scene/camera/controls/overlay, and starts simulation.【F:drones/stationary/stationary_test.js†L38-L92】
- Visual setup includes axis helpers, grid/floor, motor/thrust arrows, HUD, and interactive target selection via raycasting; motors positions use Crazyflie-like X configuration and color coding.【F:drones/stationary/stationary_test.js†L80-L194】【F:drones/stationary/stationary_test.js†L290-L371】
- Main loop `_loop` accumulates real time, advances physics at fixed dt, and renders frames unless paused/hidden.【F:drones/stationary/stationary_test.js†L707-L722】
- `_stepPhysics` builds target, invokes controller, logs diagnostics, applies motor commands to physics engine, and steps dynamics.【F:drones/stationary/stationary_test.js†L724-L791】
- `_render` updates mesh pose, thrust arrows (desired in body frame via inverse of current quaternion), camera follow, HUD contents.【F:drones/stationary/stationary_test.js†L794-L858】 Influence: Orchestrates data flow across controller and physics and exposes monitoring hooks.

### 2.7 Helper Modules
The analyzed files rely on `THREE` global for math and rendering; no additional project-specific helpers are imported beyond the physics params/motor model and controller chain already discussed.【F:drones/stationary/stationary_test.js†L1-L3】

## 3. Control Architecture Explanation
- **Position controller (outer loop):** Computes desired acceleration in world frame: a_cmd = Kp*(p_ref - p) + Kd*(v_ref - v) + Ki*integral + a_ff + gravity in +Z; clamps magnitude and horizontal components to maxAcc and tilt limits.【F:drones/stationary/eth_controller.js†L91-L132】 Integral is windup-limited per axis.【F:drones/stationary/eth_controller.js†L95-L100】
- **Attitude controller (inner geometric SE(3)):** Desired body frame built from thrust direction b3 = normalized a_cmd (flipped if z negative) and heading b1_ref; b2 = b3 x b1_ref; b1 = b2 x b3; rotation matrix columns = (b1,b2,b3) to quaternion q_des.【F:drones/stationary/eth_controller.js†L134-L178】 Orientation error uses e_R = 0.5 * vee(R_des^T R - R^T R_des) with column-major indexing; angular velocity error assumes desired omega zero.【F:drones/stationary/eth_controller.js†L191-L218】 Control torque tau = -K_R e_R - K_omega e_omega + omega x (J omega) then clamped per axis.【F:drones/stationary/eth_controller.js†L227-L242】
- **Motor mixing/allocation:** Collective thrust T = mass * (a_cmd dot b3) (clamped) with Crazyflie X inverse allocation: f0=1/4(S-Tx-Ty-Tz), f1=1/4(S+Tx-Ty+Tz), f2=1/4(S+Tx+Ty-Tz), f3=1/4(S-Tx+Ty+Tz) where S=T/kF, Tx=tau_x*sqrt(2)/(L*kF), Ty=tau_y*sqrt(2)/(L*kF), Tz=tau_z/kM. Yaw torque dropped first on saturation, then uniform scaling, and thrusts mapped via omega limits to normalized motor commands.【F:drones/stationary/eth_controller.js†L185-L320】
- **Command propagation:** `_stepPhysics` in the demo builds target -> StationaryController -> EthController -> motorCommands; physics engine maps normalized commands to actual rotor speeds (with motor dynamics) -> thrust/torque -> integrates state; controller diagnostics and HUD use new state for next loop.【F:drones/stationary/stationary_test.js†L724-L791】【F:drones/physics/drone_physics_engine.js†L86-L176】

## 4. Physics Engine Consistency Analysis
- **Frames:** Physics uses body frame (x forward, y left, z up) for thrust vector (0,0,T) and torques; rotation matrix R_BW derived directly from body-to-world quaternion to map body forces into world frame (no transpose), implying quaternion represents body to world rotation.【F:drones/physics/drone_physics_engine.js†L108-L146】
- **Quaternions:** quatDerivative assumes q_WB (body to world) with body-frame omega, using right multiplication q * omega_quat scaled by 0.5, matching standard kinematics; normalization follows explicit Euler integration.【F:drones/physics/drone_physics_engine.js†L8-L21】【F:drones/physics/drone_physics_engine.js†L170-L176】 Potential numerical drift from Euler integration persists but normalized each step.
- **Rotation matrix indexing:** Uses Matrix3.from Matrix4(makeRotationFromQuaternion(q_WB)) and applies to body vectors, consistent with column-major orientation when applied via applyMatrix3 to thrust vector.【F:drones/physics/drone_physics_engine.js†L141-L146】
- **Motor numbering/spin:** Torque mapping assumes Crazyflie layout 0=FL(CW),1=FR(CCW),2=BR(CW),3=BL(CCW), with roll = right-left, pitch = back-front, yaw signs CW negative.【F:drones/physics/drone_physics_engine.js†L100-L132】
- **Inertia:** Diagonal inertia used; coriolis computed as omega cross (J omega); angular acceleration divides by per-axis inertia.【F:drones/physics/drone_physics_engine.js†L153-L168】
- **Force/torque application:** Linear drag in world frame subtracts proportional velocity; gravity along -Z world; thrust in body z mapped to world via R_BW; torques directly in body frame.【F:drones/physics/drone_physics_engine.js†L133-L151】
- **Integration:** Translational and angular velocities integrated via explicit Euler; quaternion via Euler on q_dot; potential integration error for large dt but mitigated by 240 Hz stepping.【F:drones/physics/drone_physics_engine.js†L92-L176】【F:drones/stationary/stationary_test.js†L707-L718】

## 5. Controller Consistency Analysis
- **Position PID math:** Gravity added in +Z; integral clamped; acceleration clamp global and horizontal tilt-limited; gains default moderate but demo overrides to higher values.【F:drones/stationary/eth_controller.js†L102-L132】【F:drones/stationary/stationary_test.js†L61-L71】 Potential mismatch if physics gravity sign differs (physics uses -Z), but controller adds +g then thrust direction uses b3; mapping relies on correct body thrust orientation.
- **Attitude error:** Uses geometric e_R with column-major vee mapping; correctness hinges on Matrix3 element order; implemented as (a=M[7], b=M[2], c=M[3]) matching skew layout, but risk of element misinterpretation remains.【F:drones/stationary/eth_controller.js†L199-L214】 Desired angular velocity zero; no feedforward yaw rate.
- **Control law sign:** Negative feedback on e_R and e_omega; coriolis added (not subtracted) consistent with tau = -K_R e_R - K_omega e_omega + omega x J omega. Gains may produce relatively large torques given small inertia.
- **Mixer inversion:** Inverse allocation matches physics forward model; uses same motor ordering and square-root-of-two arm projection. Saturation prioritizes attitude by zeroing yaw torque first, then scaling if still above T_max; final clamp ensures non-negative thrust.【F:drones/stationary/eth_controller.js†L244-L320】 Potential asymmetry if physics thrust_coeff or motor limits differ.
- **Saturation paths:** Collective thrust clipped to 4*T_max; per-motor thrust clamped to [0,T_max]; command mapping uses sqrt to omega then normalized to [0,1] using omega_min/omega_max (from params).【F:drones/stationary/eth_controller.js†L185-L320】 If tau demands exceed limits frequently, attitude could degrade; yaw dropped first mitigates but may still cause coupling.
- **Coordinates/signs:** Controller assumes body z thrust positive upward; b3 flipped if negative z to avoid inverted solutions; yaw heading uses world XY plane. Must match physics gravity direction and torque signs; both use CW negative yaw, consistent via kM sign in mixer equations.【F:drones/physics/drone_physics_engine.js†L128-L132】【F:drones/stationary/eth_controller.js†L244-L263】

## 6. Known Issues Suspicion List
- Quaternion integration uses explicit Euler; although normalized, large angular rates could introduce error or energy injection.【F:drones/physics/drone_physics_engine.js†L170-L176】
- Vee operator element mapping could be misaligned if Matrix3 element ordering differs from assumption, potentially flipping attitude error components.【F:drones/stationary/eth_controller.js†L199-L214】
- Torque clipping thresholds (0.02 N·m roll/pitch) may exceed Crazyflie actuator capability, causing frequent saturation and integrator windup in outer loop even with yaw suppression.【F:drones/stationary/eth_controller.js†L235-L304】
- Collective thrust clamp uses 4*T_max computed from kF*omega_max^2; if physics or motor model uses different limits, hover thrust scaling might mismatch leading to over/under-thrust.【F:drones/stationary/eth_controller.js†L185-L189】【F:drones/physics/drone_physics_engine.js†L86-L114】
- Coordinate note: demo UI describes motor layout with some differing text from physics mapping; inconsistency in comments vs physics may lead to confusion about motor numbering though actual code aligns with physics enumerations.【F:drones/stationary/stationary_test.js†L23-L37】【F:drones/physics/drone_physics_engine.js†L100-L132】
- Gravity handling relies on controller adding +g and physics applying -g; any sign inversion in target acceleration would cause climb/settle bias, worth verifying empirically.【F:drones/physics/drone_physics_engine.js†L141-L148】【F:drones/stationary/eth_controller.js†L102-L107】

## 7. Data Flow Diagram (ASCII)
```
[Target (pos, vel, yaw)]
          |
          v
 [SE(3) cascaded controller]
          |
          v
    [Motor mixer]
          |
          v
    [Motor model]
          |
          v
 [Physics integrator (forces/torques)]
          |
          v
      [State update]
          |
          v
 [Visualization + next control step]
```

## 8. Final Summary
The stationary demo couples an ETH-style cascaded SE(3) controller with a Crazyflie-based physics model and Three.js visualization. Position PID with gravity compensation and tilt/acceleration limits feeds a geometric attitude controller, which allocates torques/thrust to motors via an X-configuration mixer aligned with the physics engine. The physics integrator models motor lag, thrust/torque generation, drag, and rigid-body dynamics in a body-to-world frame with explicit Euler integration. Potential risk areas include quaternion/Matrix3 convention assumptions in the attitude error computation, aggressive torque clamps versus actuator capability leading to saturation, and reliance on Euler quaternion integration under high rates.

## 9. Vertical Waypoint Convergence Audit (Observed Descent Under Max Thrust)

The following issues directly explain the observed behavior where the outer loop commands maximum upward acceleration and thrust, yet the vehicle loses altitude and never reaches higher waypoints:

1) **Attitude authority gutted by demo overrides (Interface/Controller)** — The demo replaces the EthController’s nominal attitude gains (KR ≈ [4.5, 4.5, 1.0], Komega ≈ [0.10, 0.10, 0.05]) with values that are two orders of magnitude smaller on roll/pitch (KR = 0.05, Komega = 0.02).【F:drones/stationary/eth_controller.js†L47-L59】【F:drones/stationary/stationary_test.js†L67-L76】 This makes the inner loop far too weak to align the body z-axis with the commanded b3 direction. When the outer loop asks for upward acceleration (b3_des nearly vertical), the actual thrust direction lags and remains tilted, so the vertical component of thrust collapses even while motors are saturated, producing persistent altitude loss.

2) **Collective thrust ignores actual orientation (Controller)** — The controller sets thrust_des = m·||a_cmd|| purely from the desired acceleration norm, without projecting onto the current thrust axis R·e3 as in the standard SE(3) formulation.【F:drones/stationary/eth_controller.js†L288-L325】 Any attitude error (exacerbated by the weak gains above) directly reduces the vertical component of the applied force by cos(thrust misalignment). Once motors hit their per-rotor limit, no additional vertical authority remains, so even “maximum” commanded acceleration cannot arrest the descent if b3_actual is off-axis.

**Combined effect (High confidence):** Outer-loop Z error grows, a_cmd and thrust_des saturate upward, but the crippled attitude loop fails to rotate the body to align thrust with b3_des. Because thrust magnitude is computed without accounting for the misaligned body frame, the applied force projects largely sideways, yielding insufficient lift despite maximum motor commands. This precisely matches the symptom of continuous descent with thrust pegged high.

## 10. Error pinpoint (diagnosis-only)

- **Non-orthonormal attitude basis in logs:** The recorded b1 vector shows a Z component near -0.30 (e.g., b1 ≈ (-0.68, -0.67, -0.30)), which violates the orthogonality constraint b1·b3 = 0 and indicates the rotation matrix is not tracking the commanded frame. This confirms the attitude loop has effectively lost authority, aligning with the gain collapse noted above.
- **Thrust magnitude assumes perfect alignment:** Because thrust_des is derived solely from ||a_cmd||, the controller never accounts for the cosine loss between b3_actual and world z. With the motors already saturated and the thrust axis tilted, the achievable vertical force is thrust_des·cos(tilt), which is insufficient to oppose gravity—explaining “max thrust + descent.”
