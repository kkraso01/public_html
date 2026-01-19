# iLQR MPC Overview

This controller implements an ETH-style SE(3) iterative Linear Quadratic Regulator (iLQR) to avoid the hover local minimum and generate forward-leaning torques toward waypoints.

## State and Control Vectors
- **State (13D)**: `[p_x, p_y, p_z, v_x, v_y, v_z, q_x, q_y, q_z, q_w, \omega_x, \omega_y, \omega_z]`
  - Position `p` and velocity `v` in world frame (\+Z up, \+X forward, \+Y left).
  - Unit quaternion `q` mapping body → world (body \+Z is thrust direction).
  - Body angular velocity `\omega`.
- **Control (4D)**: `[T, \tau_x, \tau_y, \tau_z]`
  - Collective thrust and body torques before motor mixing.

## Dynamics (discrete Euler integration)
- Translational: `\dot{p} = v`, `\dot{v} = g + (T/m) R(q) e_3`
- Attitude: `\dot{q} = 0.5\,[0, \omega] ⊗ q`
- Angular rate: `\dot{\omega} = J^{-1}(\tau - \omega × J\omega)`
- The simulator integrates with fixed step `dt = predictionDt` using the same frames and gravity sign as `physics.js`.

## Linearization
For each horizon step the dynamics are linearized about the nominal `(x_k, u_k)`:
- `A_k = ∂f/∂x |_{x_k,u_k}`
- `B_k = ∂f/∂u |_{x_k,u_k}`

Key Jacobian terms:
- Position block: identity with `dt` on the `v` rows.
- Acceleration sensitivity to attitude via `∂(R e_3)/∂q` (analytic cross-product form).
- Quaternion kinematics sensitivity to `\omega` through the `Ω(\omega)` matrix.
- Angular rate sensitivities include the gyroscopic coupling `(Jω)` terms.
- Control Jacobians scale thrust and torques by `dt / m` and `dt / J` respectively.

## Cost Function
Quadratic costs encourage forward motion toward the waypoint and smooth inputs:
- Stage cost
  - Position and velocity errors weighted by `wPosition`, `wVelocity`.
  - Tilt error between current thrust axis and reference acceleration direction (`wTilt`).
  - Yaw rate penalty `wYawRate`.
  - Input effort around hover thrust `wThrust`, torque effort `wTorque`.
  - Input rate penalty `wSmooth`.
- Terminal cost
  - Higher weights on final position/velocity (`wTerminalPosition`, `wTerminalVelocity`) and tilt.

## Reference Trajectory
- Builds position **and velocity** goals toward the waypoint: `v_des = refVelocityGain · (p_target − p_start)` clamped by `maxRefVelocity`.
- A simple first-order model produces accelerating velocities: `a_ref = refAccelGain · (v_des − v)` clipped by `maxRefAcceleration`; velocity and position are propagated along the horizon.
- Non-zero `a_ref` feeds the tilt cost so the optimizer leans toward the waypoint instead of hovering.

## iLQR Steps
1. **Warm start** from previous solution or hover thrust.
2. **Rollout** nominal trajectory and cost.
3. **Linearize** dynamics along the trajectory (`A_k`, `B_k`).
4. **Backward Riccati pass** to compute feedforward `k_k` and feedback gains `K_k` (regularized inverse).
5. **Forward line-search rollout** applying `u = u_nom + α k_k + K_k (x - x_nom)`, choosing the lowest-cost candidate.

## Tuning Knobs
- Horizon length `mpcHorizon`, step `predictionDt`.
- Cost weights `wPosition`, `wVelocity`, `wTilt`, `wYawRate`, `wThrust`, `wTorque`, `wSmooth`, terminal weights.
- Reference shaping: `refVelocityGain`, `maxRefVelocity`, `refAccelGain`, `maxRefAcceleration` to set approach aggressiveness and allowable tilt/acceleration.

These pieces remove the hover local minimum and drive non-zero roll/pitch torques that move the drone toward waypoints while respecting ETH conventions and the existing motor mixer.
