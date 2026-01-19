# iLQR MPC Controller Report

## Overview
A new iterative Linear Quadratic Regulator (iLQR) model predictive controller was added for the quadrotor. It inherits from the ETH geometric controller base to preserve physical parameters, mixer logic, and conventions. The controller predicts full SE(3) dynamics, computes linearizations for each horizon step, performs a Riccati backward pass to obtain feedback/feedforward gains, and executes a line-searched forward rollout for stable convergence.

## State and Control Definitions
- **State (13 elements):** `[px, py, pz, vx, vy, vz, qx, qy, qz, qw, wx, wy, wz]` using a body→world quaternion.
- **Control (4 elements):** `[thrust, tau_x, tau_y, tau_z]` with thrust along body +Z and body-frame torques.

## Continuous-Time Dynamics
- Position: `ṗ = v`
- Velocity: `v̇ = g*[0,0,-1] + (T/m) * R(q) * e3`
- Attitude: `q̇ = 0.5 * Ω(ω) * q`, where `Ω(ω)` is the quaternion multiplication matrix
- Angular rates: `ω̇ = J⁻¹ (τ - ω × (Jω))`

## Linearization (A, B)
Discrete dynamics use Euler integration `x_{k+1} = x_k + dt * f(x_k, u_k)` leading to:
- Position block: `A_pv = I3 * dt`
- Velocity control: `B_vT = (dt/m) * R * e3`
- Velocity attitude sensitivity: `A_vqvec = (dt * T/m) * d(Re3)/dq_vec` with `d(Re3)/dq_vec = -2 * R * [e3]_x`
- Quaternion drift: `A_qq = I4 + 0.5 * Ω(ω) * dt`
- Quaternion/angular-rate coupling: `A_qω = 0.5 * dt * [Ω_x*q, Ω_y*q, Ω_z*q]`
- Angular-rate Jacobian: `A_ωω` from `J⁻¹ (-ω×J - (Jω)×)`
- Torque inputs: `B_ωτ = dt * J⁻¹`

## Cost Function
Stage cost penalizes position/velocity errors, thrust-direction error (tilt), yaw-rate deviation, control effort, and control smoothness. Terminal cost increases position/velocity and tilt weights. Hover thrust `m*g` serves as control reference.

## Constraints
- `0 ≤ thrust ≤ 4*maxThrustPerMotor`
- `|τx|, |τy| ≤ maxTorque`
- `|τz| ≤ maxYawTorque`
- Tilt discouraged via thrust-direction cost and ETH tilt limits applied through mixer saturation.

## Algorithm Outline
1. **Warm start** with shifted previous control sequence or hover inputs.
2. **Nominal rollout** of nonlinear dynamics over the horizon.
3. **Linearization** to compute `A_k`, `B_k` at each step.
4. **Backward Riccati pass** to compute feedforward `k_k` and feedback `K_k` gains with small regularization.
5. **Forward line search** applying `u_k = u_nom + α k_k + K_k (x - x_nom)` with clamping to physical limits.
6. **Output** first control mixed to motor commands using the exact EthController mixer; store predicted trajectory and metrics.

## Differences vs Previous MPC
- Replaces finite-difference gradient descent with analytic iLQR (linearization + Riccati gains).
- Uses full SE(3) Jacobians, including quaternion and angular-rate coupling.
- Adds warm-start, line search, and smoothness regularization for stability.
- Reuses EthController rotor mixer to guarantee identical allocation and tilt safety.
- Provides diagnostics (cost, horizon time, predicted trajectory) for HUD reporting.

## Tuning Notes
- Horizon: 20 steps @ 0.05 s (1.0 s lookahead).
- Key weights: position 30, velocity 8, tilt 4, yaw-rate 0.2, thrust 1e-3, torque 5e-4, smoothness 5e-3.
- Regularization: 1e-6 on `Quu` for numerical stability; line search factors `[1, 0.7, 0.5, 0.3, 0.1, 0.05]`.
