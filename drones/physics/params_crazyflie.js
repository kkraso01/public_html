// Physical parameters for a medium-size quadrotor inspired by ETH Zurich demos.
// Values are kept reasonable for a browser demo while remaining physically consistent.
export const CRAZYFLIE_PARAMS = {
  // Vehicle properties
  mass: 1.05, // kg
  armLength: 0.16, // meters (distance from center to each rotor)
  inertia: { Jxx: 0.015, Jyy: 0.015, Jzz: 0.03 }, // diagonal inertia matrix components (kg*m^2)

  // Aerodynamic/thrust coefficients
  thrustCoeff: 1.1e-6, // kF: thrust = kF * omega^2 (N)
  torqueCoeff: 2.0e-7, // kM: reaction torque = kM * omega^2 (N*m)

  // Linear drag in the world frame (simple proportional model)
  dragLinear: { x: 0.15, y: 0.25, z: 0.15 },

  // Motor model
  motor: {
    omegaMin: 200, // rad/s (idle)
    omegaMax: 2400, // rad/s (approximately 23k RPM)
    tauMotor: 0.03, // first-order time constant (s)
  },

  // Environment
  gravity: 9.81, // m/s^2
};

// Convenience derived limits used by controllers/allocators.
CRAZYFLIE_PARAMS.maxThrustPerMotor =
  CRAZYFLIE_PARAMS.thrustCoeff * CRAZYFLIE_PARAMS.motor.omegaMax * CRAZYFLIE_PARAMS.motor.omegaMax;
