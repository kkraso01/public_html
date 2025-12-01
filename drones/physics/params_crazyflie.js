// Physical parameters for Crazyflie 2.x quadrotor (real hardware specifications)
export const CRAZYFLIE_PARAMS = {
  // Vehicle properties
  mass: 0.032, // kg (32 grams)
  armLength: 0.046, // m (46 mm center to rotor)
  inertia: { Jxx: 2.4e-5, Jyy: 2.4e-5, Jzz: 3.5e-5 }, // kg·m² (measured values)

  // Aerodynamic/thrust coefficients (Crazyflie 2.x with 7mm coreless motors)
  thrustCoeff: 3.2e-8, // kF: thrust = kF * omega^2 (N)
  torqueCoeff: 7.5e-10, // kM: reaction torque = kM * omega^2 (N·m)

  // Linear drag in the world frame (simple proportional model)
  dragLinear: { x: 0.1, y: 0.1, z: 0.2 },

  // Motor model (Crazyflie brushed coreless motors)
  motor: {
    omegaMin: 300, // rad/s (idle)
    omegaMax: 2500, // rad/s (~24k RPM max)
    tauMotor: 0.015, // first-order time constant (s)
  },

  // Environment
  gravity: 9.81, // m/s²
};

// Convenience derived limits used by controllers/allocators
CRAZYFLIE_PARAMS.maxThrustPerMotor =
  CRAZYFLIE_PARAMS.thrustCoeff * CRAZYFLIE_PARAMS.motor.omegaMax ** 2;
