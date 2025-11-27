export const CRAZYFLIE_PARAMS = {
  // Retuned to represent a visibly larger (~1.7m tall) quad while keeping the
  // control loop numerically stable.
  mass: 1.25,
  inertia: new THREE.Vector3(0.035, 0.035, 0.055),
  armLength: 0.55,
  rpmMax: 22000,
  kF: 1.5e-8,
  kM: 2.0e-10,
  motorTimeConstant: 0.02,
  dragLinear: 0.18,
};
