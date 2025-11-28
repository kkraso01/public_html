// Simple first-order motor response model for four rotors.
export class MotorModel {
  constructor(params = {}) {
    const { omegaMin = 0, omegaMax = 2000, tauMotor = 0.02 } = params;
    this.omegaMin = omegaMin;
    this.omegaMax = omegaMax;
    this.tauMotor = Math.max(tauMotor, 1e-4);
    this.omega = [0, 0, 0, 0];
    this.commands = [0, 0, 0, 0];
  }

  reset() {
    this.omega = [0, 0, 0, 0];
    this.commands = [0, 0, 0, 0];
  }

  step(dt, omegaCmd = [0, 0, 0, 0]) {
    for (let i = 0; i < 4; i++) {
      const cmd = this._clampOmega(omegaCmd[i] ?? 0);
      const domega = (cmd - this.omega[i]) / this.tauMotor;
      this.omega[i] = this._clampOmega(this.omega[i] + domega * dt);
      this.commands[i] = cmd;
    }
    return this.getSpeeds();
  }

  getSpeeds() {
    return this.omega.slice();
  }

  _clampOmega(value) {
    if (!Number.isFinite(value)) return this.omegaMin;
    return Math.min(Math.max(value, this.omegaMin), this.omegaMax);
  }
}
