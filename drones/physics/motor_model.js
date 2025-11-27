export class MotorModel {
  constructor({ rpmMax = 25000, timeConstant = 0.02 } = {}) {
    this.rpmMax = rpmMax;
    this.timeConstant = timeConstant;
    this.commands = [0, 0, 0, 0];
    this.omegas = [0, 0, 0, 0];
  }

  setCommands(c0, c1, c2, c3) {
    this.commands[0] = this._clampCommand(c0);
    this.commands[1] = this._clampCommand(c1);
    this.commands[2] = this._clampCommand(c2);
    this.commands[3] = this._clampCommand(c3);
  }

  _clampCommand(cmd) {
    if (Number.isNaN(cmd)) return 0;
    return Math.min(Math.max(cmd, 0), this.rpmMax);
  }

  step(dt) {
    const alpha = Math.max(dt / Math.max(this.timeConstant, 1e-4), 0);
    for (let i = 0; i < 4; i++) {
      const u = this._clampCommand(this.commands[i]);
      this.omegas[i] += (u - this.omegas[i]) * alpha;
    }
    return this.omegas.slice();
  }
}
