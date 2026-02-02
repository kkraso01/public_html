// Drone Race Controls HTML Generator
export function createRaceControlsPanel() {
  const panel = document.createElement('div');
  panel.id = 'drone-race-controls';
  panel.className = 'hud-panel';

  panel.innerHTML = `
    <div class="hud-button-row">
      <span class="hud-chip">Drone Race</span>
    </div>

    <div class="hud-section">
      <div class="hud-button-row">
        <button id="drone-race-toggle" class="ui-btn">Pause</button>
        <button id="drone-race-restart" class="ui-btn">Restart</button>
      </div>

      <label class="hud-label">
        <span class="hud-title" style="display:block;margin-bottom:6px;">Speed</span>
        <input id="drone-race-speed-slider" class="ui-input" type="range" min="0.5" max="2" value="1" step="0.1">
      </label>
    </div>

    <div class="hud-section">
      <h4 class="hud-title">Gains</h4>
      <div class="hud-controls">
        <label class="hud-label">
          <span>Pos P:</span>
          <input id="drone-pid-posp" class="ui-input" type="range" min="0.5" max="5" value="6.0" step="0.1">
          <span id="drone-posp-val" class="hud-value">6.00</span>
        </label>
        <label class="hud-label">
          <span>Vel P:</span>
          <input id="drone-pid-velp" class="ui-input" type="range" min="0.5" max="5" value="6.0" step="0.1">
          <span id="drone-velp-val" class="hud-value">6.00</span>
        </label>
        <label class="hud-label">
          <span>Yaw P:</span>
          <input id="drone-pid-yawp" class="ui-input" type="range" min="0.5" max="5" value="8.0" step="0.1">
          <span id="drone-yawp-val" class="hud-value">8.00</span>
        </label>
        <label class="hud-label">
          <span>Alt P:</span>
          <input id="drone-pid-altp" class="ui-input" type="range" min="0.5" max="6" value="8.0" step="0.1">
          <span id="drone-altp-val" class="hud-value">8.00</span>
        </label>
      </div>
    </div>

    <div class="hud-section">
      <h4 class="hud-title">Dynamics</h4>
      <div class="hud-controls">
        <label class="hud-label">
          <span>Accel:</span>
          <input id="drone-maxaccel" class="ui-input" type="range" min="5" max="30" value="20" step="0.5">
          <span id="drone-maxaccel-val" class="hud-value">20.0</span>
        </label>
        <label class="hud-label">
          <span>Speed:</span>
          <input id="drone-maxspeed" class="ui-input" type="range" min="5" max="20" value="15" step="0.5">
          <span id="drone-maxspeed-val" class="hud-value">15.0</span>
        </label>
        <label class="hud-label">
          <span>Damp:</span>
          <input id="drone-damping" class="ui-input" type="range" min="0.90" max="1.0" value="0.98" step="0.01">
          <span id="drone-damping-val" class="hud-value">0.98</span>
        </label>
      </div>
    </div>

    <button id="drone-pid-reset" class="ui-btn ui-btn--solid">↻ Reset</button>
  `;

  return panel;
}
