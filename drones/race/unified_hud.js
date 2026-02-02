/**
 * Unified HUD System for Drone Race Demo
 * Consolidates all UI elements into the drone-race-controls panel
 */

export function createUnifiedHUD(controlsPanel) {
  if (!controlsPanel) {
    console.warn('[UnifiedHUD] Controls panel not found');
    return null;
  }

  const hudContainer = document.createElement('div');
  hudContainer.id = 'unified-race-hud';
  hudContainer.className = 'hud-controls';

  hudContainer.innerHTML = `
    <div class="hud-section">
      <div class="hud-title">Status</div>
      <div class="hud-grid">
        <div>
          <div class="hud-label">State</div>
          <div id="hud-race-state" class="hud-value">COUNTDOWN</div>
        </div>
        <div>
          <div class="hud-label">Time</div>
          <div id="hud-race-time" class="hud-value">0.00 s</div>
        </div>
        <div>
          <div class="hud-label">Gate</div>
          <div id="hud-race-gate" class="hud-value">0 / 0</div>
        </div>
        <div>
          <div class="hud-label">Speed</div>
          <div id="hud-race-speed" class="hud-value">0.0 m/s</div>
        </div>
      </div>
    </div>

    <div class="hud-section">
      <div class="hud-title">Control</div>
      <div id="hud-controller-name" class="hud-value">Geometric (ETH)</div>
      <div class="hud-grid">
        <div>
          <div class="hud-label">Cam</div>
          <div id="hud-camera-mode" class="hud-value">CHASE</div>
        </div>
        <div>
          <div class="hud-label">Error</div>
          <div id="hud-tracking-error" class="hud-value">0.00 m</div>
        </div>
      </div>
    </div>

    <div class="hud-section">
      <div class="hud-title">Perf</div>
      <div class="hud-grid">
        <div>
          <div class="hud-label">Vel</div>
          <div id="hud-velocity" class="hud-value">0.0 m/s</div>
        </div>
        <div>
          <div class="hud-label">Accel</div>
          <div id="hud-acceleration" class="hud-value">0.0 m/s²</div>
        </div>
        <div>
          <div class="hud-label">Tilt</div>
          <div id="hud-tilt-angle" class="hud-value">0.0°</div>
        </div>
        <div>
          <div class="hud-label">Alt</div>
          <div id="hud-altitude" class="hud-value">0.00 m</div>
        </div>
      </div>
    </div>

    <div id="hud-timeopt-metrics" class="hud-section" style="display: none;">
      <div class="hud-title">Time-Opt</div>
      <div class="hud-grid">
        <div>
          <div class="hud-label">Duration</div>
          <div id="hud-timeopt-duration" class="hud-value">0.0 s</div>
        </div>
        <div>
          <div class="hud-label">Avg</div>
          <div id="hud-timeopt-avg-speed" class="hud-value">0.0 m/s</div>
        </div>
        <div>
          <div class="hud-label">Peak</div>
          <div id="hud-timeopt-max-speed" class="hud-value">0.0 m/s</div>
        </div>
        <div>
          <div class="hud-label">Aggr</div>
          <div id="hud-timeopt-aggr" class="hud-value">85%</div>
        </div>
      </div>
    </div>

    <div id="hud-debug-panel" class="hud-section" style="display: none;">
      <div class="hud-title">Debug</div>
      <div class="hud-value" id="hud-dbg-pos">Pos: 0, 0, 0</div>
      <div class="hud-value" id="hud-dbg-vel">Vel: 0.00 m/s (0,0,0)</div>
      <div class="hud-value" id="hud-dbg-rpm">RPM: 0 | 0 | 0 | 0</div>
      <div class="hud-value" id="hud-dbg-thrust">Thrust: 0.0 N</div>
      <div class="hud-value" id="hud-dbg-att">R/P/Y: 0 / 0 / 0</div>
      <div class="hud-value" id="hud-dbg-att-err">Att Err: 0.00</div>
    </div>

    <div class="hud-section">
      <div class="hud-title">Controller Mode</div>
      <div class="hud-button-row">
        <button id="hud-controller-geometric" class="ui-btn ui-btn--solid">Geometric</button>
        <button id="hud-controller-timeopt" class="ui-btn">Time-Opt</button>
      </div>
      <div class="hud-button-row">
        <button id="hud-cycle-camera-btn" class="ui-btn">Camera</button>
        <button id="hud-toggle-debug-btn" class="ui-btn">Debug</button>
      </div>
    </div>
  `;

  controlsPanel.appendChild(hudContainer);

  return {
    container: hudContainer,
    elements: {
      raceState: hudContainer.querySelector('#hud-race-state'),
      raceTime: hudContainer.querySelector('#hud-race-time'),
      raceGate: hudContainer.querySelector('#hud-race-gate'),
      raceSpeed: hudContainer.querySelector('#hud-race-speed'),
      controllerName: hudContainer.querySelector('#hud-controller-name'),
      cameraMode: hudContainer.querySelector('#hud-camera-mode'),
      trackingError: hudContainer.querySelector('#hud-tracking-error'),
      velocity: hudContainer.querySelector('#hud-velocity'),
      acceleration: hudContainer.querySelector('#hud-acceleration'),
      tiltAngle: hudContainer.querySelector('#hud-tilt-angle'),
      altitude: hudContainer.querySelector('#hud-altitude'),
      timeoptMetrics: hudContainer.querySelector('#hud-timeopt-metrics'),
      debugPanel: hudContainer.querySelector('#hud-debug-panel'),
    }
  };
}

export function updateUnifiedHUD(hud, data) {
  if (!hud || !hud.elements) return;

  const el = hud.elements;

  if (data.state) el.raceState.textContent = data.state;
  if (data.time !== undefined) el.raceTime.textContent = `${data.time.toFixed(2)} s`;
  if (data.gateIdx !== undefined && data.gateTotal !== undefined) {
    el.raceGate.textContent = `${data.gateIdx} / ${data.gateTotal}`;
  }
  if (data.speed !== undefined) el.raceSpeed.textContent = `${data.speed.toFixed(1)} m/s`;

  if (data.controllerMode) {
    const names = {
      geometric: 'Geometric (ETH)',
      'time-optimal': 'Time-Optimal Racing'
    };
    el.controllerName.textContent = names[data.controllerMode] || 'Unknown';
  }

  if (data.cameraMode) {
    el.cameraMode.textContent = data.cameraMode.toUpperCase();
  }

  if (data.trackingError !== undefined) {
    el.trackingError.textContent = `${data.trackingError.toFixed(2)} m`;
  }

  if (data.velocity !== undefined) el.velocity.textContent = `${data.velocity.toFixed(1)} m/s`;
  if (data.acceleration !== undefined) el.acceleration.textContent = `${data.acceleration.toFixed(1)} m/s²`;
  if (data.tiltAngle !== undefined) el.tiltAngle.textContent = `${data.tiltAngle.toFixed(1)}°`;
  if (data.altitude !== undefined) el.altitude.textContent = `${data.altitude.toFixed(2)} m`;

  if (data.controllerMode === 'time-optimal' && data.timeOptData) {
    el.timeoptMetrics.style.display = 'block';
    if (data.timeOptData.totalDuration !== undefined) {
      hud.container.querySelector('#hud-timeopt-duration').textContent =
        `${data.timeOptData.totalDuration.toFixed(1)} s`;
    }
    if (data.timeOptData.avgSpeed !== undefined) {
      hud.container.querySelector('#hud-timeopt-avg-speed').textContent =
        `${data.timeOptData.avgSpeed.toFixed(1)} m/s`;
    }
    if (data.timeOptData.maxSpeed !== undefined) {
      hud.container.querySelector('#hud-timeopt-max-speed').textContent =
        `${data.timeOptData.maxSpeed.toFixed(1)} m/s`;
    }
    if (data.timeOptData.aggressiveness !== undefined) {
      hud.container.querySelector('#hud-timeopt-aggr').textContent =
        `${(data.timeOptData.aggressiveness * 100).toFixed(0)}%`;
    }
  } else {
    el.timeoptMetrics.style.display = 'none';
  }

  if (data.debugEnabled !== undefined) {
    el.debugPanel.style.display = data.debugEnabled ? 'block' : 'none';
  }

  if (data.debugData && data.debugEnabled) {
    const d = data.debugData;
    if (d.position) {
      hud.container.querySelector('#hud-dbg-pos').textContent =
        `Pos: ${d.position.x.toFixed(2)}, ${d.position.y.toFixed(2)}, ${d.position.z.toFixed(2)}`;
    }
    if (d.velocity) {
      hud.container.querySelector('#hud-dbg-vel').textContent =
        `Vel: ${d.velocityMag?.toFixed(2)} m/s (${d.velocity.x.toFixed(2)}, ${d.velocity.y.toFixed(2)}, ${d.velocity.z.toFixed(2)})`;
    }
    if (d.motorRPM) {
      hud.container.querySelector('#hud-dbg-rpm').textContent =
        `RPM: ${d.motorRPM.map(r => r.toFixed(0)).join(' | ')}`;
    }
    if (d.thrust !== undefined) {
      hud.container.querySelector('#hud-dbg-thrust').textContent = `Thrust: ${d.thrust.toFixed(2)} N`;
    }
    if (d.attitude) {
      hud.container.querySelector('#hud-dbg-att').textContent =
        `R/P/Y: ${d.attitude.roll.toFixed(1)} / ${d.attitude.pitch.toFixed(1)} / ${d.attitude.yaw.toFixed(1)}`;
    }
    if (d.attitudeError !== undefined) {
      hud.container.querySelector('#hud-dbg-att-err').textContent = `Att Err: ${d.attitudeError.toFixed(2)}`;
    }
  }
}
