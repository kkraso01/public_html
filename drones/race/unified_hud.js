/**
 * Unified HUD System for Drone Race Demo
 * Consolidates all UI elements into the drone-race-controls panel
 */

export function createUnifiedHUD(controlsPanel) {
  if (!controlsPanel) {
    console.warn('[UnifiedHUD] Controls panel not found');
    return null;
  }

  // Create HUD container that will be inserted into controls panel
  const hudContainer = document.createElement('div');
  hudContainer.id = 'unified-race-hud';
  hudContainer.className = 'space-y-2 pt-2 border-t border-gray-700';
  
  hudContainer.innerHTML = `
    <!-- Race Status HUD -->
    <div class="bg-gradient-to-r from-indigo-900/20 to-purple-900/20 p-2 rounded-lg border border-indigo-700/30">
      <div class="text-xs font-semibold text-indigo-300 mb-1.5">📊 Status</div>
      <div class="grid grid-cols-2 gap-1.5 text-xs">
        <div>
          <div class="text-xs text-gray-400">State</div>
          <div id="hud-race-state" class="font-mono font-semibold text-indigo-300">COUNTDOWN</div>
        </div>
        <div>
          <div class="text-xs text-gray-400">Time</div>
          <div id="hud-race-time" class="font-mono font-semibold text-indigo-300">0.00 s</div>
        </div>
        <div>
          <div class="text-xs text-gray-400">Gate Progress</div>
          <div id="hud-race-gate" class="font-mono font-semibold text-purple-300">0 / 0</div>
        </div>
        <div>
          <div class="text-xs text-gray-400">Speed</div>
          <div id="hud-race-speed" class="font-mono font-semibold text-cyan-300">0.0 m/s</div>
        </div>
      </div>
    </div>

    <!-- Controller Info -->
    <div class="bg-gradient-to-r from-emerald-900/20 to-teal-900/20 p-2 rounded-lg border border-emerald-700/30">
      <div class="text-xs font-semibold text-emerald-300 mb-1.5">🎮 Control</div>
      <div id="hud-controller-name" class="text-xs font-bold text-emerald-300 mb-1">Geometric (ETH)</div>
      <div class="grid grid-cols-2 gap-1.5 text-xs">
        <div>
          <div class="text-gray-500">Cam</div>
          <div id="hud-camera-mode" class="font-mono text-cyan-300">CHASE</div>
        </div>
        <div>
          <div class="text-gray-500">Error</div>
          <div id="hud-tracking-error" class="font-mono text-amber-300">0.00 m</div>
        </div>
      </div>
    </div>

    <!-- Performance Metrics -->
    <div class="bg-gradient-to-r from-violet-900/20 to-fuchsia-900/20 p-2 rounded-lg border border-violet-700/30">
      <div class="text-xs font-semibold text-violet-300 mb-1.5">⚡ Perf</div>
      <div class="grid grid-cols-2 gap-1.5 text-xs">
        <div>
          <div class="text-gray-500">Vel</div>
          <div id="hud-velocity" class="font-mono font-semibold text-violet-300">0.0 m/s</div>
        </div>
        <div>
          <div class="text-gray-500">Accel</div>
          <div id="hud-acceleration" class="font-mono font-semibold text-violet-300">0.0 m/s²</div>
        </div>
        <div>
          <div class="text-gray-500">Tilt</div>
          <div id="hud-tilt-angle" class="font-mono font-semibold text-fuchsia-300">0.0°</div>
        </div>
        <div>
          <div class="text-gray-500">Alt</div>
          <div id="hud-altitude" class="font-mono font-semibold text-fuchsia-300">0.00 m</div>
        </div>
      </div>
    </div>

    <!-- Time-Optimal Metrics (hidden by default) -->
    <div id="hud-timeopt-metrics" class="bg-gradient-to-r from-orange-900/20 to-amber-900/20 p-2 rounded-lg border border-orange-700/30" style="display: none;">
      <div class="text-xs font-semibold text-orange-300 mb-1.5">🔥 Time-Opt</div>
      <div class="space-y-0.5 text-xs">
        <div class="flex justify-between">
          <span class="text-gray-500">Duration</span>
          <span id="hud-timeopt-duration" class="font-mono text-orange-300">0.0 s</span>
        </div>
        <div class="flex justify-between">
          <span class="text-gray-500">Avg</span>
          <span id="hud-timeopt-avg-speed" class="font-mono text-orange-300">0.0 m/s</span>
        </div>
        <div class="flex justify-between">
          <span class="text-gray-500">Peak</span>
          <span id="hud-timeopt-max-speed" class="font-mono text-orange-300">0.0 m/s</span>
        </div>
        <div class="flex justify-between">
          <span class="text-gray-500">Aggr</span>
          <span id="hud-timeopt-aggr" class="font-mono text-orange-300">85%</span>
        </div>
      </div>
    </div>

    <!-- Debug Panel (hidden by default) -->
    <div id="hud-debug-panel" class="bg-gray-900/60 p-2 rounded-lg border border-gray-700" style="display: none;">
      <div class="text-xs font-semibold text-cyan-300 mb-1.5">🐛 Debug</div>
      <div class="space-y-0.5 text-xs font-mono text-gray-300">
        <div id="hud-dbg-pos">Pos: 0, 0, 0</div>
        <div id="hud-dbg-vel">Vel: 0.00 m/s (0,0,0)</div>
        <div id="hud-dbg-rpm">RPM: 0 | 0 | 0 | 0</div>
        <div id="hud-dbg-thrust">Thrust: 0.0 N</div>
        <div id="hud-dbg-att">R/P/Y: 0 / 0 / 0</div>
        <div id="hud-dbg-att-err">Att Err: 0.00</div>
      </div>
    </div>

    <!-- Control Buttons -->
    <div class="bg-gray-900/40 p-3 rounded-lg border border-gray-700/50">
      <div class="text-xs font-semibold text-gray-400 uppercase tracking-wide mb-2">🎮 Controller Mode</div>
      <div class="grid grid-cols-2 gap-1.5 mb-3">
        <button id="hud-controller-geometric" class="px-2 py-1.5 rounded bg-emerald-600 hover:bg-emerald-500 text-white font-semibold text-xs transition-colors">
          Geometric
        </button>
        <button id="hud-controller-timeopt" class="px-2 py-1.5 rounded bg-gray-700 hover:bg-gray-600 text-white font-semibold text-xs transition-colors">
          Time-Opt
        </button>
      </div>
      <div class="flex gap-1.5">
        <button id="hud-cycle-camera-btn" class="flex-1 px-2 py-1.5 rounded bg-cyan-600 hover:bg-cyan-500 text-white font-semibold text-xs transition-colors">
          📷 Camera
        </button>
        <button id="hud-toggle-debug-btn" class="flex-1 px-2 py-1.5 rounded bg-amber-600 hover:bg-amber-500 text-white font-semibold text-xs transition-colors">
          🐛 Debug
        </button>
      </div>
    </div>
  `;
  
  // Insert HUD into controls panel (after existing content)
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
  
  // Race status
  if (data.state) el.raceState.textContent = data.state;
  if (data.time !== undefined) el.raceTime.textContent = `${data.time.toFixed(2)} s`;
  if (data.gateIdx !== undefined && data.gateTotal !== undefined) {
    el.raceGate.textContent = `${data.gateIdx} / ${data.gateTotal}`;
  }
  if (data.speed !== undefined) el.raceSpeed.textContent = `${data.speed.toFixed(1)} m/s`;
  
  // Controller info
  if (data.controllerMode) {
    const names = {
      'geometric': 'Geometric (ETH)',
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
  
  // Performance metrics
  if (data.velocity !== undefined) el.velocity.textContent = `${data.velocity.toFixed(1)} m/s`;
  if (data.acceleration !== undefined) el.acceleration.textContent = `${data.acceleration.toFixed(1)} m/s²`;
  if (data.tiltAngle !== undefined) el.tiltAngle.textContent = `${data.tiltAngle.toFixed(1)}°`;
  if (data.altitude !== undefined) el.altitude.textContent = `${data.altitude.toFixed(2)} m`;
  
  // Time-optimal metrics
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
  
  // Debug panel
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
      hud.container.querySelector('#hud-dbg-att-err').textContent = `Att Err: ${d.attitudeError.toFixed(3)}`;
    }
  }
}
