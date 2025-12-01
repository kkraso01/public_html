// Drone Race Controls HTML Generator
export function createRaceControlsPanel() {
  const panel = document.createElement('div');
  panel.id = 'drone-race-controls';
  panel.className = 'space-y-3 bg-gray-900/70 p-4 rounded-xl border border-gray-800 shadow-lg';
  
  panel.innerHTML = `
    <div class="flex items-center gap-2">
      <span class="hud-chip text-xs">Drone Race</span>
    </div>
    
    <!-- Playback Controls -->
    <div class="space-y-2">
      <div class="flex gap-2">
        <button id="drone-race-toggle" class="flex-1 px-3 py-1.5 rounded-lg bg-indigo-600 hover:bg-indigo-500 text-white font-semibold text-xs transition-colors">Pause</button>
        <button id="drone-race-restart" class="flex-1 px-3 py-1.5 rounded-lg bg-gray-800 hover:bg-gray-700 text-white font-semibold text-xs transition-colors border border-gray-700">Restart</button>
      </div>
      
      <label class="text-xs text-gray-300">
        <span class="block text-xs font-semibold text-indigo-300 mb-1">Speed</span>
        <input id="drone-race-speed-slider" type="range" min="0.5" max="2" value="1" step="0.1" class="w-full accent-indigo-500">
      </label>
    </div>

    <div class="pt-2 border-t border-gray-700">
      <h4 class="text-xs font-semibold text-indigo-300 uppercase tracking-wide mb-2">🎛️ Gains</h4>
      <div class="space-y-1.5">
        <label class="flex items-center gap-1.5 text-xs text-gray-300">
          <span class="w-12">Pos P:</span>
          <input id="drone-pid-posp" type="range" min="0.5" max="5" value="6.0" step="0.1" class="flex-1 accent-violet-500">
          <span id="drone-posp-val" class="w-10 text-right font-mono text-indigo-300 text-xs">6.00</span>
        </label>
        <label class="flex items-center gap-1.5 text-xs text-gray-300">
          <span class="w-12">Vel P:</span>
          <input id="drone-pid-velp" type="range" min="0.5" max="5" value="6.0" step="0.1" class="flex-1 accent-violet-500">
          <span id="drone-velp-val" class="w-10 text-right font-mono text-indigo-300 text-xs">6.00</span>
        </label>
        <label class="flex items-center gap-1.5 text-xs text-gray-300">
          <span class="w-12">Yaw P:</span>
          <input id="drone-pid-yawp" type="range" min="0.5" max="5" value="8.0" step="0.1" class="flex-1 accent-violet-500">
          <span id="drone-yawp-val" class="w-10 text-right font-mono text-indigo-300 text-xs">8.00</span>
        </label>
        <label class="flex items-center gap-1.5 text-xs text-gray-300">
          <span class="w-12">Alt P:</span>
          <input id="drone-pid-altp" type="range" min="0.5" max="6" value="8.0" step="0.1" class="flex-1 accent-violet-500">
          <span id="drone-altp-val" class="w-10 text-right font-mono text-indigo-300 text-xs">8.00</span>
        </label>
      </div>
    </div>

    <div class="pt-2 border-t border-gray-700">
      <h4 class="text-xs font-semibold text-emerald-300 uppercase tracking-wide mb-2">⚙️ Dynamics</h4>
      <div class="space-y-1.5">
        <label class="flex items-center gap-1.5 text-xs text-gray-300">
          <span class="w-12">Accel:</span>
          <input id="drone-maxaccel" type="range" min="5" max="30" value="20" step="0.5" class="flex-1 accent-emerald-500">
          <span id="drone-maxaccel-val" class="w-10 text-right font-mono text-emerald-300 text-xs">20.0</span>
        </label>
        <label class="flex items-center gap-1.5 text-xs text-gray-300">
          <span class="w-12">Speed:</span>
          <input id="drone-maxspeed" type="range" min="5" max="20" value="15" step="0.5" class="flex-1 accent-emerald-500">
          <span id="drone-maxspeed-val" class="w-10 text-right font-mono text-emerald-300 text-xs">15.0</span>
        </label>
        <label class="flex items-center gap-1.5 text-xs text-gray-300">
          <span class="w-12">Damp:</span>
          <input id="drone-damping" type="range" min="0.90" max="1.0" value="0.98" step="0.01" class="flex-1 accent-amber-500">
          <span id="drone-damping-val" class="w-10 text-right font-mono text-amber-300 text-xs">0.98</span>
        </label>
      </div>
    </div>

    <button id="drone-pid-reset" class="w-full mt-2 px-2 py-1.5 rounded-lg bg-gradient-to-r from-violet-900/40 to-indigo-900/40 hover:from-violet-900/60 hover:to-indigo-900/60 border border-violet-700/50 text-violet-300 font-semibold text-xs transition-colors">↻ Reset</button>
  `;

  return panel;
}
