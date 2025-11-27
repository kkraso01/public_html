export function createRaceHUD(container) {
  const overlay = document.createElement('div');
  overlay.style.cssText =
    'position:absolute; top:8px; left:8px; background:rgba(8,12,24,0.85); color:#e2e8f0; padding:10px; font-family:"Fira Code", monospace; border:1px solid rgba(99,102,241,0.5); border-radius:8px; backdrop-filter: blur(8px); z-index:5; min-width:180px;';
  overlay.innerHTML = `
    <div style="font-size:12px; color:#a5b4fc; margin-bottom:4px;">Autonomous Drone Race</div>
    <div id="raceState" style="font-size:12px; margin-bottom:4px;">State: COUNTDOWN</div>
    <div id="raceTime" style="font-size:12px;">t = 0.00 s</div>
    <div id="raceGate" style="font-size:12px;">Gate: 0 / 0</div>
    <div id="raceSpeed" style="font-size:12px;">Speed: 0.0 m/s</div>
  `;
  container.style.position = 'relative';
  container.appendChild(overlay);
  return overlay;
}

export function updateHUD(overlay, { state, time, gateIdx, gateTotal, speed }) {
  if (!overlay) return;
  overlay.querySelector('#raceState').textContent = `State: ${state}`;
  overlay.querySelector('#raceTime').textContent = `t = ${time.toFixed(2)} s`;
  overlay.querySelector('#raceGate').textContent = `Gate: ${gateIdx} / ${gateTotal}`;
  overlay.querySelector('#raceSpeed').textContent = `Speed: ${speed.toFixed(1)} m/s`;
}
