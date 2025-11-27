import { CaveSim } from './cave_sim.js';
import { OBSTACLE_TYPES, listPresets } from './obstacle_presets.js';

export function initObstacleDemo(container, options = {}) {
  const sim = new CaveSim(container, options);
  sim.start();
  const panel = buildUI(container, sim);
  return { sim, panel };
}

function buildUI(container, sim) {
  const panel = document.createElement('div');
  panel.style.cssText =
    'position:absolute; top:10px; left:10px; display:flex; flex-direction:column; gap:8px; background:rgba(9,12,20,0.72); padding:10px; border:1px solid rgba(56,189,248,0.35); border-radius:10px; color:#e2e8f0; font-family:"Inter",sans-serif; z-index:20; backdrop-filter: blur(6px);';

  const title = document.createElement('div');
  title.textContent = 'Add Obstacle';
  title.style.cssText = 'font-size:12px; letter-spacing:0.1em; text-transform:uppercase; color:#38bdf8;';
  panel.appendChild(title);

  const buttons = document.createElement('div');
  buttons.style.cssText = 'display:grid; grid-template-columns:repeat(3, auto); gap:6px;';
  for (const preset of listPresets()) {
    const btn = document.createElement('button');
    btn.textContent = preset.label;
    btn.style.cssText =
      'padding:6px 10px; border:1px solid rgba(56,189,248,0.4); background:rgba(30,41,59,0.6); color:#e2e8f0; border-radius:6px; cursor:pointer; transition:all 0.2s ease;';
    btn.addEventListener('mouseenter', () => (btn.style.borderColor = '#22d3ee'));
    btn.addEventListener('mouseleave', () => (btn.style.borderColor = 'rgba(56,189,248,0.4)'));
    btn.addEventListener('click', () => sim.spawnObstacle(preset.type));
    buttons.appendChild(btn);
  }
  panel.appendChild(buttons);

  const actions = document.createElement('div');
  actions.style.cssText = 'display:flex; gap:6px; margin-top:8px;';
  const reset = document.createElement('button');
  reset.textContent = 'Reset';
  reset.style.cssText =
    'padding:6px 10px; border:1px solid rgba(74,222,128,0.45); background:rgba(21,128,61,0.35); color:#dcfce7; border-radius:6px; cursor:pointer;';
  reset.onclick = () => {
    sim.resetObstacles();
    sim.spawnObstacle(OBSTACLE_TYPES.boulder);
  };
  actions.appendChild(reset);
  panel.appendChild(actions);

  container.style.position = 'relative';
  container.appendChild(panel);
  return panel;
}
