import { initDroneRaceDemo } from '../demos/drone_race/drone_race_demo.js';
import { initDroneCaveDemo } from '../demos/cave_explorer/drone_cave_demo.js';
import { initObstacleDropDemo } from '../demos/obstacle_drop/obstacle_drop_demo.js';

const demoFactories = {
  race: (container, opts) => initDroneRaceDemo(container, opts),
  cave: (container, opts) => initDroneCaveDemo(container, opts),
  obstacle: (container, opts) => initObstacleDropDemo(container, opts),
};

function setRaceControlsEnabled(enabled) {
  const panel = document.getElementById('drone-race-controls');
  if (!panel) return;
  panel.classList.toggle('opacity-50', !enabled);
  panel.classList.toggle('pointer-events-none', !enabled);
}

export function setupDroneShowcase() {
  const container = document.getElementById('drone-race-demo');
  const selector = document.getElementById('drone-demo-selector');
  const statusLabel = document.getElementById('drone-demo-status');
  const pauseBtn = document.getElementById('drone-race-toggle');
  const restartBtn = document.getElementById('drone-race-restart');

  if (!container || !selector) return;

  let activeDemo = null;
  let activeKey = selector.value || 'race';
  let visibilityObserver = null;
  let visibilityHandler = null;
  let userPaused = false;

  const attachVisibility = () => {
    if (visibilityObserver) visibilityObserver.disconnect();
    if (visibilityHandler) document.removeEventListener('visibilitychange', visibilityHandler);
    if (!activeDemo) return;

    visibilityObserver = new IntersectionObserver((entries) => {
      entries.forEach((entry) => {
        if (activeDemo.setPausedFromVisibility) {
          activeDemo.setPausedFromVisibility(entry.isIntersecting);
        } else if (entry.isIntersecting) {
          activeDemo.resume?.();
        } else {
          activeDemo.pause?.();
        }
      });
    }, { threshold: 0.2 });

    visibilityObserver.observe(container);

    visibilityHandler = () => {
      const visible = document.visibilityState === 'visible';
      if (activeDemo.setPausedFromVisibility) activeDemo.setPausedFromVisibility(visible);
      else if (visible) activeDemo.resume?.();
      else activeDemo.pause?.();
    };
    document.addEventListener('visibilitychange', visibilityHandler);
  };

  const teardownActive = () => {
    if (visibilityObserver) visibilityObserver.disconnect();
    if (visibilityHandler) document.removeEventListener('visibilitychange', visibilityHandler);
    visibilityObserver = null;
    visibilityHandler = null;
    activeDemo?.destroy?.();
    activeDemo = null;
    container.innerHTML = '';
  };

  const loadDemo = (key) => {
    const init = demoFactories[key] || demoFactories.race;
    activeKey = key;
    setRaceControlsEnabled(key === 'race');
    if (statusLabel) statusLabel.textContent =
      key === 'race' ? 'Autonomous Drone Race' : key === 'cave' ? 'Cave Explorer' : 'Obstacle Drop';
    teardownActive();
    const opts = {
      width: container.clientWidth || 960,
      height: container.clientHeight || 520,
      enableShadows: true,
      highQuality: true,
    };
    activeDemo = init(container, opts);
    userPaused = false;
    if (pauseBtn) pauseBtn.textContent = 'Pause';
    attachVisibility();
  };

  const togglePause = () => {
    if (activeKey !== 'race' || !activeDemo) return;
    if (!userPaused) {
      activeDemo.pause?.(true);
      if (pauseBtn) pauseBtn.textContent = 'Resume';
    } else {
      activeDemo.resume?.(true);
      if (pauseBtn) pauseBtn.textContent = 'Pause';
    }
    userPaused = !userPaused;
  };

  selector.addEventListener('change', (e) => {
    loadDemo(e.target.value);
  });

  if (pauseBtn) pauseBtn.addEventListener('click', togglePause);
  if (restartBtn) restartBtn.addEventListener('click', () => {
    if (activeKey === 'race') {
      activeDemo?.restart?.();
      userPaused = false;
      if (pauseBtn) pauseBtn.textContent = 'Pause';
    } else if (activeDemo?.restart) {
      activeDemo.restart();
    }
  });

  loadDemo(activeKey);
}

if (document.readyState === 'loading') {
  document.addEventListener('DOMContentLoaded', () => setupDroneShowcase());
} else {
  setupDroneShowcase();
}
