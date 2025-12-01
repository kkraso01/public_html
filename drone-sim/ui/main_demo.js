// Drone Race Demo is independently managed by RaceUIController in drone_race_demo.js
import { initDroneCaveDemo } from '../../drones/cave/drone_cave_demo.js';
import { initObstacleDropDemo } from '../../drones/obstacles/obstacle_sim.js';

function setupDemo(containerId, demoFactory, controlsId, toggleBtnId, restartBtnId) {
  const container = document.getElementById(containerId);
  if (!container) return null;

  let activeDemo = null;
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

  const initialize = () => {
    if (activeDemo) {
      visibilityObserver?.disconnect();
      visibilityHandler && document.removeEventListener('visibilitychange', visibilityHandler);
      activeDemo?.destroy?.();
    }

    container.innerHTML = '';
    const opts = {
      width: container.clientWidth || 960,
      height: container.clientHeight || 520,
      enableShadows: true,
      highQuality: true,
    };
    activeDemo = demoFactory(container, opts);
    userPaused = false;
    const toggleBtn = document.getElementById(toggleBtnId);
    if (toggleBtn) toggleBtn.textContent = 'Pause';
    attachVisibility();
  };

  const togglePause = () => {
    if (!activeDemo) return;
    if (!userPaused) {
      activeDemo.pause?.(true);
      const toggleBtn = document.getElementById(toggleBtnId);
      if (toggleBtn) toggleBtn.textContent = 'Resume';
    } else {
      activeDemo.resume?.(true);
      const toggleBtn = document.getElementById(toggleBtnId);
      if (toggleBtn) toggleBtn.textContent = 'Pause';
    }
    userPaused = !userPaused;
  };

  const restart = () => {
    if (activeDemo?.restart) {
      activeDemo.restart();
      userPaused = false;
      const toggleBtn = document.getElementById(toggleBtnId);
      if (toggleBtn) toggleBtn.textContent = 'Pause';
    }
  };

  // Hook up button listeners
  const toggleBtn = document.getElementById(toggleBtnId);
  const restartBtn = document.getElementById(restartBtnId);
  if (toggleBtn) toggleBtn.addEventListener('click', togglePause);
  if (restartBtn) restartBtn.addEventListener('click', restart);

  initialize();

  return { activeDemo, initialize };
}

export function setupDroneShowcase() {
  // Drone Race Demo is now independently managed by RaceUIController in drone_race_demo.js
  // It handles its own control injection and binding.

  // Setup Cave Explorer Demo
  setupDemo(
    'drone-cave-demo',
    (container, opts) => initDroneCaveDemo(container, opts),
    'drone-cave-controls',
    'drone-cave-toggle',
    'drone-cave-restart'
  );

  // Setup Obstacle Drop Demo
  setupDemo(
    'drone-obstacle-demo',
    (container, opts) => initObstacleDropDemo(container, opts),
    'drone-obstacle-controls',
    'drone-obstacle-spawn-boulder',
    'drone-obstacle-reset'
  );
}

if (document.readyState === 'loading') {
  document.addEventListener('DOMContentLoaded', () => setupDroneShowcase());
} else {
  setupDroneShowcase();
}
