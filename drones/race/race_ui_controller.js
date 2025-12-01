// Drone Race UI Controller - Manages all HTML controls and HUD independently
import { createRaceControlsPanel } from './race_controls_html.js';
import { createUnifiedHUD } from './unified_hud.js';

export class RaceUIController {
  constructor(demoInstance, containerElement) {
    this.demo = demoInstance;
    this.container = containerElement;
    this.userPaused = false;
    
    // Inject the controls panel into the page if it doesn't exist
    this.injectControlsPanel();
    
    // Create unified HUD inside controls panel
    this.createHUD();
    
    // Immediately bind controls - they should exist after injection
    this.initialize();
  }

  injectControlsPanel() {
    // Check if controls panel already exists
    let controlsPanel = document.getElementById('drone-race-controls');
    
    if (!controlsPanel) {
      // Create and inject the controls panel
      controlsPanel = createRaceControlsPanel();
      
      // Find the new controls container (left sidebar)
      const controlsContainer = document.getElementById('drone-race-controls-container');
      if (controlsContainer) {
        controlsContainer.appendChild(controlsPanel);
      }
    }
    this.controlsPanel = controlsPanel;
  }

  createHUD() {
    if (this.controlsPanel) {
      const hud = createUnifiedHUD(this.controlsPanel);
      if (hud) {
        this.demo.unifiedHUD = hud;
        // Bind HUD button controls after creation
        this.setupHUDButtons();
      }
    }
  }

  initialize() {
    this.setupControlListeners();
    this.setupPIDControls();
    this.setupDynamicsControls();
    this.setupVisibilityObserver();
  }

  setupHUDButtons() {
    // Individual controller selection buttons
    const geometricBtn = document.getElementById('hud-controller-geometric');
    const mpcBtn = document.getElementById('hud-controller-mpc');
    const timeoptBtn = document.getElementById('hud-controller-timeopt');
    
    const updateButtonStates = (activeMode) => {
      [geometricBtn, mpcBtn, timeoptBtn].forEach(btn => {
        if (btn) btn.classList.remove('bg-emerald-600', 'hover:bg-emerald-500');
        if (btn) btn.classList.add('bg-gray-700', 'hover:bg-gray-600');
      });
      
      if (activeMode === 'geometric' && geometricBtn) {
        geometricBtn.classList.remove('bg-gray-700', 'hover:bg-gray-600');
        geometricBtn.classList.add('bg-emerald-600', 'hover:bg-emerald-500');
      } else if (activeMode === 'mpc' && mpcBtn) {
        mpcBtn.classList.remove('bg-gray-700', 'hover:bg-gray-600');
        mpcBtn.classList.add('bg-emerald-600', 'hover:bg-emerald-500');
      } else if (activeMode === 'time-optimal' && timeoptBtn) {
        timeoptBtn.classList.remove('bg-gray-700', 'hover:bg-gray-600');
        timeoptBtn.classList.add('bg-emerald-600', 'hover:bg-emerald-500');
      }
    };
    
    if (geometricBtn) {
      geometricBtn.addEventListener('click', () => {
        this.demo._setControllerMode('geometric');
        updateButtonStates('geometric');
      });
    }
    
    if (mpcBtn) {
      mpcBtn.addEventListener('click', () => {
        this.demo._setControllerMode('mpc');
        updateButtonStates('mpc');
      });
    }
    
    if (timeoptBtn) {
      timeoptBtn.addEventListener('click', () => {
        this.demo._setControllerMode('time-optimal');
        updateButtonStates('time-optimal');
      });
    }

    // Cycle camera button
    const cycleCameraBtn = document.getElementById('hud-cycle-camera-btn');
    if (cycleCameraBtn) {
      cycleCameraBtn.addEventListener('click', () => {
        this.demo._cycleCameraMode();
      });
    }

    // Toggle debug button
    const toggleDebugBtn = document.getElementById('hud-toggle-debug-btn');
    if (toggleDebugBtn) {
      toggleDebugBtn.addEventListener('click', () => {
        this.demo.debugEnabled = !this.demo.debugEnabled;
      });
    }
  }

  setupControlListeners() {
    // Pause/Resume button
    const pauseBtn = document.getElementById('drone-race-toggle');
    if (pauseBtn) {
      pauseBtn.addEventListener('click', () => {
        this.userPaused = !this.userPaused;
        this.demo.userPaused = this.userPaused;
        pauseBtn.textContent = this.userPaused ? 'Resume' : 'Pause';
      });
    }

    // Restart button
    const restartBtn = document.getElementById('drone-race-restart');
    if (restartBtn) {
      restartBtn.addEventListener('click', () => {
        this.demo.restart();
        if (pauseBtn) pauseBtn.textContent = 'Pause';
        this.userPaused = false;
        this.demo.userPaused = false;
      });
    }

    // Playback speed slider
    const speedSlider = document.getElementById('drone-race-speed-slider');
    if (speedSlider) {
      speedSlider.addEventListener('input', (e) => {
        this.demo.timeScale = parseFloat(e.target.value);
      });
    }

    // Reset defaults button
    const resetBtn = document.getElementById('drone-pid-reset');
    if (resetBtn) {
      resetBtn.addEventListener('click', () => {
        this.resetToDefaults();
      });
    }
  }

  setupPIDControls() {
    // Position P gain (affects Kp.x which is position X)
    const posPSlider = document.getElementById('drone-pid-posp');
    const posPVal = document.getElementById('drone-posp-val');
    if (posPSlider) {
      posPSlider.addEventListener('input', (e) => {
        const value = parseFloat(e.target.value);
        if (posPVal) posPVal.textContent = value.toFixed(2);
        this.updateGains('posp', value);
      });
    }

    // Velocity P gain (affects Kp.y which is position Y)
    const velPSlider = document.getElementById('drone-pid-velp');
    const velPVal = document.getElementById('drone-velp-val');
    if (velPSlider) {
      velPSlider.addEventListener('input', (e) => {
        const value = parseFloat(e.target.value);
        if (velPVal) velPVal.textContent = value.toFixed(2);
        this.updateGains('velp', value);
      });
    }

    // Yaw P gain (affects Kp.z which is altitude/yaw)
    const yawPSlider = document.getElementById('drone-pid-yawp');
    const yawPVal = document.getElementById('drone-yawp-val');
    if (yawPSlider) {
      yawPSlider.addEventListener('input', (e) => {
        const value = parseFloat(e.target.value);
        if (yawPVal) yawPVal.textContent = value.toFixed(2);
        this.updateGains('yawp', value);
      });
    }

    // Altitude P gain (affects Kp.z)
    const altPSlider = document.getElementById('drone-pid-altp');
    const altPVal = document.getElementById('drone-altp-val');
    if (altPSlider) {
      altPSlider.addEventListener('input', (e) => {
        const value = parseFloat(e.target.value);
        if (altPVal) altPVal.textContent = value.toFixed(2);
        this.updateGains('altp', value);
      });
    }
  }

  updateGains(gainType, value) {
    if (!this.demo.controller?.eth) return;

    const kp = this.demo.controller.eth.Kp.clone();
    
    switch(gainType) {
      case 'posp':
        kp.x = value;
        break;
      case 'velp':
        kp.y = value;
        break;
      case 'yawp':
        kp.z = value;
        break;
      case 'altp':
        kp.z = value;
        break;
    }
    
    this.demo.controller.eth.Kp.copy(kp);
  }

  setupDynamicsControls() {
    // Max Acceleration
    const accelSlider = document.getElementById('drone-maxaccel');
    const accelVal = document.getElementById('drone-maxaccel-val');
    if (accelSlider) {
      accelSlider.addEventListener('input', (e) => {
        const value = parseFloat(e.target.value);
        if (accelVal) accelVal.textContent = value.toFixed(1);
        if (this.demo.controller?.eth) {
          this.demo.controller.eth.maxAcc = value;
        }
      });
    }

    // Max Speed
    const speedSlider = document.getElementById('drone-maxspeed');
    const speedVal = document.getElementById('drone-maxspeed-val');
    if (speedSlider) {
      speedSlider.addEventListener('input', (e) => {
        const value = parseFloat(e.target.value);
        if (speedVal) speedVal.textContent = value.toFixed(1);
        // Could apply to max speed constraint if needed
      });
    }

    // Damping
    const dampingSlider = document.getElementById('drone-damping');
    const dampingVal = document.getElementById('drone-damping-val');
    if (dampingSlider) {
      dampingSlider.addEventListener('input', (e) => {
        const value = parseFloat(e.target.value);
        if (dampingVal) dampingVal.textContent = value.toFixed(2);
        // Could apply damping to physics if needed
      });
    }
  }

  resetToDefaults() {
    // Reset all sliders to default values
    const defaults = {
      'drone-pid-posp': { value: '2.5', display: 'drone-posp-val' },
      'drone-pid-velp': { value: '2.2', display: 'drone-velp-val' },
      'drone-pid-yawp': { value: '2.5', display: 'drone-yawp-val' },
      'drone-pid-altp': { value: '3.0', display: 'drone-altp-val' },
      'drone-maxaccel': { value: '18', display: 'drone-maxaccel-val' },
      'drone-maxspeed': { value: '12', display: 'drone-maxspeed-val' },
      'drone-damping': { value: '0.98', display: 'drone-damping-val' },
      'drone-race-speed-slider': { value: '1' },
    };

    for (const [sliderId, config] of Object.entries(defaults)) {
      const slider = document.getElementById(sliderId);
      if (slider) {
        slider.value = config.value;
        if (config.display) {
          const display = document.getElementById(config.display);
          if (display) {
            const numVal = parseFloat(config.value);
            display.textContent = numVal > 1 ? numVal.toFixed(1) : numVal.toFixed(2);
          }
        }
      }
    }

    // Reset controller to default gains
    if (this.demo.controller) {
      this.demo.controller.updateGains({
        kp: { x: 6.0, y: 6.0, z: 8.0 },
        kd: { x: 4.0, y: 4.0, z: 5.0 },
        ki: { x: 0.12, y: 0.12, z: 0.12 },
      });
      this.demo.controller.updateAttitudeGains({
        kR: { x: 1.0, y: 1.0, z: 5.0 },
        kOmega: { x: 0.2, y: 0.2, z: 0.3 },
      });
    }

    this.demo.timeScale = 1.0;
  }

  setupVisibilityObserver() {
    if (!this.container) return;

    const observer = new IntersectionObserver((entries) => {
      entries.forEach((entry) => {
        if (!entry.isIntersecting) {
          if (!this.userPaused) {
            this.demo.visibilityPaused = true;
          }
        } else {
          this.demo.visibilityPaused = false;
        }
      });
    }, { threshold: 0.2 });

    observer.observe(this.container);

    // Also handle page visibility change
    const handleVisibilityChange = () => {
      const visible = document.visibilityState === 'visible';
      if (!visible && !this.userPaused) {
        this.demo.visibilityPaused = true;
      } else {
        this.demo.visibilityPaused = false;
      }
    };

    document.addEventListener('visibilitychange', handleVisibilityChange);
  }
}
