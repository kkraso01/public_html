(function () {
  const canvas = document.getElementById("line-following-canvas");
  const robotEl = document.getElementById("line-following-robot");
  const sensorEls = Array.from(document.querySelectorAll("[data-sensor-index]"));
  const pEl = document.getElementById("pid-p");
  const iEl = document.getElementById("pid-i");
  const dEl = document.getElementById("pid-d");
  const errorEl = document.getElementById("pid-error");
  const speedEl = document.getElementById("pid-speed");
  const bus = window.EventBus || { emit: () => {}, on: () => {} };

  if (!canvas || !robotEl) return;

  const ctx = canvas.getContext("2d");
  let dpr = window.devicePixelRatio || 1;
  
  // Control element references
  const kpSlider = document.getElementById("kp-slider");
  const kiSlider = document.getElementById("ki-slider");
  const kdSlider = document.getElementById("kd-slider");
  const kpValue = document.getElementById("kp-value");
  const kiValue = document.getElementById("ki-value");
  const kdValue = document.getElementById("kd-value");
  const speedSlider = document.getElementById("speed-slider");
  const speedValue = document.getElementById("speed-value");
  const noiseToggle = document.getElementById("noise-toggle");
  const antiWindupToggle = document.getElementById("anti-windup-toggle");
  const dFilterToggle = document.getElementById("d-filter-toggle");
  const adaptiveSpeedToggle = document.getElementById("adaptive-speed-toggle");
  const resetBtn = document.getElementById("pid-reset-btn");
  
  // Default PID gains and speed
  const DEFAULT_GAINS = { kp: 2, ki: 0.42, kd: 0.32 };
  const DEFAULT_SPEED = 120;
  
  // Animation control
  let isRunning = false;
  let intersectionObserver = null;

  // Viewport observer for performance
  const control = { isRunning: true };
  window.ViewportObserver.observe(canvas, control, 0.1);

  const track = {
    center: { x: 0, y: 0 },
    radius: 140,
    width: 18,
    path: []
  };

  const sensorOffsets = [
    { angle: -0.54, distance: 50 },
    { angle: -0.47, distance: 50 },
    { angle: -0.40, distance: 50 },
    { angle: -0.33, distance: 50 },
    { angle: -0.26, distance: 50 },
    { angle: -0.19, distance: 50 },
    { angle: -0.12, distance: 50 },
    { angle: -0.05, distance: 50 },
    { angle: 0.05, distance: 50 },
    { angle: 0.12, distance: 50 },
    { angle: 0.19, distance: 50 },
    { angle: 0.26, distance: 50 },
    { angle: 0.33, distance: 50 },
    { angle: 0.40, distance: 50 },
    { angle: 0.47, distance: 50 },
    { angle: 0.54, distance: 50 }
  ];

  // const weights = sensorOffsets.map((_, i) => i - (sensorOffsets.length - 1) / 2);
  const weights = sensorOffsets.map(s => s.angle);

  const gains = { ...DEFAULT_GAINS };

  const state = {
    x: 0,
    y: 0,
    angle: -Math.PI / 2,
    speed: DEFAULT_SPEED,
    integral: 0,
    prevError: 0,
    prevDeriv: null,
    // --- simulation state ---
    vL: DEFAULT_SPEED,   // left wheel speed (px/s)
    vR: DEFAULT_SPEED,   // right wheel speed (px/s)
    omega: 0  // angular velocity (rad/s)
  };
  
  // Feature flags and configuration
  const features = {
    enableNoise: false,
    enableAntiWindup: true,
    enableDFilter: false,
    enableAdaptiveSpeed: false
  };

  let dragging = false;
  let lastTime = performance.now();

  // function buildTrackPath() {
  //   track.path = [];
  //   const cx = track.center.x;
  //   const cy = track.center.y;
    
  //   // Get canvas dimensions
  //   const w = canvas.width / dpr;
  //   const h = canvas.height / dpr;
  //   const margin = 50;
    
  //   // Oval track with different radii for width and height
  //   const radiusX = w / 2 - margin;
  //   const radiusY = h / 2 - margin;
    
  //   // Draw an ellipse
  //   const steps = 360;
  //   for (let i = 0; i <= steps; i++) {
  //     const angle = (Math.PI * 2) * (i / steps);
  //     const x = cx + radiusX * Math.cos(angle);
  //     const y = cy + radiusY * Math.sin(angle);
  //     track.path.push({ x, y });
  //   }
  // }
function buildTrackPath() {
  track.path = [];
  const cx = track.center.x;
  const cy = track.center.y;

  // Get canvas dimensions
  const w = canvas.width / dpr;
  const h = canvas.height / dpr;
  const margin = 50;

  const radiusX = w / 2 - margin;
  const radiusY = h / 2 - margin;

  // Generate random control points around a circle
  const pointsCount = 5;
  const pts = [];

  for (let i = 0; i < pointsCount; i++) {
    const angle = Math.random() * Math.PI * 2;

    const rx = radiusX * (0.65 + Math.random() * 0.35);
    const ry = radiusY * (0.65 + Math.random() * 0.35);

    pts.push({
      angle,
      x: cx + rx * Math.cos(angle),
      y: cy + ry * Math.sin(angle)
    });
  }

  // Sort by angle to avoid crossings
  pts.sort((a, b) => a.angle - b.angle);

  // Loop back to start
  pts.push(pts[0]);

  // Smooth curve using cosine interpolation
  const smoothSteps = 20; // more = smoother
  for (let i = 0; i < pts.length - 1; i++) {
    const p0 = pts[i];
    const p1 = pts[i + 1];

    for (let t = 0; t <= 1; t += 1 / smoothSteps) {
      // cosine interpolation factor
      const ft = (1 - Math.cos(t * Math.PI)) / 2;

      const x = p0.x * (1 - ft) + p1.x * ft;
      const y = p0.y * (1 - ft) + p1.y * ft;

      track.path.push({ x, y });
    }
  }
}

  function resize() {
    const rect = canvas.getBoundingClientRect();
    dpr = window.devicePixelRatio || 1;
    canvas.width = rect.width * dpr;
    canvas.height = rect.height * dpr;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);

    track.center.x = rect.width / 2;
    track.center.y = rect.height / 2;
    track.radius = Math.min(rect.width, rect.height) / 2 - 46;
    buildTrackPath();

    if (state.x === 0 && state.y === 0) {
      const start = track.path[0] || { x: track.center.x + track.radius, y: track.center.y };
      state.x = start.x;
      state.y = start.y;
    }
  }

  resize();
  window.addEventListener("resize", resize);

  function clamp(value, min, max) {
    return Math.min(Math.max(value, min), max);
  }

  function gaussianRandom() {
    let u = 0, v = 0;
    while (u === 0) u = Math.random();
    while (v === 0) v = Math.random();
    return Math.sqrt(-2.0 * Math.log(u)) * Math.cos(2.0 * Math.PI * v);
  }

  function getSensorPositions() {
    return sensorOffsets.map((offset) => {
      const theta = state.angle + offset.angle;
      return {
        x: state.x + Math.cos(theta) * offset.distance,
        y: state.y + Math.sin(theta) * offset.distance,
        theta
      };
    });
  }

  function sampleSensors(positions) {
    const readings = positions.map((pos) => {
      const distToLine = distanceToTrack(pos.x, pos.y);
      let normalized = 1 - clamp(distToLine / (track.width / 2 + 10), 0, 1);
      
      // Add Gaussian noise if enabled
      if (features.enableNoise) {
        const noise = gaussianRandom() * 0.05;
        normalized = clamp(normalized + noise, 0, 1);
      }
      
      return Number(normalized.toFixed(2));
    });

    bus.emit("robot:sensors", { readings, positions });
    return readings;
  }

  function drawTrack(sensorPositions, readings) {
    const w = canvas.width / dpr;
    const h = canvas.height / dpr;

    ctx.clearRect(0, 0, w, h);
    ctx.fillStyle = "rgba(17, 24, 39, 0.75)";
    ctx.fillRect(0, 0, w, h);

    if (track.path.length) {
      ctx.beginPath();
      track.path.forEach((pt, idx) => {
        if (idx === 0) ctx.moveTo(pt.x, pt.y);
        else ctx.lineTo(pt.x, pt.y);
      });
      ctx.closePath();

      ctx.strokeStyle = "#0f172a";
      ctx.lineWidth = track.width + 12;
      ctx.stroke();

      ctx.strokeStyle = "#0b0b0b";
      ctx.lineWidth = track.width;
      ctx.stroke();
    }

    sensorPositions.forEach((pos, idx) => {
      ctx.beginPath();
      ctx.fillStyle = `rgba(79,70,229,${0.3 + readings[idx] * 0.5})`;
      ctx.arc(pos.x, pos.y, 3.5, 0, Math.PI * 2);
      ctx.fill();
    });
  }

  function updatePID(dt, error, hasSignal) {
    // Proportional term
    const pTerm = gains.kp * error;

    // Integral term with anti-windup
    if (dragging) {
      state.integral = 0;
    } else if (hasSignal) {
      const integralGain = state.integral + error * dt;
      if (features.enableAntiWindup) {
        // Conditional integration: only accumulate if not saturated
        const control = gains.ki * integralGain;
        if (Math.abs(control + pTerm) < 3.2) {
          state.integral = clamp(integralGain, -14, 14);
        }
      } else {
        state.integral = clamp(integralGain, -14, 14);
      }
    } else {
      state.integral *= 0.8;
    }

    const iTerm = gains.ki * state.integral;

    // Derivative term with optional low-pass filtering
    let derivative = (error - state.prevError) / dt;
    if (features.enableDFilter && state.prevDeriv !== null) {
      // First-order low-pass filter with smooth blending
      const alpha = 0.4; // Increased from 0.3 for better responsiveness
      derivative = alpha * derivative + (1 - alpha) * state.prevDeriv;
    }
    state.prevDeriv = derivative; // Store for next iteration

    const dTerm = gains.kd * derivative;
    const control = clamp(pTerm + iTerm + dTerm, -3.2, 3.2);

    state.prevError = error;

    bus.emit("robot:pid", {
      p: pTerm,
      i: iTerm,
      d: dTerm,
      error
    });

    return control;
  }


  // Lost line recovery state
  let recoveryMode = false;
  let lastErrorSign = 0;
  let lastSpeed = state.speed;
  function updateRobot(dt, control, hasSignal) {
    // Base forward speed from slider / adaptive logic
    let speed = state.speed;
    if (features.enableAdaptiveSpeed) {
      const errorMagnitude = Math.abs(state.prevError);
      const k = 0.7;
      speed = state.speed * (0.2 + 0.8 * Math.exp(-k * Math.pow(errorMagnitude, 0.9)));
    }

    // --- line lost / recovery mode logic preserved ---
    if (!hasSignal) {
      recoveryMode = true;
    } else {
      recoveryMode = false;
    }

    // --- initialise extra simulation state if needed ---
    if (state.vL == null) state.vL = speed;
    if (state.vR == null) state.vR = speed;
    if (state.omega == null) state.omega = 0;

    // --- lightweight "physics" parameters ---
    const wheelBase     = 80;     // distance between wheels (pixels, just a scale factor)
    const motorAccel    = 10;     // how fast wheels can change speed (1/s)
    const rotResponse   = 18;     // how fast robot follows commanded turn (1/s)
    const slipFactor    = 0.0006; // how strongly high speed + high turn cause slip
    const maxSlip       = 0.4;    // cap slip to avoid crazy behaviour
    const steeringGain  = 100;     // how strongly PID control affects wheel speed difference

    // --- steering command from PID ---
    let steering = control; // PID output is roughly in [-3.2, 3.2]

    // Keep your original recovery behaviour, but expressed as steering
    if (recoveryMode) {
      const turnDir = lastErrorSign !== 0 ? Math.sign(lastErrorSign) : 1;
      steering = turnDir * 2.2; // same 2.2 you used before, just as steering now
    }

    // --- target wheel speeds (differential drive) ---
    const targetVL = speed - steeringGain * steering;
    const targetVR = speed + steeringGain * steering;

    // --- motor acceleration / inertia (no instant changes) ---
    const motorBlend = clamp(motorAccel * dt, 0, 1);
    state.vL += (targetVL - state.vL) * motorBlend;
    state.vR += (targetVR - state.vR) * motorBlend;

    // --- forward & angular velocities from wheel speeds ---
    let vForward = 0.5 * (state.vL + state.vR);
    let omegaCmd = (state.vR - state.vL) / wheelBase;

    // --- simple slip model: big turns at high speed reduce effective turning ---
    const slip = clamp(Math.abs(omegaCmd * vForward) * slipFactor, 0, maxSlip);
    const grip = 1 - slip * 0.85;   // keep a bit of turn even when slipping
    omegaCmd *= grip;

    // --- angular inertia: robot doesn't instantly match omegaCmd ---
    const rotBlend = clamp(rotResponse * dt, 0, 1);
    state.omega += (omegaCmd - state.omega) * rotBlend;

    // --- integrate pose ---
    state.angle += state.omega * dt;
    state.x += Math.cos(state.angle) * vForward * dt;
    state.y += Math.sin(state.angle) * vForward * dt;

    // --- clamp to canvas bounds (unchanged) ---
    const rect = canvas.getBoundingClientRect();
    state.x = clamp(state.x, 20, rect.width - 20);
    state.y = clamp(state.y, 20, rect.height - 20);

    // --- DOM updates (unchanged except using new angle) ---
    robotEl.style.left = `${state.x}px`;
    robotEl.style.top = `${state.y}px`;

    const adjustedAngle = state.angle - Math.PI * 1;
    robotEl.style.transform = `translate(-50%, -50%) rotate(${adjustedAngle}rad)`;

    // Report *actual* forward speed for UI
    lastSpeed = Math.abs(vForward);
  }

  function computeError(readings) {
    const total = readings.reduce((sum, r) => sum + r, 0);
    if (total === 0) return { error: state.prevError * 0.7, hasSignal: false };
    const weighted = readings.reduce((sum, r, idx) => sum + r * weights[idx], 0);
    return { error: weighted / total, hasSignal: true };
  }

  function pointToSegmentDistance(px, py, ax, ay, bx, by) {
    const dx = bx - ax;
    const dy = by - ay;
    const lenSq = dx * dx + dy * dy;
    const t = lenSq === 0 ? 0 : clamp(((px - ax) * dx + (py - ay) * dy) / lenSq, 0, 1);
    const projX = ax + t * dx;
    const projY = ay + t * dy;
    return Math.hypot(projX - px, projY - py);
  }

  function distanceToTrack(x, y) {
    if (!track.path.length) {
      return Math.abs(Math.hypot(x - track.center.x, y - track.center.y) - track.radius);
    }

    let minDist = Infinity;
    for (let i = 0; i < track.path.length; i++) {
      const a = track.path[i];
      const b = track.path[(i + 1) % track.path.length];
      const dist = pointToSegmentDistance(x, y, a.x, a.y, b.x, b.y);
      if (dist < minDist) minDist = dist;
    }
    return minDist;
  }

  function loop(now) {
    if (!control.isRunning) {
      requestAnimationFrame(loop);
      return;
    }

    const dt = clamp((now - lastTime) / 1000, 0.001, 0.05);
    lastTime = now;

    const positions = getSensorPositions();
    const readings = sampleSensors(positions);
    const { error, hasSignal } = computeError(readings);

    if (!dragging) {
      const pidControl = updatePID(dt, error, hasSignal);
      // Track last error sign for recovery
      if (hasSignal && error !== 0) lastErrorSign = error;
      updateRobot(dt, pidControl, hasSignal);
    } else {
      updatePID(dt, error, hasSignal);
    }

    drawTrack(positions, readings);
    
    if (isRunning) {
      requestAnimationFrame(loop);
    }
  }

  function syncSensorsUI() {
    bus.on("robot:sensors", ({ readings }) => {
      sensorEls.forEach((el, idx) => {
        const val = readings[idx] ?? 0;
        el.textContent = val.toFixed(2);
        el.classList.remove("metric-flash");
        if (val > 0.75) {
          el.classList.add("metric-flash");
          setTimeout(() => el.classList.remove("metric-flash"), 300);
        }
      });
    });

    bus.on("robot:pid", ({ p, i, d, error }) => {
      if (pEl) pEl.textContent = p.toFixed(2);
      if (iEl) iEl.textContent = i.toFixed(2);
      if (dEl) dEl.textContent = d.toFixed(2);
      if (errorEl) errorEl.textContent = error.toFixed(2);
      if (speedEl) speedEl.textContent = lastSpeed.toFixed(2);
    });
  }

  function enableDrag() {
    const handlePointerMove = (evt) => {
      if (!dragging) return;
      const rect = canvas.getBoundingClientRect();
      state.x = clamp(evt.clientX - rect.left, 20, rect.width - 20);
      state.y = clamp(evt.clientY - rect.top, 20, rect.height - 20);
      robotEl.style.left = `${state.x}px`;
      robotEl.style.top = `${state.y}px`;
    };

    const handlePointerUp = (evt) => {
      dragging = false;
      lastTime = performance.now();
      robotEl.releasePointerCapture(evt.pointerId);
      robotEl.style.cursor = "grab";
    };

    robotEl.addEventListener("pointerdown", (evt) => {
      dragging = true;
      state.integral = 0;
      state.prevError = 0;
      lastTime = performance.now();
      // Do not reset speed or gains here; keep user settings
      robotEl.setPointerCapture(evt.pointerId);
      robotEl.style.cursor = "grabbing";
    });

    window.addEventListener("pointermove", handlePointerMove);
    window.addEventListener("pointerup", handlePointerUp);
  }

  enableDrag();
  syncSensorsUI();
  robotEl.style.left = `${state.x}px`;
  robotEl.style.top = `${state.y}px`;
  const adjustedAngle = state.angle - Math.PI * 0.75;
  robotEl.style.transform = `translate(-50%, -50%) rotate(${adjustedAngle}rad)`;

  // --- Synchronize UI with gains on load ---
  if (kpSlider) kpSlider.value = gains.kp;
  if (kiSlider) kiSlider.value = gains.ki;
  if (kdSlider) kdSlider.value = gains.kd;
  if (kpValue) kpValue.textContent = gains.kp.toFixed(2);
  if (kiValue) kiValue.textContent = gains.ki.toFixed(2);
  if (kdValue) kdValue.textContent = gains.kd.toFixed(2);
  
  // Initialize control panel
  function setupControlPanel() {
    // PID Gain Sliders
    if (kpSlider) {
      kpSlider.addEventListener("input", (e) => {
        gains.kp = parseFloat(e.target.value);
        kpValue.textContent = gains.kp.toFixed(2);
      });
    }

    if (kiSlider) {
      kiSlider.addEventListener("input", (e) => {
        gains.ki = parseFloat(e.target.value);
        kiValue.textContent = gains.ki.toFixed(2);
      });
    }

    if (kdSlider) {
      kdSlider.addEventListener("input", (e) => {
        gains.kd = parseFloat(e.target.value);
        kdValue.textContent = gains.kd.toFixed(2);
      });
    }

    // Speed Slider
    if (speedSlider) {
      speedSlider.addEventListener("input", (e) => {
        state.speed = parseFloat(e.target.value);
        speedValue.textContent = state.speed + " px/s";
      });
    }

    // Feature Toggles
    if (noiseToggle) {
      noiseToggle.addEventListener("change", (e) => {
        features.enableNoise = e.target.checked;
      });
    }

    if (antiWindupToggle) {
      antiWindupToggle.addEventListener("change", (e) => {
        features.enableAntiWindup = e.target.checked;
      });
    }

    if (dFilterToggle) {
      dFilterToggle.addEventListener("change", (e) => {
        features.enableDFilter = e.target.checked;
      });
    }

    if (adaptiveSpeedToggle) {
      adaptiveSpeedToggle.addEventListener("change", (e) => {
        features.enableAdaptiveSpeed = e.target.checked;
      });
    }

    // Reset Button
    if (resetBtn) {
      resetBtn.addEventListener("click", () => {
        gains.kp = DEFAULT_GAINS.kp;
        gains.ki = DEFAULT_GAINS.ki;
        gains.kd = DEFAULT_GAINS.kd;
        state.speed = DEFAULT_SPEED;
        
        if (kpSlider) kpSlider.value = DEFAULT_GAINS.kp;
        if (kiSlider) kiSlider.value = DEFAULT_GAINS.ki;
        if (kdSlider) kdSlider.value = DEFAULT_GAINS.kd;
        if (speedSlider) speedSlider.value = DEFAULT_SPEED;
        
        if (kpValue) kpValue.textContent = DEFAULT_GAINS.kp.toFixed(2);
        if (kiValue) kiValue.textContent = DEFAULT_GAINS.ki.toFixed(2);
        if (kdValue) kdValue.textContent = DEFAULT_GAINS.kd.toFixed(2);
        if (speedValue) speedValue.textContent = DEFAULT_SPEED + " px/s";
        
        state.integral = 0;
        state.prevError = 0;
        state.prevDeriv = null;
        
        noiseToggle.checked = false;
        features.enableNoise = false;
        
        antiWindupToggle.checked = true;
        features.enableAntiWindup = true;
        
        dFilterToggle.checked = false;
        features.enableDFilter = false;
        
        adaptiveSpeedToggle.checked = false;
        features.enableAdaptiveSpeed = false;
      });
    }
  }
  
  setupControlPanel();
  
  // Intersection Observer for pause/resume on scroll
  function setupIntersectionObserver() {
    const section = canvas.closest("section") || canvas.closest("div[data-nav-section]");
    if (!section) {
      // Fallback: just start running
      isRunning = true;
      requestAnimationFrame(loop);
      return;
    }

    intersectionObserver = new IntersectionObserver(
      (entries) => {
        entries.forEach((entry) => {
          if (entry.isIntersecting) {
            isRunning = true;
            lastTime = performance.now();
            requestAnimationFrame(loop);
          } else {
            isRunning = false;
          }
        });
      },
      { threshold: 0.3 }
    );

    intersectionObserver.observe(section);
  }
  
  setupIntersectionObserver();
})();
