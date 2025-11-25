(function () {
  const canvas = document.getElementById("line-following-canvas");
  const robotEl = document.getElementById("line-following-robot");
  const sensorEls = Array.from(document.querySelectorAll("[data-sensor-index]"));
  const pEl = document.getElementById("pid-p");
  const iEl = document.getElementById("pid-i");
  const dEl = document.getElementById("pid-d");
  const errorEl = document.getElementById("pid-error");
  const bus = window.EventBus || { emit: () => {}, on: () => {} };

  if (!canvas || !robotEl) return;

  const ctx = canvas.getContext("2d");
  let dpr = window.devicePixelRatio || 1;

  const track = {
    center: { x: 0, y: 0 },
    radius: 140,
    width: 18,
    path: []
  };

  const sensorOffsets = [
    { angle: -0.42, distance: 34 },
    { angle: -0.36, distance: 36 },
    { angle: -0.31, distance: 38 },
    { angle: -0.26, distance: 40 },
    { angle: -0.21, distance: 42 },
    { angle: -0.16, distance: 44 },
    { angle: -0.11, distance: 46 },
    { angle: -0.06, distance: 48 },
    { angle: 0.06, distance: 48 },
    { angle: 0.11, distance: 46 },
    { angle: 0.16, distance: 44 },
    { angle: 0.21, distance: 42 },
    { angle: 0.26, distance: 40 },
    { angle: 0.31, distance: 38 },
    { angle: 0.36, distance: 36 },
    { angle: 0.42, distance: 34 }
  ];

  const weights = [-7.5, -6.5, -5.5, -4.5, -3.5, -2.5, -1.5, -0.5, 0.5, 1.5, 2.5, 3.5, 4.5, 5.5, 6.5, 7.5];
  const gains = { kp: 1.6, ki: 0.55, kd: 0.42 };

  const state = {
    x: 0,
    y: 0,
    angle: -Math.PI / 2,
    speed: 95,
    integral: 0,
    prevError: 0
  };

  let dragging = false;
  let lastTime = performance.now();

  function buildTrackPath() {
    const segments = 420;
    track.path = [];

    for (let i = 0; i < segments; i++) {
      const t = (i / segments) * Math.PI * 2;
      const wobble = 0.18 * Math.sin(t * 1.4 + 0.6) + 0.14 * Math.cos(t * 2.2) + 0.1 * Math.sin(t * 4.4 + 1.2);
      const squiggle = 12 * Math.sin(t * 3.1) + 8 * Math.cos(t * 5.3);
      const radius = track.radius * (1 + wobble) + squiggle;
      track.path.push({
        x: track.center.x + Math.cos(t) * radius,
        y: track.center.y + Math.sin(t) * radius
      });
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

  function sampleSensors() {
    const positions = getSensorPositions();
    const readings = positions.map((pos) => {
      const distToLine = distanceToTrack(pos.x, pos.y);
      const normalized = 1 - clamp(distToLine / (track.width / 2 + 10), 0, 1);
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

      ctx.setLineDash([6, 10]);
      ctx.strokeStyle = "rgba(129, 140, 248, 0.25)";
      ctx.lineWidth = 1;
      ctx.stroke();
      ctx.setLineDash([]);
    }

    sensorPositions.forEach((pos, idx) => {
      ctx.beginPath();
      ctx.fillStyle = `rgba(79,70,229,${0.25 + readings[idx] * 0.6})`;
      ctx.arc(pos.x, pos.y, 4, 0, Math.PI * 2);
      ctx.fill();
    });
  }

  function updatePID(dt, error) {
    if (dragging) {
      state.integral = 0;
    } else {
      state.integral = clamp(state.integral + error * dt, -18, 18);
    }
    const derivative = (error - state.prevError) / dt;

    const pTerm = gains.kp * error;
    const iTerm = gains.ki * state.integral;
    const dTerm = gains.kd * derivative;
    const control = pTerm + iTerm + dTerm;

    state.prevError = error;

    bus.emit("robot:pid", {
      p: pTerm,
      i: iTerm,
      d: dTerm,
      error
    });

    return control;
  }

  function updateRobot(dt, control) {
    state.angle += control * dt;
    state.x += Math.cos(state.angle) * state.speed * dt;
    state.y += Math.sin(state.angle) * state.speed * dt;

    const rect = canvas.getBoundingClientRect();
    state.x = clamp(state.x, 20, rect.width - 20);
    state.y = clamp(state.y, 20, rect.height - 20);

    robotEl.style.left = `${state.x}px`;
    robotEl.style.top = `${state.y}px`;
  }

  function computeError(readings) {
    const total = readings.reduce((sum, r) => sum + r, 0);
    if (total === 0) return state.prevError * 0.8;
    const weighted = readings.reduce((sum, r, idx) => sum + r * weights[idx], 0);
    return weighted / total;
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
    const dt = clamp((now - lastTime) / 1000, 0.001, 0.05);
    lastTime = now;

    const readings = sampleSensors();
    const positions = getSensorPositions();
    const error = computeError(readings);

    if (!dragging) {
      const control = updatePID(dt, error);
      updateRobot(dt, control);
    } else {
      updatePID(dt, error);
    }

    drawTrack(positions, readings);
    requestAnimationFrame(loop);
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
      state.speed = 95;
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
  requestAnimationFrame(loop);
})();
