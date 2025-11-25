// 3D-Style Neural Galaxy Visualizer (Canvas-based 2.5D)
function init3DVisualizer() {
  const canvas3d = document.getElementById("viz3d");
  if (!canvas3d) {
    console.warn("Canvas viz3d not found");
    return;
  }

  const ctx = canvas3d.getContext("2d");
  const bus = window.EventBus;
  let w, h, dpr;
  let t = 0;
  let running = true;
  let speed = 1;
  let highlight = 0;

  function resize() {
    dpr = window.devicePixelRatio || 1;
    const rect = canvas3d.getBoundingClientRect();
    w = rect.width;
    h = rect.height;
    canvas3d.width = w * dpr;
    canvas3d.height = h * dpr;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  }

  resize();
  window.addEventListener("resize", resize);

  const clusters = [
    { label: "retrieval", hue: 200 },
    { label: "generation", hue: 320 },
    { label: "routing", hue: 160 }
  ];

  const particles = Array.from({ length: 220 }, (_, i) => {
    const arm = i % clusters.length;
    const angle = (i / 120) * Math.PI * 4;
    const radius = 30 + Math.random() * 180;
    return {
      arm,
      baseAngle: angle,
      radius,
      speed: 0.0008 + Math.random() * 0.0012,
      z: Math.random(),
      size: 1 + Math.random() * 2,
      jitter: Math.random() * Math.PI * 2
    };
  });

  const topics = [
    { label: "Embeddings", x: 0.26, y: 0.32 },
    { label: "Latency bands", x: 0.52, y: 0.22 },
    { label: "Clustered intents", x: 0.68, y: 0.46 }
  ];

  function drawRoundedRect(x, y, width, height, radius) {
    if (ctx.roundRect) {
      ctx.roundRect(x, y, width, height, radius);
      return;
    }

    const r = Math.min(radius, width / 2, height / 2);
    ctx.beginPath();
    ctx.moveTo(x + r, y);
    ctx.lineTo(x + width - r, y);
    ctx.quadraticCurveTo(x + width, y, x + width, y + r);
    ctx.lineTo(x + width, y + height - r);
    ctx.quadraticCurveTo(x + width, y + height, x + width - r, y + height);
    ctx.lineTo(x + r, y + height);
    ctx.quadraticCurveTo(x, y + height, x, y + height - r);
    ctx.lineTo(x, y + r);
    ctx.quadraticCurveTo(x, y, x + r, y);
  }

  function drawBackground() {
    const grad = ctx.createRadialGradient(w / 2, h / 2, 0, w / 2, h / 2, Math.max(w, h) / 1.2);
    grad.addColorStop(0, "rgba(15,23,42,1)");
    grad.addColorStop(1, "rgba(2,6,23,0.9)");
    ctx.fillStyle = grad;
    ctx.fillRect(0, 0, w, h);
  }

  function drawParticles() {
    const cx = w / 2;
    const cy = h / 2;

    particles.forEach((p, idx) => {
      if (running) p.baseAngle += p.speed * speed;
      const spiral = p.baseAngle + Math.sin(t * 0.002 + p.jitter) * 0.02;
      const r = p.radius * (1 + 0.1 * Math.sin(t * 0.001 + idx)) + highlight * 4;
      const x = cx + Math.cos(spiral) * r;
      const y = cy + Math.sin(spiral) * r * 0.6 + Math.sin(p.baseAngle) * 6;

      const cluster = clusters[p.arm];
      const alpha = 0.55 + p.z * 0.25 + highlight * 0.1;
      ctx.fillStyle = `hsla(${cluster.hue}, 70%, 70%, ${alpha})`;
      ctx.fillRect(x - 2, y - 2, 4, 4);
    });
  }

  function drawRibbons() {
    const cx = w / 2;
    const cy = h / 2;

    clusters.forEach((cluster, idx) => {
      ctx.strokeStyle = `hsla(${cluster.hue}, 90%, 70%, 0.4)`;
      ctx.lineWidth = 2;
      ctx.beginPath();
      for (let angle = 0; angle < Math.PI * 2; angle += 0.12) {
        const r = 40 + idx * 30 + Math.sin(angle * 2 + t * 0.002 * speed) * 10;
        const x = cx + Math.cos(angle) * r;
        const y = cy + Math.sin(angle) * r * 0.7;
        if (angle === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
      }
      ctx.closePath();
      ctx.stroke();
    });
  }

  function drawHUD() {
    ctx.save();
    ctx.translate(16, 16);
    ctx.fillStyle = "rgba(15,23,42,0.82)";
    ctx.strokeStyle = "rgba(129,140,248,0.5)";
    ctx.lineWidth = 1.2;
    ctx.beginPath();
    drawRoundedRect(0, 0, 180, 90, 12);
    ctx.fill();
    ctx.stroke();

    ctx.fillStyle = "rgba(226,232,240,0.92)";
    ctx.font = "11px monospace";
    ctx.fillText("Neural Galaxy", 12, 20);
    ctx.fillStyle = "rgba(148,163,184,0.9)";
    ctx.fillText(`speed x${speed.toFixed(2)}`, 12, 36);
    ctx.fillText(`highlight ${highlight.toFixed(2)}`, 12, 52);
    ctx.fillText(`particles ${particles.length}`, 12, 68);
    ctx.restore();
  }

  function drawTopics() {
    ctx.fillStyle = "rgba(226,232,240,0.85)";
    ctx.font = "11px monospace";
    topics.forEach((tpc) => {
      const x = tpc.x * w;
      const y = tpc.y * h;
      ctx.fillText(tpc.label, x, y);
      ctx.strokeStyle = "rgba(94,234,212,0.5)";
      ctx.beginPath();
      ctx.arc(x, y - 6, 6 + highlight * 0.5, 0, Math.PI * 2);
      ctx.stroke();
    });
  }

  function animate() {
    if (running) {
      t += 16;
      highlight = Math.max(0, highlight - 0.01);
    }

    ctx.clearRect(0, 0, w, h);
    drawBackground();
    drawParticles();
    drawRibbons();
    drawTopics();
    drawHUD();

    requestAnimationFrame(animate);
  }

  function handleControl(payload) {
    if (!payload) return;
    if (payload.action === "toggle") running = !payload.value;
    if (payload.action === "speed" && typeof payload.value === "number") speed = payload.value;
    if (payload.action === "spike") {
      highlight = Math.min(3, highlight + (payload.value || 1));
      if (bus) bus.emit("telemetry:spike", { source: "galaxy3d", intensity: highlight });
    }
  }

  canvas3d.addEventListener("click", () => {
    highlight = Math.min(3, highlight + 0.8);
    if (bus) bus.emit("telemetry:spike", { source: "galaxy3d", intensity: highlight });
  });

  if (bus) {
    bus.on("control:galaxy3d", handleControl);
    bus.on("telemetry:spike", ({ source, intensity }) => {
      if (source === "galaxy3d") return;
      highlight = Math.min(3, highlight + (intensity || 0.4));
    });
  }

  animate();
}

// Initialize when DOM is ready
if (document.readyState === "loading") {
  document.addEventListener("DOMContentLoaded", init3DVisualizer);
} else {
  init3DVisualizer();
}
