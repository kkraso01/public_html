// 2D GPT-4 System Visualizer Engine with trails and control bus
const canvas2d = document.getElementById("viz2d");
if (!canvas2d) {
  console.warn("Canvas viz2d not found");
} else {
  const ctx2d = canvas2d.getContext("2d");
  const bus = window.EventBus;

  let w, h, dpr;
  let t = 0;
  let running = true;
  let speed = 1;
  let burst = 0;
  let highlight = 0;

  function resize() {
    dpr = window.devicePixelRatio || 1;
    const rect = canvas2d.getBoundingClientRect();
    w = rect.width;
    h = rect.height;

    canvas2d.width = w * dpr;
    canvas2d.height = h * dpr;
    ctx2d.setTransform(dpr, 0, 0, dpr, 0, 0);
  }

  resize();
  window.addEventListener("resize", resize);

  const core = { x: 0.5, y: 0.45, r: 28 };
  const vectorDB = { x: 0.16, y: 0.78, r: 18, hue: 110 };

  const llmNodes = Array.from({ length: 5 }, (_, i) => ({
    orbit: 0.18 + i * 0.08,
    size: 11 + Math.random() * 4,
    speed: 0.004 + Math.random() * 0.004,
    hue: 200 + i * 32,
    active: Math.random() > 0.4
  }));

  const tokenParticles = [];
  for (let i = 0; i < 120; i++) {
    tokenParticles.push({
      a: Math.random() * Math.PI * 2,
      r: Math.random() * 0.38 + 0.1,
      speed: 0.0025 + Math.random() * 0.004,
      hue: 190 + Math.random() * 120,
      size: 1 + Math.random() * 1.8,
      history: []
    });
  }

  const channels = [
    { label: "retrieval", color: "#22d3ee" },
    { label: "routing", color: "#a78bfa" },
    { label: "generation", color: "#34d399" }
  ];

  function drawRoundedRect(x, y, width, height, radius) {
    if (ctx2d.roundRect) {
      ctx2d.roundRect(x, y, width, height, radius);
      return;
    }

    const r = Math.min(radius, width / 2, height / 2);
    ctx2d.beginPath();
    ctx2d.moveTo(x + r, y);
    ctx2d.lineTo(x + width - r, y);
    ctx2d.quadraticCurveTo(x + width, y, x + width, y + r);
    ctx2d.lineTo(x + width, y + height - r);
    ctx2d.quadraticCurveTo(x + width, y + height, x + width - r, y + height);
    ctx2d.lineTo(x + r, y + height);
    ctx2d.quadraticCurveTo(x, y + height, x, y + height - r);
    ctx2d.lineTo(x, y + r);
    ctx2d.quadraticCurveTo(x, y, x + r, y);
  }

  function drawCore() {
    const x = w * core.x;
    const y = h * core.y;

    ctx2d.fillStyle = "rgba(226,232,240,0.92)";
    ctx2d.beginPath();
    ctx2d.arc(x, y, core.r, 0, Math.PI * 2);
    ctx2d.fill();

    ctx2d.strokeStyle = `rgba(100, 200, 255, ${0.3 + 0.3 * Math.sin(t * 0.01)})`;
    ctx2d.lineWidth = 2;
    ctx2d.beginPath();
    ctx2d.arc(x, y, core.r * 1.4, 0, Math.PI * 2);
    ctx2d.stroke();
  }

  function drawLLMNodes() {
    const cx = w * core.x;
    const cy = h * core.y;

    llmNodes.forEach((n, i) => {
      const angle = t * n.speed * speed + i * (Math.PI * 2 / llmNodes.length);
      const x = cx + Math.cos(angle) * (w * n.orbit);
      const y = cy + Math.sin(angle) * (h * n.orbit);

      ctx2d.fillStyle = `hsla(${n.hue}, 85%, 70%, 0.9)`;
      ctx2d.beginPath();
      ctx2d.arc(x, y, n.size, 0, Math.PI * 2);
      ctx2d.fill();

      if (i % 2 === 0) {
        ctx2d.strokeStyle = `hsla(${n.hue}, 80%, 70%, 0.5)`;
        ctx2d.lineWidth = 1.5;
        ctx2d.beginPath();
        ctx2d.moveTo(cx, cy);
        ctx2d.lineTo(x, y);
        ctx2d.stroke();
      }
    });
  }

  function drawVectorDB() {
    const x = w * vectorDB.x;
    const y = h * vectorDB.y;
    const pulse = Math.sin(t * 0.01 * speed) * 0.3 + 0.7 + burst * 0.1;

    ctx2d.strokeStyle = `hsla(${vectorDB.hue}, 80%, 60%, ${pulse * 0.6})`;
    ctx2d.lineWidth = 1;
    const gridSize = vectorDB.r / 2;
    for (let i = -1; i <= 1; i++) {
      for (let j = -1; j <= 1; j++) {
        ctx2d.beginPath();
        ctx2d.rect(
          x + i * gridSize - gridSize / 2,
          y + j * gridSize - gridSize / 2,
          gridSize,
          gridSize
        );
        ctx2d.stroke();
      }
    }

    ctx2d.fillStyle = `hsla(${vectorDB.hue}, 90%, 70%, 0.9)`;
    ctx2d.beginPath();
    ctx2d.arc(x, y, vectorDB.r * 0.6, 0, Math.PI * 2);
    ctx2d.fill();

    ctx2d.fillStyle = "rgba(148,163,184,0.9)";
    ctx2d.font = "10px monospace";
    ctx2d.textAlign = "center";
    ctx2d.fillText("vector ops", x, y + 30);
  }

  function drawTrails() {
    const cx = w * core.x;
    const cy = h * core.y;

    tokenParticles.forEach((p, idx) => {
      if (running) p.a += p.speed * speed * (1 + burst * 0.6);
      const radius = w * p.r * (1 + burst * 0.2);
      const x = cx + Math.cos(p.a) * radius;
      const y = cy + Math.sin(p.a) * radius;

      p.history.push({ x, y });
      if (p.history.length > 8) p.history.shift();

      const channel = channels[idx % channels.length];
      ctx2d.strokeStyle = `${channel.color}66`;
      ctx2d.lineWidth = 1.2;
      ctx2d.beginPath();
      p.history.forEach((pt, i) => {
        if (i === 0) ctx2d.moveTo(pt.x, pt.y);
        else ctx2d.lineTo(pt.x, pt.y);
      });
      ctx2d.stroke();

      ctx2d.fillStyle = channel.color;
      ctx2d.beginPath();
      ctx2d.arc(x, y, p.size + burst * 0.6, 0, Math.PI * 2);
      ctx2d.fill();
    });
  }

  function drawHUD() {
    ctx2d.save();
    ctx2d.translate(w - 170, 12);
    ctx2d.fillStyle = "rgba(15,23,42,0.85)";
    ctx2d.strokeStyle = "rgba(99,102,241,0.4)";
    ctx2d.lineWidth = 1;
    ctx2d.beginPath();
    drawRoundedRect(0, 0, 160, 70, 10);
    ctx2d.fill();
    ctx2d.stroke();

    ctx2d.fillStyle = "rgba(148,163,184,0.9)";
    ctx2d.font = "11px monospace";
    ctx2d.fillText("Vector ops", 12, 18);
    ctx2d.fillText(`speed x${speed.toFixed(2)}`, 12, 34);
    ctx2d.fillStyle = "rgba(94,234,212,0.9)";
    ctx2d.fillText(`burst ${burst.toFixed(2)}`, 12, 50);
    ctx2d.fillStyle = "rgba(244,114,182,0.9)";
    ctx2d.fillText(`signal ${highlight.toFixed(2)}`, 12, 66);
    ctx2d.restore();
  }

  function animate() {
    if (running) {
      t += 1;
      burst = Math.max(0, burst - 0.01);
      highlight = Math.max(0, highlight - 0.008);
    }

    ctx2d.clearRect(0, 0, w, h);
    drawTrails();
    drawVectorDB();
    drawLLMNodes();
    drawCore();
    drawHUD();

    requestAnimationFrame(animate);
  }

  canvas2d.addEventListener("click", () => {
    burst = Math.min(2.5, burst + 0.8);
    highlight = 1.2;
    if (bus) bus.emit("telemetry:spike", { source: "vector2d", intensity: burst });
  });

  function handleControl(payload) {
    if (!payload) return;
    if (payload.action === "toggle") running = !payload.value;
    if (payload.action === "speed" && typeof payload.value === "number") speed = payload.value;
    if (payload.action === "spike") {
      burst = Math.min(3, burst + (payload.value || 1));
      highlight = 1.2;
    }
  }

  if (bus) {
    bus.on("control:vector2d", handleControl);
    bus.on("telemetry:spike", ({ source, intensity }) => {
      if (source === "vector2d") return;
      highlight = Math.min(2, highlight + (intensity || 0.5));
    });
  }

  animate();
}
