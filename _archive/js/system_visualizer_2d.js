// 2D GPT-4 System Visualizer Engine with explicit AI plumbing instead of glowing balls
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

  const layout = {
    ingress: { x: 0.12, y: 0.5 },
    router: { x: 0.32, y: 0.5 },
    vectorDB: { x: 0.16, y: 0.78 },
    cache: { x: 0.48, y: 0.25 },
    merge: { x: 0.75, y: 0.5 },
    egress: { x: 0.9, y: 0.5 }
  };

  const llmNodes = Array.from({ length: 4 }, (_, i) => ({
    x: 0.52 + (i % 2) * 0.12,
    y: 0.32 + Math.floor(i / 2) * 0.26,
    hue: 200 + i * 25,
    label: `LLM-${i + 1}`,
    load: 0.3 + Math.random() * 0.6
  }));

  const colors = {
    bg: "#020617",
    grid: "rgba(39, 61, 117, 0.3)",
    outline: "rgba(129,140,248,0.65)",
    accent: "#22d3ee"
  };

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

  function drawBackground() {
    ctx2d.fillStyle = colors.bg;
    ctx2d.fillRect(0, 0, w, h);
    ctx2d.strokeStyle = colors.grid;
    ctx2d.lineWidth = 1;
    const spacing = 28;
    ctx2d.beginPath();
    for (let x = 0; x < w; x += spacing) {
      ctx2d.moveTo(x, 0);
      ctx2d.lineTo(x, h);
    }
    for (let y = 0; y < h; y += spacing) {
      ctx2d.moveTo(0, y);
      ctx2d.lineTo(w, y);
    }
    ctx2d.stroke();
  }

  function drawModule(position, label, color, sublabel) {
    const widthBox = 110;
    const heightBox = 70;
    const x = position.x * w - widthBox / 2;
    const y = position.y * h - heightBox / 2;
    drawRoundedRect(x, y, widthBox, heightBox, 12);
    ctx2d.fillStyle = "rgba(15,23,42,0.9)";
    ctx2d.fill();
    ctx2d.strokeStyle = color;
    ctx2d.lineWidth = 1.8;
    ctx2d.stroke();

    ctx2d.fillStyle = "rgba(226,232,240,0.92)";
    ctx2d.font = "12px monospace";
    ctx2d.textAlign = "center";
    ctx2d.fillText(label, position.x * w, position.y * h - 4);

    if (sublabel) {
      ctx2d.fillStyle = "rgba(148,163,184,0.9)";
      ctx2d.font = "10px monospace";
      ctx2d.fillText(sublabel, position.x * w, position.y * h + 12);
    }
  }

  function drawLLMPods() {
    llmNodes.forEach((node) => {
      const x = node.x * w;
      const y = node.y * h;
      const boxW = 120;
      const boxH = 64;
      drawRoundedRect(x - boxW / 2, y - boxH / 2, boxW, boxH, 10);
      ctx2d.fillStyle = "rgba(17,24,39,0.95)";
      ctx2d.fill();
      ctx2d.strokeStyle = `hsla(${node.hue}, 75%, 65%, 0.9)`;
      ctx2d.lineWidth = 1.6;
      ctx2d.stroke();

      ctx2d.fillStyle = "rgba(226,232,240,0.9)";
      ctx2d.font = "11px monospace";
      ctx2d.textAlign = "center";
      ctx2d.fillText(node.label, x, y - 6);
      ctx2d.fillStyle = "rgba(148,163,184,0.9)";
      ctx2d.font = "10px monospace";
      ctx2d.fillText("context window", x, y + 10);

      ctx2d.fillStyle = `hsla(${node.hue}, 85%, 65%, 0.5)`;
      ctx2d.fillRect(x - 40, y + 16, 80 * node.load, 6);
    });
  }

  const routes = [
    { from: "ingress", to: "router", color: "#22d3ee", label: "client query" },
    { from: "router", to: "vectorDB", color: "#34d399", label: "vector lookup" },
    { from: "router", to: "cache", color: "#f472b6", label: "prompt cache" },
    ...llmNodes.map((_, i) => ({ from: "router", to: `llm${i}`, color: "#a78bfa", label: "dispatch" })),
    ...llmNodes.map((_, i) => ({ from: `llm${i}`, to: "merge", color: "#38bdf8", label: "tokens" })),
    { from: "merge", to: "egress", color: "#22c55e", label: "response" }
  ];

  const packets = routes.map((route) => ({
    route,
    progress: Math.random(),
    speed: 0.003 + Math.random() * 0.003,
    hueShift: Math.random() * 40
  }));

  function pointFor(key) {
    if (key.startsWith("llm")) {
      const idx = Number(key.replace("llm", ""));
      const node = llmNodes[idx];
      return { x: node.x * w, y: node.y * h };
    }
    const pos = layout[key];
    return { x: pos.x * w, y: pos.y * h };
  }

  function drawRouteLines() {
    routes.forEach((r) => {
      const from = pointFor(r.from);
      const to = pointFor(r.to);
      const alpha = 0.2 + 0.6 * Math.abs(Math.sin(t * 0.008));
      ctx2d.strokeStyle = `${r.color}${Math.floor(alpha * 255)
        .toString(16)
        .padStart(2, "0")}`;
      ctx2d.lineWidth = 2;
      ctx2d.beginPath();
      ctx2d.moveTo(from.x, from.y);
      ctx2d.lineTo(to.x, to.y);
      ctx2d.stroke();
    });
  }

  function drawPackets() {
    packets.forEach((p) => {
      if (running) p.progress = (p.progress + p.speed * speed) % 1;
      const from = pointFor(p.route.from);
      const to = pointFor(p.route.to);
      const x = from.x + (to.x - from.x) * p.progress;
      const y = from.y + (to.y - from.y) * p.progress;
      const size = 9 + burst * 2;

      drawRoundedRect(x - size / 2, y - size / 2, size, size, 4);
      ctx2d.fillStyle = `${p.route.color}`;
      ctx2d.fill();
      ctx2d.strokeStyle = "rgba(15,23,42,0.8)";
      ctx2d.stroke();
    });
  }

  function drawHUD() {
    ctx2d.save();
    ctx2d.translate(w - 200, 14);
    ctx2d.fillStyle = "rgba(15,23,42,0.88)";
    ctx2d.strokeStyle = colors.outline;
    ctx2d.lineWidth = 1;
    ctx2d.beginPath();
    drawRoundedRect(0, 0, 180, 90, 12);
    ctx2d.fill();
    ctx2d.stroke();

    ctx2d.fillStyle = "rgba(226,232,240,0.95)";
    ctx2d.font = "11px monospace";
    ctx2d.fillText("LLM orchestration", 12, 20);
    ctx2d.fillStyle = "rgba(148,163,184,0.9)";
    ctx2d.fillText(`speed x${speed.toFixed(2)}`, 12, 36);
    ctx2d.fillText(`spike ${burst.toFixed(2)}`, 12, 52);
    ctx2d.fillText(`routes ${routes.length}`, 12, 68);
    ctx2d.restore();
  }

  function annotate() {
    ctx2d.fillStyle = "rgba(148,163,184,0.8)";
    ctx2d.font = "10px monospace";
    ctx2d.textAlign = "center";
    ctx2d.fillText("router", layout.router.x * w, layout.router.y * h + 32);
    ctx2d.fillText("vector DB", layout.vectorDB.x * w, layout.vectorDB.y * h + 32);
    ctx2d.fillText("fusion / merge", layout.merge.x * w, layout.merge.y * h + 32);
  }

  function animate() {
    if (running) {
      t += 1;
      burst = Math.max(0, burst - 0.01);
      highlight = Math.max(0, highlight - 0.008);
    }

    drawBackground();
    drawRouteLines();
    drawModule(layout.ingress, "Client", colors.outline, "prompt + files");
    drawModule(layout.router, "LLM Router", "#22d3ee", "policy: quality/cost");
    drawModule(layout.vectorDB, "Vector DB", "#34d399", "retrieval hop");
    drawModule(layout.cache, "Prompt Cache", "#f472b6", "hot paths");
    drawModule(layout.merge, "Merger", "#38bdf8", "rank + stitch");
    drawModule(layout.egress, "Response", "#22c55e", "stream tokens");
    drawLLMPods();
    drawPackets();
    annotate();
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
