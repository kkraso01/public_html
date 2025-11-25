const ragCanvas = document.getElementById("ragCanvas");
if (!ragCanvas) {
  console.warn("[RAG] canvas not found");
} else {
  const ctx = ragCanvas.getContext("2d");
  const bus = window.EventBus;

  let w = 0;
  let h = 0;
  let dpr = 1;
  let t = 0;
  let running = true;
  let speed = 1;
  let spikeLevel = 0;
  let highlightNode = null;

  const nodes = [
    { id: "q", label: "Client Query", x: 0.08, y: 0.55, hue: 210, metric: "intent" },
    { id: "emb", label: "Embed", x: 0.22, y: 0.55, hue: 250, metric: "768d" },
    { id: "vdb", label: "Vector DB", x: 0.38, y: 0.35, hue: 140, metric: "Chroma" },
    { id: "retr", label: "Retriever", x: 0.52, y: 0.55, hue: 280, metric: "top-k" },
    { id: "llm", label: "LLM Pool", x: 0.70, y: 0.55, hue: 310, metric: "router" },
    { id: "out", label: "Response", x: 0.88, y: 0.55, hue: 190, metric: "merge" }
  ];

  const edges = [
    { from: "q", to: "emb", latency: 12, score: 0.92 },
    { from: "emb", to: "vdb", latency: 18, score: 0.88 },
    { from: "emb", to: "retr", latency: 24, score: 0.81 },
    { from: "vdb", to: "retr", latency: 30, score: 0.95 },
    { from: "retr", to: "llm", latency: 36, score: 0.86 },
    { from: "llm", to: "out", latency: 22, score: 0.93 }
  ];

  const retrievals = [
    { title: "Latency-aware re-rank", score: 0.82 },
    { title: "Token filter", score: 0.78 },
    { title: "Edge LLM summary", score: 0.71 }
  ];

  function resizeRAG() {
    dpr = window.devicePixelRatio || 1;
    const rect = ragCanvas.getBoundingClientRect();
    w = rect.width;
    h = rect.height;
    ragCanvas.width = w * dpr;
    ragCanvas.height = h * dpr;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  }

  resizeRAG();
  window.addEventListener("resize", resizeRAG);

  // Viewport observer for performance
  const control = { isRunning: true };
  window.ViewportObserver.observe(ragCanvas, control, 0.1);

  function lerp(a, b, p) {
    return a + (b - a) * p;
  }

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
    ctx.fillStyle = "rgba(2,6,23,0.92)";
    ctx.fillRect(0, 0, w, h);

    ctx.strokeStyle = "rgba(30,64,175,0.25)";
    ctx.lineWidth = 1;
    const step = 20;
    ctx.beginPath();
    for (let x = 0; x < w; x += step) {
      ctx.moveTo(x, 0);
      ctx.lineTo(x, h);
    }
    for (let y = 0; y < h; y += step) {
      ctx.moveTo(0, y);
      ctx.lineTo(w, y);
    }
    ctx.stroke();
  }

  function nodePosition(id) {
    const node = nodes.find((n) => n.id === id);
    return { x: node.x * w, y: node.y * h };
  }

  function drawEdge(edge, idx) {
    const from = nodePosition(edge.from);
    const to = nodePosition(edge.to);
    const pulse = (t * 0.006 * speed + idx * 0.18) % 1;
    const midX = lerp(from.x, to.x, 0.5);
    const midY = lerp(from.y, to.y, 0.5);

    const hue = lerp(200, 320, idx / edges.length);
    const alpha = 0.5 + spikeLevel * 0.2;
    ctx.strokeStyle = `hsla(${hue}, 80%, 70%, ${alpha})`;
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(from.x, from.y);
    ctx.lineTo(to.x, to.y);
    ctx.stroke();

    const px = lerp(from.x, to.x, pulse);
    const py = lerp(from.y, to.y, pulse);
    ctx.fillStyle = `hsla(${hue}, 85%, 75%, ${0.65 + spikeLevel * 0.1})`;
    ctx.beginPath();
    ctx.arc(px, py, 5 + spikeLevel * 0.8, 0, Math.PI * 2);
    ctx.fill();

    ctx.fillStyle = "rgba(148,163,184,0.9)";
    ctx.font = "11px monospace";
    ctx.textAlign = "center";
    ctx.fillText(`${edge.latency}ms • ${Math.round(edge.score * 100)}%`, midX, midY - 8);
  }

  function drawNode(node) {
    const x = node.x * w;
    const y = node.y * h;

    ctx.fillStyle = highlightNode === node.id ? "rgba(52,211,153,0.95)" : `hsla(${node.hue}, 95%, 75%, 0.95)`;
    ctx.strokeStyle = "rgba(226,232,240,0.4)";
    ctx.lineWidth = 2;
    ctx.beginPath();
    drawRoundedRect(x - 50, y - 22, 100, 44, 10);
    ctx.fill();
    ctx.stroke();

    ctx.fillStyle = "rgba(15,23,42,0.9)";
    ctx.font = "12px monospace";
    ctx.textAlign = "center";
    ctx.fillText(node.label, x, y - 2);
    ctx.fillStyle = "rgba(148,163,184,0.9)";
    ctx.font = "11px monospace";
    ctx.fillText(node.metric, x, y + 12);
  }

  function drawRetrievals() {
    ctx.save();
    ctx.translate(w - 180, 16);
    ctx.fillStyle = "rgba(15,23,42,0.8)";
    ctx.strokeStyle = "rgba(129,140,248,0.5)";
    ctx.lineWidth = 1.2;
    ctx.beginPath();
    drawRoundedRect(0, 0, 170, 90, 10);
    ctx.fill();
    ctx.stroke();

    ctx.fillStyle = "rgba(148,163,184,0.9)";
    ctx.font = "11px monospace";
    ctx.fillText("Retrieval log", 14, 18);
    retrievals.forEach((item, i) => {
      const y = 36 + i * 18;
      ctx.fillStyle = "rgba(226,232,240,0.92)";
      ctx.fillText(item.title, 14, y);
      ctx.fillStyle = "rgba(52,211,153,0.9)";
      ctx.fillText((item.score * 100).toFixed(0) + "%", 150, y);
    });
    ctx.restore();
  }

  function draw() {
    if (control.isRunning) {
      if (running) {
        t += 16;
        spikeLevel = Math.max(0, spikeLevel - 0.01);
      }

      ctx.clearRect(0, 0, w, h);
      drawBackground();
      edges.forEach(drawEdge);
      nodes.forEach(drawNode);
      drawRetrievals();
    }

    requestAnimationFrame(draw);
  }

  draw();

  function handleControl(payload) {
    if (!payload) return;
    if (payload.action === "toggle") {
      running = !payload.value;
    }
    if (payload.action === "speed" && typeof payload.value === "number") {
      speed = payload.value;
    }
    if (payload.action === "spike") {
      spikeLevel = Math.min(3, spikeLevel + (payload.value || 1));
      if (bus) bus.emit("telemetry:spike", { source: "rag", intensity: spikeLevel });
    }
  }

  function nearestNode(evt) {
    const rect = ragCanvas.getBoundingClientRect();
    const x = (evt.clientX - rect.left);
    const y = (evt.clientY - rect.top);
    let best = null;
    let bestDist = Infinity;
    nodes.forEach((n) => {
      const px = n.x * w;
      const py = n.y * h;
      const dist = Math.hypot(px - x, py - y);
      if (dist < bestDist) {
        best = n;
        bestDist = dist;
      }
    });
    return bestDist < 80 ? best : null;
  }

  ragCanvas.addEventListener("click", (evt) => {
    const n = nearestNode(evt);
    if (!n) return;
    highlightNode = n.id;
    spikeLevel = Math.min(3, spikeLevel + 0.6);
    if (bus) bus.emit("telemetry:spike", { source: "rag", node: n.id, metric: "latency" });
  });

  if (bus) {
    bus.on("control:rag", handleControl);
    bus.on("telemetry:spike", ({ source, intensity }) => {
      if (source === "rag") return;
      spikeLevel = Math.min(3, spikeLevel + (intensity || 0.4));
    });
  }
}
