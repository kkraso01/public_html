const ragCanvas = document.getElementById("ragCanvas");
if (!ragCanvas) {
  console.warn("[RAG] canvas not found");
} else {
  const ctx = ragCanvas.getContext("2d");
  const bus = window.EventBus;
  const theme = window.UI_THEME;

  let w = 0;
  let h = 0;
  let dpr = 1;
  let t = 0;
  let running = true;
  let speed = 1;
  let spikeLevel = 0;
  let highlightNode = null;
  let lastMouse = null;

  const nodes = [
    { id: "client", label: "CLIENT", sub: "Query", x: 0.08, y: 0.55 },
    { id: "router", label: "ROUTER", sub: "Policy", x: 0.22, y: 0.55 },
    { id: "vector", label: "VECTOR DB", sub: "Chroma", x: 0.40, y: 0.35 },
    { id: "retriever", label: "RETRIEVER", sub: "Top-K", x: 0.40, y: 0.72 },
    { id: "context", label: "CONTEXT", sub: "Chunks", x: 0.58, y: 0.55 },
    { id: "llm", label: "LLM", sub: "Pool", x: 0.74, y: 0.55 },
    { id: "response", label: "RESPONSE", sub: "Stream", x: 0.90, y: 0.55 }
  ];

  const edges = [
    { from: "client", to: "router", metric: "12ms" },
    { from: "router", to: "vector", metric: "lookup" },
    { from: "router", to: "retriever", metric: "rank" },
    { from: "vector", to: "context", metric: "hits" },
    { from: "retriever", to: "context", metric: "top-k" },
    { from: "context", to: "llm", metric: "prompt" },
    { from: "llm", to: "response", metric: "decode" }
  ];

  const retrievals = [
    { title: "Latency-aware re-rank", score: 0.82 },
    { title: "Token filter", score: 0.78 },
    { title: "Edge LLM summary", score: 0.71 }
  ];

  function resizeRAG() {
    if (theme) {
      const metrics = theme.setDPR(ragCanvas, ctx);
      w = metrics.width;
      h = metrics.height;
      dpr = metrics.dpr;
      return;
    }
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
  window.addEventListener("themechange", resizeRAG);

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

  function palette() {
    if (theme) return theme.palette();
    const styles = getComputedStyle(document.documentElement);
    return {
      surface: styles.getPropertyValue("--surface").trim(),
      surfaceElevated: styles.getPropertyValue("--surface-elevated").trim(),
      text: styles.getPropertyValue("--text").trim(),
      textStrong: styles.getPropertyValue("--text-strong").trim(),
      muted: styles.getPropertyValue("--muted").trim(),
      border: styles.getPropertyValue("--border").trim(),
      accent: styles.getPropertyValue("--accent").trim(),
    };
  }

  function drawBackground(colors) {
    ctx.fillStyle = colors.surface;
    ctx.fillRect(0, 0, w, h);

    if (theme) {
      theme.drawGrid(ctx, 24, colors.border, theme.isDark() ? 0.12 : 0.08, w, h);
    }
  }

  function nodePosition(id) {
    const node = nodes.find((n) => n.id === id);
    return { x: node.x * w, y: node.y * h };
  }

  function drawEdge(edge, idx, colors) {
    const from = nodePosition(edge.from);
    const to = nodePosition(edge.to);
    const pulse = (t * 0.003 * speed + idx * 0.18) % 1;
    const stepped = Math.floor(pulse * 6) / 6;
    const midX = lerp(from.x, to.x, 0.5);
    const midY = lerp(from.y, to.y, 0.5);

    ctx.strokeStyle = theme ? theme.rgba(colors.border, 0.6 + spikeLevel * 0.05) : "rgba(148,163,184,0.6)";
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(from.x, from.y);
    ctx.lineTo(to.x, to.y);
    ctx.stroke();

    const px = lerp(from.x, to.x, stepped);
    const py = lerp(from.y, to.y, stepped);
    ctx.fillStyle = theme ? theme.rgba(colors.muted, 0.8) : "rgba(148,163,184,0.8)";
    ctx.fillRect(px - 3, py - 3, 6, 6);

    ctx.fillStyle = theme ? theme.rgba(colors.muted, 0.9) : "rgba(148,163,184,0.9)";
    ctx.font = "600 10px ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace";
    ctx.textAlign = "center";
    ctx.fillText(edge.metric, midX, midY - 8);
  }

  function drawNode(node, colors) {
    const x = node.x * w;
    const y = node.y * h;
    const isActive = highlightNode === node.id;

    ctx.fillStyle = colors.surfaceElevated;
    ctx.strokeStyle = theme
      ? theme.rgba(isActive ? colors.accent : colors.border, isActive ? 0.9 : 0.7)
      : "rgba(226,232,240,0.4)";
    ctx.lineWidth = isActive ? 1.5 : 1;
    ctx.beginPath();
    drawRoundedRect(x - 54, y - 22, 108, 44, 2);
    ctx.fill();
    ctx.stroke();

    ctx.fillStyle = theme ? colors.textStrong : "rgba(15,23,42,0.9)";
    ctx.font = "600 10px ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace";
    ctx.textAlign = "center";
    ctx.fillText(node.label, x, y - 2);
    ctx.fillStyle = theme ? colors.muted : "rgba(148,163,184,0.9)";
    ctx.font = "10px ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace";
    ctx.fillText(node.sub, x, y + 12);
  }

  function drawRetrievals(colors) {
    ctx.save();
    ctx.translate(w - 190, 16);
    ctx.fillStyle = colors.surfaceElevated;
    ctx.strokeStyle = theme ? theme.rgba(colors.border, 0.8) : "rgba(129,140,248,0.5)";
    ctx.lineWidth = 1;
    ctx.beginPath();
    drawRoundedRect(0, 0, 180, 90, 2);
    ctx.fill();
    ctx.stroke();

    ctx.fillStyle = theme ? colors.muted : "rgba(148,163,184,0.9)";
    ctx.font = "600 10px ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace";
    ctx.fillText("Retrieval log", 14, 18);
    retrievals.forEach((item, i) => {
      const y = 36 + i * 18;
      ctx.fillStyle = theme ? colors.text : "rgba(226,232,240,0.92)";
      ctx.fillText(item.title, 14, y);
      ctx.fillStyle = theme ? colors.accent : "rgba(52,211,153,0.9)";
      ctx.fillText((item.score * 100).toFixed(0) + "%", 160, y);
    });
    ctx.restore();
  }

  function drawTooltip(node, colors, mouse) {
    if (!node || !mouse) return;
    const padding = 8;
    const text = `${node.label} • ${node.sub}`;
    ctx.save();
    ctx.font = "600 10px ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace";
    const metrics = ctx.measureText(text);
    const width = metrics.width + padding * 2;
    const height = 24;
    const x = Math.min(mouse.x + 12, w - width - 8);
    const y = Math.min(mouse.y + 12, h - height - 8);
    ctx.fillStyle = colors.surfaceElevated;
    ctx.strokeStyle = theme ? theme.rgba(colors.border, 0.8) : "rgba(148,163,184,0.6)";
    ctx.lineWidth = 1;
    ctx.beginPath();
    drawRoundedRect(x, y, width, height, 2);
    ctx.fill();
    ctx.stroke();
    ctx.fillStyle = theme ? colors.textStrong : "rgba(226,232,240,0.9)";
    ctx.textAlign = "left";
    ctx.textBaseline = "middle";
    ctx.fillText(text, x + padding, y + height / 2);
    ctx.restore();
  }

  function draw() {
    if (control.isRunning) {
      if (running) {
        t += 16;
        spikeLevel = Math.max(0, spikeLevel - 0.01);
      }

      const colors = palette();
      ctx.clearRect(0, 0, w, h);
      drawBackground(colors);
      edges.forEach((edge, idx) => drawEdge(edge, idx, colors));
      nodes.forEach((node) => drawNode(node, colors));
      drawRetrievals(colors);
      drawTooltip(highlightNode ? nodes.find((n) => n.id === highlightNode) : null, colors, lastMouse);
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
    lastMouse = { x, y };
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

  ragCanvas.addEventListener("mousemove", (evt) => {
    const n = nearestNode(evt);
    highlightNode = n ? n.id : null;
  });

  ragCanvas.addEventListener("mouseleave", () => {
    highlightNode = null;
    lastMouse = null;
  });

  if (bus) {
    bus.on("control:rag", handleControl);
    bus.on("telemetry:spike", ({ source, intensity }) => {
      if (source === "rag") return;
      spikeLevel = Math.min(3, spikeLevel + (intensity || 0.4));
    });
  }
}
