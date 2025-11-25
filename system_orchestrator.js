// system_orchestrator.js
// Multi-LLM Orchestrator Visualization - LLM-MS Architecture Demo

(function () {
  const canvas = document.getElementById("orchestratorCanvas");
  if (!canvas) {
    console.warn("[Orchestrator] canvas not found");
    return;
  }

  const ctx = canvas.getContext("2d");
  let w, h, dpr;
  let t = 0;

  function resize() {
    dpr = window.devicePixelRatio || 1;
    const rect = canvas.getBoundingClientRect();
    w = rect.width;
    h = rect.height;
    canvas.width = w * dpr;
    canvas.height = h * dpr;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  }

  resize();
  window.addEventListener("resize", resize);

  // Layout model - normalized coordinates (0-1)
  const layout = {
    client: { x: 0.10, y: 0.50, w: 80, h: 40, label: "Client" },
    router: { x: 0.30, y: 0.50, w: 120, h: 60, label: "Router / LLM-MS" },
    vdb:    { x: 0.28, y: 0.75, w: 120, h: 40, label: "Vector DB\n(ChromaDB)" },
    scorer: { x: 0.55, y: 0.50, w: 130, h: 50, label: "Scoring / Merge" },
    output: { x: 0.80, y: 0.50, w: 90, h: 40, label: "Response" }
  };

  const llms = [
    { name: "LLM-A", subtitle: "fast / cheap",   x: 0.55, y: 0.20 },
    { name: "LLM-B", subtitle: "balanced",       x: 0.70, y: 0.30 },
    { name: "LLM-C", subtitle: "slow / strong",  x: 0.70, y: 0.70 }
  ];

  // Routing modes that cycle
  const routes = [
    { mode: "cost-aware", chosen: 0 }, // LLM-A
    { mode: "balanced",   chosen: 1 }, // LLM-B
    { mode: "quality",    chosen: 2 }  // LLM-C
  ];

  // Metrics state
  const metrics = {
    tokensPerSec: 0,
    latencyMs: 0,
    cost: 0
  };

  function updateMetrics(routeIndex) {
    if (routeIndex === 0) {        // cost-aware / LLM-A
      metrics.tokensPerSec = 180;
      metrics.latencyMs = 40;
      metrics.cost = 1.0;
    } else if (routeIndex === 1) { // balanced / LLM-B
      metrics.tokensPerSec = 120;
      metrics.latencyMs = 60;
      metrics.cost = 1.4;
    } else {                       // quality / LLM-C
      metrics.tokensPerSec = 80;
      metrics.latencyMs = 90;
      metrics.cost = 2.0;
    }
  }

  function currentRouteWithMetrics() {
    const idx = Math.floor(t / 300) % routes.length;
    updateMetrics(idx);
    return routes[idx];
  }

  // Draw utility: rounded box
  function box(pos, options = {}) {
    const { x, y, w: bw, h: bh, label } = pos;
    const px = x * w - bw / 2;
    const py = y * h - bh / 2;

    ctx.save();
    ctx.beginPath();
    const r = 10;
    ctx.moveTo(px + r, py);
    ctx.lineTo(px + bw - r, py);
    ctx.quadraticCurveTo(px + bw, py, px + bw, py + r);
    ctx.lineTo(px + bw, py + bh - r);
    ctx.quadraticCurveTo(px + bw, py + bh, px + bw - r, py + bh);
    ctx.lineTo(px + r, py + bh);
    ctx.quadraticCurveTo(px, py + bh, px, py + bh - r);
    ctx.lineTo(px, py + r);
    ctx.quadraticCurveTo(px, py, px + r, py);
    ctx.closePath();

    ctx.fillStyle = options.fill || "rgba(15,23,42,0.9)";
    ctx.strokeStyle = options.stroke || "rgba(148,163,184,0.8)";
    ctx.lineWidth = options.lineWidth || 1.4;
    ctx.fill();
    ctx.stroke();

    if (label) {
      ctx.fillStyle = options.textColor || "rgba(226,232,240,0.96)";
      ctx.font = "11px monospace";
      ctx.textAlign = "center";
      ctx.textBaseline = "middle";

      const lines = label.split("\n");
      lines.forEach((line, i) => {
        ctx.fillText(line, x * w, y * h - 6 + i * 12);
      });
    }

    ctx.restore();
  }

  // Draw animated arrow with dashes
  function arrow(from, to, color, thickness = 2, dashed = false, pulse = 0) {
    const fx = from.x * w;
    const fy = from.y * h;
    const tx = to.x * w;
    const ty = to.y * h;

    ctx.save();
    ctx.strokeStyle = color;
    ctx.lineWidth = thickness;
    if (dashed) {
      ctx.setLineDash([8, 6]);
      ctx.lineDashOffset = -pulse * 20;
    }

    ctx.beginPath();
    ctx.moveTo(fx, fy);
    ctx.lineTo(tx, ty);
    ctx.stroke();
    ctx.setLineDash([]);

    // Arrow head
    const angle = Math.atan2(ty - fy, tx - fx);
    const ah = 8;
    ctx.beginPath();
    ctx.moveTo(tx, ty);
    ctx.lineTo(tx - ah * Math.cos(angle - Math.PI / 6), ty - ah * Math.sin(angle - Math.PI / 6));
    ctx.lineTo(tx - ah * Math.cos(angle + Math.PI / 6), ty - ah * Math.sin(angle + Math.PI / 6));
    ctx.closePath();
    ctx.fillStyle = color;
    ctx.fill();

    ctx.restore();
  }

  // Draw background with subtle grid
  function drawBackground() {
    const grad = ctx.createLinearGradient(0, 0, w, h);
    grad.addColorStop(0, "rgba(15,23,42,1)");
    grad.addColorStop(1, "rgba(15,23,42,0.96)");
    ctx.fillStyle = grad;
    ctx.fillRect(0, 0, w, h);

    ctx.strokeStyle = "rgba(30,64,175,0.25)";
    ctx.lineWidth = 1;
    const step = 24;
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

  // Draw HUD showing current mode and metrics
  function drawHUD(route) {
    ctx.save();
    ctx.font = "11px monospace";
    ctx.textAlign = "left";
    ctx.fillStyle = "rgba(148,163,184,0.9)";

    ctx.fillText(
      `Mode: ${route.mode} | Chosen: ${llms[route.chosen].name}`,
      16,
      20
    );

    ctx.fillText(
      `~Tokens/s: ${metrics.tokensPerSec} | Lat: ${metrics.latencyMs}ms | Cost (rel): ${metrics.cost.toFixed(1)}`,
      16,
      36
    );

    ctx.restore();
  }

  // Main animation loop
  function drawScene() {
    t += 1;
    const route = currentRouteWithMetrics();

    ctx.clearRect(0, 0, w, h);
    drawBackground();

    // Core architecture boxes
    box(layout.client, { fill: "rgba(15,23,42,0.9)", stroke: "rgba(148,163,184,0.9)" });
    box(layout.router, { fill: "rgba(15,23,42,0.98)", stroke: "rgba(129,140,248,0.9)" });
    box(layout.vdb,    { fill: "rgba(15,23,42,0.95)", stroke: "rgba(52,211,153,0.8)" });
    box(layout.scorer, { fill: "rgba(15,23,42,0.98)", stroke: "rgba(244,114,182,0.8)" });
    box(layout.output, { fill: "rgba(15,23,42,0.9)", stroke: "rgba(148,163,184,0.9)" });

    // LLM model boxes
    llms.forEach((llm, i) => {
      const selected = route.chosen === i;
      const pos = { x: llm.x, y: llm.y, w: 110, h: 46, label: `${llm.name}\n${llm.subtitle}` };
      box(pos, {
        fill: selected ? "rgba(30,64,175,0.95)" : "rgba(15,23,42,0.9)",
        stroke: selected ? "rgba(129,140,248,0.95)" : "rgba(148,163,184,0.8)",
        textColor: selected ? "rgba(226,232,255,0.98)" : "rgba(226,232,240,0.9)"
      });
    });

    // Animated data flow arrows
    const pulse = (t % 300) / 300;

    // Client -> Router
    arrow(
      layout.client,
      layout.router,
      "rgba(129,140,248,0.9)",
      2,
      true,
      pulse
    );

    // Router -> Vector DB
    arrow(
      { x: layout.router.x, y: layout.router.y + 0.10 },
      { x: layout.vdb.x,    y: layout.vdb.y - 0.12 },
      "rgba(52,211,153,0.8)",
      1.8,
      true,
      pulse + 0.2
    );

    // Router -> Chosen LLM
    const chosenLLM = llms[route.chosen];
    arrow(
      { x: layout.router.x + 0.06, y: layout.router.y - 0.14 },
      { x: chosenLLM.x - 0.05,     y: chosenLLM.y },
      "rgba(129,140,248,0.9)",
      2,
      true,
      pulse + 0.4
    );

    // LLM -> Scorer
    arrow(
      { x: chosenLLM.x + 0.06, y: chosenLLM.y },
      { x: layout.scorer.x - 0.06, y: layout.scorer.y - 0.04 },
      "rgba(244,114,182,0.8)",
      1.8,
      true,
      pulse + 0.6
    );

    // Scorer -> Output
    arrow(
      layout.scorer,
      layout.output,
      "rgba(52,211,153,0.9)",
      2,
      true,
      pulse + 0.8
    );

    drawHUD(route);

    requestAnimationFrame(drawScene);
  }

  drawScene();

  // Optional: Register with SystemPlugins if available
  if (window.SystemPlugins && typeof window.SystemPlugins.registerNode === "function") {
    llms.forEach(llm => {
      window.SystemPlugins.registerNode(llm.name, {
        type: "llm",
        role: llm.subtitle,
        latency: 50,
        cost: 1.0
      });
    });
    console.log("[Orchestrator] Registered LLM nodes with SystemPlugins");
  }
})();
