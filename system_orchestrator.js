// system_orchestrator.js
// Multi-LLM Orchestrator Visualization - LLM-MS Architecture Demo

(function () {
  const canvas = document.getElementById("orchestratorCanvas");
  if (!canvas) {
    console.warn("[Orchestrator] canvas not found");
    return;
  }

  const ctx = canvas.getContext("2d");
  const bus = window.EventBus;
  const theme = window.UI_THEME;
  let w, h, dpr;
  let t = 0;
  let running = true;
  let speed = 1;
  let spike = 0;

  function resize() {
    if (theme) {
      const metrics = theme.setDPR(canvas, ctx);
      w = metrics.width;
      h = metrics.height;
      dpr = metrics.dpr;
      return;
    }
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
  window.addEventListener("themechange", resize);

  // Viewport observer for performance
  const control = { isRunning: true };
  window.ViewportObserver.observe(canvas, control, 0.1);

  const layout = {
    client: { x: 0.10, y: 0.50, w: 84, h: 38, label: "Client" },
    router: { x: 0.30, y: 0.50, w: 132, h: 50, label: "Router" },
    vdb: { x: 0.30, y: 0.78, w: 132, h: 36, label: "Vector DB" },
    scorer: { x: 0.58, y: 0.50, w: 138, h: 46, label: "Scoring" },
    output: { x: 0.84, y: 0.50, w: 90, h: 38, label: "Response" }
  };

  const llms = [
    { name: "LLM-A", subtitle: "fast / cheap", x: 0.62, y: 0.24 },
    { name: "LLM-B", subtitle: "balanced", x: 0.62, y: 0.42 },
    { name: "LLM-C", subtitle: "slow / strong", x: 0.62, y: 0.60 }
  ];

  const routes = [
    { mode: "cost-aware", chosen: 0 },
    { mode: "balanced", chosen: 1 },
    { mode: "quality", chosen: 2 }
  ];

  const metrics = {
    tokensPerSec: 0,
    latencyMs: 0,
    cost: 0
  };

  function updateMetrics(routeIndex) {
    if (routeIndex === 0) {
      metrics.tokensPerSec = 180;
      metrics.latencyMs = 40;
      metrics.cost = 1.0;
    } else if (routeIndex === 1) {
      metrics.tokensPerSec = 120;
      metrics.latencyMs = 60;
      metrics.cost = 1.4;
    } else {
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

  function drawBackground(colors) {
    ctx.fillStyle = colors.surface;
    ctx.fillRect(0, 0, w, h);
    if (theme) {
      theme.drawGrid(ctx, 26, colors.border, theme.isDark() ? 0.12 : 0.08, w, h);
    }
  }

  function drawBox(pos, colors, options = {}) {
    const { x, y, w: bw, h: bh, label } = pos;
    const px = x * w - bw / 2;
    const py = y * h - bh / 2;
    const borderColor = options.accent ? colors.accent : colors.border;

    ctx.save();
    ctx.fillStyle = options.fill || colors.surfaceElevated;
    ctx.strokeStyle = theme ? theme.rgba(borderColor, options.accent ? 0.9 : 0.8) : borderColor;
    ctx.lineWidth = 1;
    ctx.beginPath();
    drawRoundedRect(px, py, bw, bh, 2);
    ctx.fill();
    ctx.stroke();

    ctx.fillStyle = colors.textStrong;
    ctx.font = "600 10px ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace";
    ctx.textAlign = "center";
    ctx.textBaseline = "middle";
    ctx.fillText(label, x * w, y * h);

    ctx.restore();
  }

  function drawRack(colors, activeIndex) {
    const rackX = w * 0.62 - 70;
    const rackY = h * 0.16;
    const rackW = 140;
    const rackH = h * 0.56;

    ctx.save();
    ctx.fillStyle = colors.surface;
    ctx.strokeStyle = theme ? theme.rgba(colors.border, 0.9) : colors.border;
    ctx.lineWidth = 1;
    ctx.beginPath();
    drawRoundedRect(rackX, rackY, rackW, rackH, 2);
    ctx.fill();
    ctx.stroke();

    ctx.fillStyle = colors.muted;
    ctx.font = "600 9px ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace";
    ctx.textAlign = "left";
    ctx.fillText("LLM POOL", rackX + 10, rackY + 14);

    llms.forEach((llm, i) => {
      const slotW = 120;
      const slotH = 36;
      const slotX = rackX + 10;
      const slotY = rackY + 24 + i * 56;
      const isActive = activeIndex === i;

      ctx.fillStyle = colors.surfaceElevated;
      ctx.strokeStyle = theme ? theme.rgba(isActive ? colors.accent : colors.border, isActive ? 0.9 : 0.6) : colors.border;
      ctx.lineWidth = 1;
      ctx.beginPath();
      drawRoundedRect(slotX, slotY, slotW, slotH, 2);
      ctx.fill();
      ctx.stroke();

      ctx.fillStyle = colors.textStrong;
      ctx.font = "600 10px ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace";
      ctx.textAlign = "left";
      ctx.fillText(llm.name, slotX + 8, slotY + 14);
      ctx.fillStyle = colors.muted;
      ctx.font = "9px ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace";
      ctx.fillText(llm.subtitle, slotX + 8, slotY + 28);
    });

    ctx.restore();
  }

  function drawArrow(from, to, color, alpha = 0.7) {
    const fx = from.x * w;
    const fy = from.y * h;
    const tx = to.x * w;
    const ty = to.y * h;

    ctx.save();
    ctx.strokeStyle = theme ? theme.rgba(color, alpha) : color;
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(fx, fy);
    ctx.lineTo(tx, ty);
    ctx.stroke();

    const angle = Math.atan2(ty - fy, tx - fx);
    const ah = 7;
    ctx.beginPath();
    ctx.moveTo(tx, ty);
    ctx.lineTo(tx - ah * Math.cos(angle - Math.PI / 6), ty - ah * Math.sin(angle - Math.PI / 6));
    ctx.lineTo(tx - ah * Math.cos(angle + Math.PI / 6), ty - ah * Math.sin(angle + Math.PI / 6));
    ctx.closePath();
    ctx.fillStyle = theme ? theme.rgba(color, alpha) : color;
    ctx.fill();
    ctx.restore();
  }

  function drawPacket(path, progress, colors, active) {
    const [start, end] = path;
    const px = lerp(start.x * w, end.x * w, progress);
    const py = lerp(start.y * h, end.y * h, progress);
    ctx.save();
    ctx.fillStyle = theme ? theme.rgba(active ? colors.accent : colors.text, active ? 0.9 : 0.7) : colors.text;
    ctx.fillRect(px - 3, py - 3, 6, 6);
    ctx.restore();
  }

  function lerp(a, b, p) {
    return a + (b - a) * p;
  }

  function drawHUD(route, colors) {
    ctx.save();
    ctx.fillStyle = colors.muted;
    ctx.font = "600 10px ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace";
    ctx.textAlign = "left";
    ctx.fillText(`MODE: ${route.mode.toUpperCase()}`, 16, 20);
    ctx.fillText(`LAT ${metrics.latencyMs}ms  TPS ${metrics.tokensPerSec}  COST ${metrics.cost.toFixed(1)}`, 16, 36);
    ctx.restore();
  }

  function drawScene() {
    if (control.isRunning) {
      if (running) {
        t += 1 * speed;
        spike = Math.max(0, spike - 0.01);
      }
    }
    const route = currentRouteWithMetrics();
    const colors = palette();

    ctx.clearRect(0, 0, w, h);
    drawBackground(colors);

    drawBox(layout.client, colors);
    drawBox(layout.router, colors, { accent: true });
    drawBox(layout.vdb, colors);
    drawBox(layout.scorer, colors);
    drawBox(layout.output, colors);

    drawRack(colors, route.chosen);

    const activeColor = colors.accent;
    const passiveColor = colors.border;

    drawArrow(layout.client, layout.router, passiveColor, 0.7 + spike * 0.1);
    drawArrow({ x: layout.router.x, y: layout.router.y + 0.12 }, { x: layout.vdb.x, y: layout.vdb.y - 0.12 }, passiveColor, 0.6);
    drawArrow({ x: layout.router.x + 0.07, y: layout.router.y - 0.08 }, { x: llms[route.chosen].x - 0.08, y: llms[route.chosen].y }, activeColor, 0.85);
    drawArrow({ x: llms[route.chosen].x + 0.08, y: llms[route.chosen].y }, { x: layout.scorer.x - 0.08, y: layout.scorer.y }, activeColor, 0.85);
    drawArrow(layout.scorer, layout.output, passiveColor, 0.7);

    const packetPhase = (t % 200) / 200;
    const stepped = Math.floor(packetPhase * 8) / 8;
    drawPacket([{ x: layout.client.x, y: layout.client.y }, { x: layout.router.x, y: layout.router.y }], stepped, colors, false);
    drawPacket([{ x: layout.router.x + 0.07, y: layout.router.y - 0.08 }, { x: llms[route.chosen].x - 0.08, y: llms[route.chosen].y }], stepped, colors, true);
    drawPacket([{ x: llms[route.chosen].x + 0.08, y: llms[route.chosen].y }, { x: layout.scorer.x - 0.08, y: layout.scorer.y }], stepped, colors, true);
    drawPacket([{ x: layout.scorer.x, y: layout.scorer.y }, { x: layout.output.x, y: layout.output.y }], stepped, colors, false);

    drawHUD(route, colors);

    requestAnimationFrame(drawScene);
  }

  drawScene();

  function handleControl(payload) {
    if (!payload) return;
    if (payload.action === "toggle") running = !payload.value;
    if (payload.action === "speed" && typeof payload.value === "number") speed = payload.value;
    if (payload.action === "spike") {
      spike = Math.min(3, spike + (payload.value || 1));
      if (bus) bus.emit("telemetry:spike", { source: "orchestrator", intensity: spike });
    }
  }

  if (bus) {
    bus.on("control:orchestrator", handleControl);
    bus.on("telemetry:spike", ({ source, intensity }) => {
      if (source === "orchestrator") return;
      spike = Math.min(3, spike + (intensity || 0.4));
    });
  }

  if (window.SystemPlugins && typeof window.SystemPlugins.registerNode === "function") {
    llms.forEach((llm) => {
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
