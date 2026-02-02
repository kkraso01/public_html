(function () {
  const canvas = document.getElementById("llm-viz");
  if (!canvas) return;

  const ctx = canvas.getContext("2d");
  const bus = window.EventBus;
  const theme = window.UI_THEME;
  let width, height, dpr;

  let time = 0;
  let running = true;
  let speed = 1;
  let loadSpike = 0;
  let hoveredLayer = null;
  let lastMouse = null;

  function resize() {
    if (theme) {
      const metrics = theme.setDPR(canvas, ctx);
      width = metrics.width;
      height = metrics.height || 260;
      dpr = metrics.dpr;
      return;
    }
    dpr = window.devicePixelRatio || 1;
    const rect = canvas.getBoundingClientRect();
    width = rect.width;
    height = rect.height || 260;

    canvas.width = width * dpr;
    canvas.height = height * dpr;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  }

  window.addEventListener("resize", resize);
  window.addEventListener("themechange", resize);
  resize();

  const control = { isRunning: true };
  window.ViewportObserver.observe(canvas, control, 0.1);

  const layers = [
    { id: "embed", label: "Embeddings", x: 0.12, y: 0.2, w: 80, h: 200 },
    { id: "attn1", label: "Attention", x: 0.32, y: 0.15, w: 110, h: 220 },
    { id: "ffn", label: "Feed Forward", x: 0.56, y: 0.18, w: 110, h: 210 },
    { id: "logits", label: "Logits", x: 0.80, y: 0.22, w: 90, h: 190 }
  ];

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

  function drawRoundedRect(x, y, w, h, r) {
    if (ctx.roundRect) {
      ctx.roundRect(x, y, w, h, r);
      return;
    }

    const rr = Math.min(r, w / 2, h / 2);
    ctx.beginPath();
    ctx.moveTo(x + rr, y);
    ctx.lineTo(x + w - rr, y);
    ctx.quadraticCurveTo(x + w, y, x + w, y + rr);
    ctx.lineTo(x + w, y + h - rr);
    ctx.quadraticCurveTo(x + w, y + h, x + w - rr, y + h);
    ctx.lineTo(x + rr, y + h);
    ctx.quadraticCurveTo(x, y + h, x, y + h - rr);
    ctx.lineTo(x, y + rr);
    ctx.quadraticCurveTo(x, y, x + rr, y);
    ctx.closePath();
  }

  function drawBackground(colors) {
    ctx.fillStyle = colors.surface;
    ctx.fillRect(0, 0, width, height);
    if (theme) {
      theme.drawGrid(ctx, 22, colors.border, theme.isDark() ? 0.12 : 0.08, width, height);
    }
  }

  function drawLayer(layer, colors) {
    const x = layer.x * width;
    const y = layer.y * height;
    const w = layer.w;
    const h = layer.h;
    const isActive = hoveredLayer && hoveredLayer.id === layer.id;

    ctx.save();
    ctx.fillStyle = colors.surfaceElevated;
    ctx.strokeStyle = theme ? theme.rgba(isActive ? colors.accent : colors.border, isActive ? 0.9 : 0.7) : colors.border;
    ctx.lineWidth = 1;
    ctx.beginPath();
    drawRoundedRect(x - w / 2, y, w, h, 2);
    ctx.fill();
    ctx.stroke();

    ctx.fillStyle = colors.textStrong;
    ctx.font = "600 10px ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace";
    ctx.textAlign = "center";
    ctx.fillText(layer.label.toUpperCase(), x, y + 14);

    const matrixSize = 4;
    const cellSize = 10;
    const matrixX = x - (matrixSize * cellSize) / 2;
    const matrixY = y + 36;

    for (let row = 0; row < matrixSize; row += 1) {
      for (let col = 0; col < matrixSize; col += 1) {
        const phase = (Math.sin(time * 0.8 + row * 0.7 + col * 0.5) + 1) / 2;
        const alpha = 0.18 + phase * 0.25;
        ctx.fillStyle = theme ? theme.rgba(colors.text, alpha) : `rgba(226,232,240,${alpha})`;
        ctx.fillRect(matrixX + col * cellSize, matrixY + row * cellSize, cellSize - 1, cellSize - 1);
      }
    }

    ctx.strokeStyle = theme ? theme.rgba(colors.border, 0.7) : colors.border;
    ctx.strokeRect(matrixX - 4, matrixY - 4, matrixSize * cellSize + 8, matrixSize * cellSize + 8);

    ctx.restore();
    return { x: x - w / 2, y, w, h };
  }

  function drawResidual(colors) {
    const startX = layers[0].x * width + layers[0].w / 2;
    const endX = layers[layers.length - 1].x * width - layers[layers.length - 1].w / 2;
    const y = height * 0.82;

    ctx.save();
    ctx.strokeStyle = theme ? theme.rgba(colors.border, 0.7) : colors.border;
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(startX, y);
    ctx.lineTo(endX, y);
    ctx.stroke();

    const pulse = Math.floor((time * 0.9) % 8) / 8;
    const px = startX + (endX - startX) * pulse;
    ctx.fillStyle = theme ? theme.rgba(colors.accent, 0.85) : colors.accent;
    ctx.fillRect(px - 4, y - 4, 8, 8);
    ctx.restore();
  }

  function drawTooltip(colors) {
    if (!hoveredLayer || !lastMouse) return;
    const text = hoveredLayer.label;
    const padding = 8;
    ctx.save();
    ctx.font = "600 10px ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace";
    const metrics = ctx.measureText(text);
    const boxWidth = metrics.width + padding * 2;
    const boxHeight = 24;
    const x = Math.min(lastMouse.x + 12, width - boxWidth - 8);
    const y = Math.min(lastMouse.y + 12, height - boxHeight - 8);
    ctx.fillStyle = colors.surfaceElevated;
    ctx.strokeStyle = theme ? theme.rgba(colors.border, 0.8) : colors.border;
    ctx.lineWidth = 1;
    ctx.beginPath();
    drawRoundedRect(x, y, boxWidth, boxHeight, 2);
    ctx.fill();
    ctx.stroke();
    ctx.fillStyle = colors.textStrong;
    ctx.textAlign = "left";
    ctx.textBaseline = "middle";
    ctx.fillText(text, x + padding, y + boxHeight / 2);
    ctx.restore();
  }

  function draw() {
    if (control.isRunning) {
      if (running) {
        time += 0.015 * speed;
        loadSpike = Math.max(0, loadSpike - 0.01);
      }
    }

    const colors = palette();
    ctx.clearRect(0, 0, width, height);
    drawBackground(colors);

    layers.forEach((layer) => drawLayer(layer, colors));
    drawResidual(colors);
    drawTooltip(colors);

    ctx.save();
    ctx.fillStyle = colors.muted;
    ctx.font = "600 10px ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace";
    ctx.textAlign = "left";
    ctx.fillText(`load ${loadSpike.toFixed(2)} · speed x${speed.toFixed(2)}`, 16, height - 16);
    ctx.restore();

    requestAnimationFrame(draw);
  }

  draw();

  function handleControl(payload) {
    if (!payload) return;
    if (payload.action === "toggle") running = !payload.value;
    if (payload.action === "speed" && typeof payload.value === "number") speed = payload.value;
    if (payload.action === "spike") {
      loadSpike = Math.min(3, loadSpike + (payload.value || 1));
      if (bus) bus.emit("telemetry:spike", { source: "transformer", intensity: loadSpike });
    }
  }

  canvas.addEventListener("click", () => {
    loadSpike = Math.min(3, loadSpike + 0.6);
    if (bus) bus.emit("telemetry:spike", { source: "transformer", intensity: loadSpike });
  });

  canvas.addEventListener("mousemove", (event) => {
    const rect = canvas.getBoundingClientRect();
    const x = event.clientX - rect.left;
    const y = event.clientY - rect.top;
    lastMouse = { x, y };
    hoveredLayer = null;
    layers.forEach((layer) => {
      const box = {
        x: layer.x * width - layer.w / 2,
        y: layer.y * height,
        w: layer.w,
        h: layer.h,
        id: layer.id,
        label: layer.label
      };
      if (x >= box.x && x <= box.x + box.w && y >= box.y && y <= box.y + box.h) {
        hoveredLayer = box;
      }
    });
  });

  canvas.addEventListener("mouseleave", () => {
    hoveredLayer = null;
    lastMouse = null;
  });

  if (bus) {
    bus.on("control:transformer", handleControl);
    bus.on("telemetry:spike", ({ source, intensity }) => {
      if (source === "transformer") return;
      loadSpike = Math.min(3, loadSpike + (intensity || 0.4));
    });
  }
})();
