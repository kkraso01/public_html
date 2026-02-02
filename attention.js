(function () {
  const canvas = document.getElementById("attentionCanvas");
  if (!canvas) {
    console.warn("[Attention] canvas not found");
    return;
  }

  const ctx = canvas.getContext("2d");
  const bus = window.EventBus;
  const theme = window.UI_THEME;

  let W, H, dpr;
  let running = true;
  let speed = 1;
  let entropySpike = 0;
  let time = 0;
  let hoverCell = null;

  function resize() {
    if (theme) {
      const metrics = theme.setDPR(canvas, ctx);
      W = metrics.width;
      H = metrics.height;
      dpr = metrics.dpr;
      return;
    }
    dpr = window.devicePixelRatio || 1;
    const rect = canvas.getBoundingClientRect();
    W = rect.width;
    H = rect.height;
    canvas.width = W * dpr;
    canvas.height = H * dpr;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  }

  resize();
  window.addEventListener("resize", resize);
  window.addEventListener("themechange", resize);

  const control = { isRunning: true };
  window.ViewportObserver.observe(canvas, control, 0.1);

  const TOKENS = ["query", "retrieval", "route", "context", "logits", "answer"];
  const SIZE = TOKENS.length;

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

  function valueAt(i, j) {
    const base = Math.sin(time * 0.01 + i * 1.3 + j * 0.7);
    return (base + 1) / 2;
  }

  function drawMatrix(colors) {
    const margin = 30;
    const gridSize = Math.min(W, H) - margin * 2;
    const cell = gridSize / SIZE;
    const startX = (W - gridSize) / 2;
    const startY = (H - gridSize) / 2;

    ctx.fillStyle = colors.surface;
    ctx.fillRect(0, 0, W, H);

    if (theme) {
      theme.drawGrid(ctx, 24, colors.border, theme.isDark() ? 0.1 : 0.06, W, H);
    }

    ctx.save();
    ctx.strokeStyle = theme ? theme.rgba(colors.border, 0.7) : colors.border;
    ctx.lineWidth = 1;
    ctx.beginPath();
    drawRoundedRect(startX - 8, startY - 8, gridSize + 16, gridSize + 16, 2);
    ctx.fillStyle = colors.surfaceElevated;
    ctx.fill();
    ctx.stroke();

    for (let row = 0; row < SIZE; row += 1) {
      for (let col = 0; col < SIZE; col += 1) {
        const value = valueAt(row, col);
        const alpha = 0.08 + value * 0.55 + entropySpike * 0.05;
        ctx.fillStyle = theme ? theme.rgba(colors.text, alpha) : `rgba(226,232,240,${alpha})`;
        ctx.fillRect(startX + col * cell, startY + row * cell, cell - 1, cell - 1);
      }
    }

    ctx.restore();

    ctx.save();
    ctx.font = "600 10px ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace";
    ctx.fillStyle = colors.muted;
    ctx.textAlign = "center";
    TOKENS.forEach((token, i) => {
      ctx.fillText(token.toUpperCase(), startX + i * cell + cell / 2, startY - 14);
      ctx.save();
      ctx.translate(startX - 12, startY + i * cell + cell / 2);
      ctx.rotate(-Math.PI / 2);
      ctx.fillText(token.toUpperCase(), 0, 0);
      ctx.restore();
    });

    const legendX = startX + gridSize - 80;
    const legendY = startY + gridSize + 12;
    ctx.fillStyle = colors.muted;
    ctx.textAlign = "left";
    ctx.fillText("LOW", legendX, legendY + 10);
    ctx.fillText("HIGH", legendX + 60, legendY + 10);
    for (let i = 0; i <= 6; i += 1) {
      const value = i / 6;
      const alpha = 0.08 + value * 0.55;
      ctx.fillStyle = theme ? theme.rgba(colors.text, alpha) : `rgba(226,232,240,${alpha})`;
      ctx.fillRect(legendX + i * 10, legendY - 6, 8, 6);
    }
    ctx.restore();

    if (hoverCell) {
      ctx.save();
      ctx.strokeStyle = theme ? theme.rgba(colors.accent, 0.9) : colors.accent;
      ctx.lineWidth = 1;
      ctx.strokeRect(startX + hoverCell.col * cell, startY + hoverCell.row * cell, cell, cell);
      ctx.strokeRect(startX, startY + hoverCell.row * cell, gridSize, cell);
      ctx.strokeRect(startX + hoverCell.col * cell, startY, cell, gridSize);
      ctx.restore();
    }

    return { startX, startY, cell, gridSize };
  }

  function draw() {
    if (control.isRunning) {
      if (running) {
        time += 1 * speed;
        entropySpike = Math.max(0, entropySpike - 0.01);
      }
    }

    const colors = palette();
    const grid = drawMatrix(colors);

    if (hoverCell && grid) {
      hoverCell.x = grid.startX + hoverCell.col * grid.cell;
      hoverCell.y = grid.startY + hoverCell.row * grid.cell;
    }

    requestAnimationFrame(draw);
  }

  function handleControl(payload) {
    if (!payload) return;
    if (payload.action === "toggle") running = !payload.value;
    if (payload.action === "speed" && typeof payload.value === "number") speed = payload.value;
    if (payload.action === "spike") {
      entropySpike = Math.min(3, entropySpike + (payload.value || 1));
      if (bus) bus.emit("telemetry:spike", { source: "attention", intensity: entropySpike });
    }
  }

  canvas.addEventListener("mousemove", (event) => {
    const rect = canvas.getBoundingClientRect();
    const x = event.clientX - rect.left;
    const y = event.clientY - rect.top;
    const margin = 30;
    const gridSize = Math.min(W, H) - margin * 2;
    const cell = gridSize / SIZE;
    const startX = (W - gridSize) / 2;
    const startY = (H - gridSize) / 2;
    const col = Math.floor((x - startX) / cell);
    const row = Math.floor((y - startY) / cell);
    if (col >= 0 && col < SIZE && row >= 0 && row < SIZE) {
      hoverCell = { row, col };
    } else {
      hoverCell = null;
    }
  });

  canvas.addEventListener("mouseleave", () => {
    hoverCell = null;
  });

  canvas.addEventListener("click", () => {
    entropySpike = Math.min(3, entropySpike + 0.8);
    if (bus) bus.emit("telemetry:spike", { source: "attention", intensity: entropySpike });
  });

  if (bus) {
    bus.on("control:attention", handleControl);
    bus.on("telemetry:spike", ({ source, intensity }) => {
      if (source === "attention") return;
      entropySpike = Math.min(3, entropySpike + (intensity || 0.4));
    });
  }

  draw();
})();
