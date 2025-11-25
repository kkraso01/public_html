const canvas = document.getElementById("attentionCanvas");
if (!canvas) {
  console.warn("[Attention] canvas not found");
} else {
  const ctx = canvas.getContext("2d");
  const bus = window.EventBus;

  let W, H, dpr;
  let running = true;
  let speed = 1;
  let entropySpike = 0;
  let time = 0;

  function resize() {
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

  const TOKENS = ["query", "retrieval", "route", "logits", "answer"];
  const HEADS = 6;

  let heatmap = generateHeatmap();

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

  function generateHeatmap() {
    return Array.from({ length: HEADS }, () =>
      Array.from({ length: TOKENS.length }, () =>
        Array.from({ length: TOKENS.length }, () => Math.random())
      )
    );
  }

  function drawTokens() {
    const tokenX = TOKENS.map((_, i) => W * 0.12 + i * (W * 0.18));
    const tokenY = H * 0.68;

    ctx.font = "12px monospace";
    ctx.textAlign = "center";
    TOKENS.forEach((tk, i) => {
      ctx.fillStyle = "rgba(148,163,184,0.9)";
      ctx.fillText(tk, tokenX[i], tokenY + 22);
      ctx.fillStyle = "rgba(255,255,255,0.9)";
      ctx.beginPath();
      ctx.arc(tokenX[i], tokenY, 10, 0, Math.PI * 2);
      ctx.fill();
    });
    return { tokenX, tokenY };
  }

  function drawRibbons(tokenPos) {
    const { tokenX, tokenY } = tokenPos;
    const headBaseY = H * 0.2;

    for (let h = 0; h < HEADS; h++) {
      const hue = 200 + h * 20;
      ctx.strokeStyle = `hsla(${hue}, 90%, 70%, 0.5)`;
      ctx.lineWidth = 2;
      ctx.beginPath();
      const startX = tokenX[0];
      const startY = headBaseY + h * 24;
      ctx.moveTo(startX, startY);
      for (let i = 1; i < tokenX.length; i++) {
        const ctrlY = startY + Math.sin(time * 0.01 * speed + i + h) * 10;
        ctx.bezierCurveTo(
          tokenX[i - 1] + 50,
          ctrlY,
          tokenX[i] - 50,
          ctrlY,
          tokenX[i],
          startY
        );
      }
      ctx.stroke();

      ctx.fillStyle = "rgba(226,232,240,0.9)";
      ctx.font = "10px monospace";
      ctx.fillText(`head ${h + 1}`, startX - 40, startY + 4);
    }

    // braided ribbons token -> top
    TOKENS.forEach((_, i) => {
      for (let j = 0; j < HEADS; j++) {
        const weight = heatmap[j][i % TOKENS.length][(i + j) % TOKENS.length];
        const alpha = 0.15 + weight * 0.6 + entropySpike * 0.1;
        ctx.strokeStyle = `rgba(79,70,229,${alpha})`;
        ctx.lineWidth = 1.6;
        ctx.beginPath();
        ctx.moveTo(tokenX[i], tokenY - 12);
        ctx.bezierCurveTo(
          tokenX[i],
          H * 0.5,
          W * 0.5,
          headBaseY + j * 24,
          W * 0.8,
          headBaseY + j * 24
        );
        ctx.stroke();
      }
    });
  }

  function drawHeatmap() {
    const boxSize = 12;
    const startX = W - HEADS * boxSize - 28;
    const startY = 24;

    ctx.fillStyle = "rgba(15,23,42,0.85)";
    ctx.strokeStyle = "rgba(129,140,248,0.45)";
    ctx.lineWidth = 1;
    ctx.beginPath();
    drawRoundedRect(startX - 8, startY - 14, HEADS * boxSize + 24, TOKENS.length * boxSize + 28, 8);
    ctx.fill();
    ctx.stroke();

    for (let i = 0; i < TOKENS.length; i++) {
      for (let h = 0; h < HEADS; h++) {
        const value = heatmap[h][i][(i + h) % TOKENS.length];
        const color = `rgba(79,70,229, ${0.2 + value * 0.8})`;
        ctx.fillStyle = color;
        ctx.fillRect(startX + h * boxSize, startY + i * boxSize, boxSize - 2, boxSize - 2);
      }
    }

    ctx.fillStyle = "rgba(148,163,184,0.9)";
    ctx.font = "10px monospace";
    ctx.fillText("head heatmap", startX - 2, startY + TOKENS.length * boxSize + 18);
  }

  function draw() {
    if (running) {
      time += 1 * speed;
      entropySpike = Math.max(0, entropySpike - 0.01);
      if (time % 80 === 0) heatmap = generateHeatmap();
    }

    ctx.clearRect(0, 0, W, H);
    ctx.fillStyle = "rgba(2,6,23,0.95)";
    ctx.fillRect(0, 0, W, H);

    const pos = drawTokens();
    drawRibbons(pos);
    drawHeatmap();

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
}
