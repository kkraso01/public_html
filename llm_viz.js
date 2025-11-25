(function () {
  const canvas = document.getElementById("llm-viz");
  if (!canvas) return;

  const ctx = canvas.getContext("2d");
  const bus = window.EventBus;
  let width, height, dpr;

  const TOKENS = ["<s>", "user", "asks", "LLM"];
  let time = 0;
  let running = true;
  let speed = 1;
  let loadSpike = 0;

  function resize() {
    dpr = window.devicePixelRatio || 1;
    const rect = canvas.getBoundingClientRect();
    width = rect.width;
    height = rect.height || 260;

    canvas.width = width * dpr;
    canvas.height = height * dpr;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  }

  window.addEventListener("resize", resize);
  resize();

  function lerp(a, b, t) {
    return a + (b - a) * t;
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

  function drawBackground() {
    const grad = ctx.createLinearGradient(0, 0, width, height);
    grad.addColorStop(0, "#0b1224");
    grad.addColorStop(1, "#050910");
    ctx.fillStyle = grad;
    ctx.fillRect(0, 0, width, height);

    ctx.strokeStyle = "rgba(56, 189, 248, 0.25)";
    ctx.lineWidth = 1;
    const gridSpacing = 22;
    ctx.beginPath();
    for (let x = 0; x < width; x += gridSpacing) {
      ctx.moveTo(x, 0);
      ctx.lineTo(x, height);
    }
    for (let y = 0; y < height; y += gridSpacing) {
      ctx.moveTo(0, y);
      ctx.lineTo(width, y);
    }
    ctx.stroke();

    // highlight band for the transformer trunk
    ctx.fillStyle = "rgba(15,23,42,0.7)";
    drawRoundedRect(width * 0.08, height * 0.18, width * 0.84, height * 0.64, 20);
    ctx.fill();
    ctx.strokeStyle = "rgba(99,102,241,0.35)";
    ctx.stroke();
  }

  function draw() {
    if (running) {
      time += 0.015 * speed;
      loadSpike = Math.max(0, loadSpike - 0.01);
    }

    ctx.clearRect(0, 0, width, height);
    drawBackground();

    const leftX = width * 0.12;
    const embedX = width * 0.28;
    const block1X = width * 0.48;
    const block2X = width * 0.66;
    const logitsX = width * 0.84;
    const centerY = height * 0.52;

    const tokenAreaHeight = height * 0.5;
    const tokenTop = centerY - tokenAreaHeight / 2;
    const tokenSpacing = tokenAreaHeight / (TOKENS.length - 1);
    const tokenPos = TOKENS.map((_, i) => ({
      x: leftX,
      y: tokenTop + i * tokenSpacing
    }));

    const headsPerBlock = 4;
    const headRadius = 6;

    const blockHeight = 96;
    const block1Y = centerY - blockHeight - 16;
    const block2Y = centerY + 8;

    function pulse(offset) {
      return 0.4 + 0.6 * Math.max(0, Math.sin(time * 2 + offset));
    }

    // Embedding box
    const embedW = 92;
    const embedH = 70;
    const embedY = centerY - embedH / 2;
    drawRoundedRect(embedX - embedW / 2, embedY, embedW, embedH, 10);
    ctx.fillStyle = "rgba(15,23,42,0.92)";
    ctx.fill();
    ctx.strokeStyle = "rgba(56,189,248,0.9)";
    ctx.lineWidth = 1.5;
    ctx.stroke();

    ctx.fillStyle = "rgba(226,232,240,0.92)";
    ctx.font = "11px monospace";
    ctx.textAlign = "center";
    ctx.fillText("Embeddings", embedX, embedY + embedH / 2 + 4);

    // Transformer blocks
    function drawBlock(x, y, label, sublabel, accent) {
      const w = 130;
      const h = blockHeight;
      drawRoundedRect(x - w / 2, y, w, h, 12);
      ctx.fillStyle = "rgba(10,12,24,0.95)";
      ctx.fill();
      ctx.strokeStyle = accent;
      ctx.lineWidth = 1.8;
      ctx.stroke();

      ctx.fillStyle = "rgba(226,232,240,0.92)";
      ctx.font = "11px monospace";
      ctx.textAlign = "left";
      ctx.fillText(label, x - w / 2 + 10, y + 18);

      ctx.fillStyle = "rgba(148,163,184,0.9)";
      ctx.font = "10px monospace";
      ctx.fillText(sublabel, x - w / 2 + 10, y + 34);
    }

    drawBlock(block1X, block1Y, "Block 1", "Multi-Head Attention", "#a78bfa");
    drawBlock(block2X, block2Y, "Block 2", "Feed Forward + Residual", "#38bdf8");

    // logits box
    const logitsW = 90;
    const logitsH = 90;
    const logitsY = centerY - logitsH / 2;
    drawRoundedRect(logitsX - logitsW / 2, logitsY, logitsW, logitsH, 12);
    ctx.fillStyle = "rgba(15,23,42,0.92)";
    ctx.fill();
    ctx.strokeStyle = "rgba(34,197,94,0.9)";
    ctx.lineWidth = 1.6;
    ctx.stroke();

    ctx.fillStyle = "rgba(226,232,240,0.9)";
    ctx.font = "11px monospace";
    ctx.textAlign = "center";
    ctx.fillText("Logits", logitsX, logitsY + logitsH / 2 + 4);

    for (let i = 0; i < 6; i++) {
      const bx = logitsX - 28 + i * 11;
      const bh = 12 + 16 * Math.abs(Math.sin(time * 1.5 + i));
      ctx.fillStyle = `rgba(34,197,94,${0.4 + 0.5 * Math.random()})`;
      ctx.fillRect(bx, logitsY + logitsH - bh - 10, 7, bh);
    }

    // Tokens
    ctx.font = "12px monospace";
    ctx.textAlign = "center";
    tokenPos.forEach((p, i) => {
      drawRoundedRect(p.x - 14, p.y - 14, 28, 28, 6);
      ctx.fillStyle = "rgba(15,23,42,0.9)";
      ctx.fill();
      ctx.strokeStyle = "rgba(148,163,184,0.9)";
      ctx.lineWidth = 1.2;
      ctx.stroke();

      ctx.fillStyle = "rgba(226,232,240,0.95)";
      ctx.fillText(TOKENS[i], p.x, p.y + 4);
    });

    // token -> embedding
    tokenPos.forEach((p, i) => {
      const alpha = pulse(i * 0.6);
      ctx.strokeStyle = `rgba(59,130,246,${alpha})`;
      ctx.lineWidth = 1.6;
      ctx.beginPath();
      ctx.moveTo(p.x + 14, p.y);
      ctx.lineTo(embedX - embedW / 2, lerp(p.y, centerY, 0.4));
      ctx.stroke();
    });

    // heads in block1
    const heads = [];
    const headAreaY = block1Y + 48;
    const headSpacing = 22;

    for (let h = 0; h < headsPerBlock; h++) {
      const hx = block1X - 34 + h * headSpacing;
      const hy = headAreaY;
      heads.push({ x: hx, y: hy });

      const alpha = 0.6 + 0.3 * Math.max(0, Math.sin(time * 2 + h));
      ctx.beginPath();
      ctx.fillStyle = `rgba(168,85,247,${alpha})`;
      ctx.arc(hx, hy, headRadius, 0, Math.PI * 2);
      ctx.fill();
      ctx.strokeStyle = "rgba(226,232,240,0.6)";
      ctx.stroke();
    }

    // Q/K/V marker
    ctx.fillStyle = "rgba(148,163,184,0.8)";
    ctx.font = "10px monospace";
    ctx.textAlign = "left";
    ctx.fillText("Q / K / V projection", block1X - 60, block1Y + 22);

    // token -> heads attention lines
    tokenPos.forEach((tp, ti) => {
      heads.forEach((h, hi) => {
        const phase = time * 2 + ti * 0.5 + hi * 0.7;
        const alpha = 0.15 + 0.6 * Math.max(0, Math.sin(phase));
        ctx.strokeStyle = `rgba(129,140,248,${alpha})`;
        ctx.lineWidth = 1;
        ctx.beginPath();
        ctx.moveTo(lerp(embedX + embedW / 2, block1X - 60, 0.7), lerp(tp.y, h.y, 0.35));
        ctx.lineTo(h.x, h.y);
        ctx.stroke();
      });
    });

    // heads -> block1 output
    heads.forEach((h, hi) => {
      const alpha = pulse(hi);
      ctx.strokeStyle = `rgba(94,234,212,${alpha})`;
      ctx.lineWidth = 1.4;
      ctx.beginPath();
      ctx.moveTo(h.x, h.y);
      ctx.lineTo(block1X + 52, centerY - 4);
      ctx.stroke();
    });

    // feed-forward bars inside block2
    const ffBaseX = block2X - 38;
    const ffBaseY = block2Y + 30;
    for (let i = 0; i < 4; i++) {
      const barW = 18 + 12 * Math.abs(Math.sin(time * 1.8 + i));
      const barY = ffBaseY + i * 16;
      ctx.fillStyle = `rgba(56,189,248,${0.6 + 0.3 * Math.sin(time + i)})`;
      ctx.fillRect(ffBaseX, barY, barW, 8);
    }

    // residual stream path
    const pts = [
      { x: embedX + embedW / 2, y: centerY },
      { x: block1X + 60, y: centerY - 6 },
      { x: block2X - 60, y: centerY + blockHeight / 2 },
      { x: block2X + 60, y: centerY + blockHeight / 2 },
      { x: logitsX - logitsW / 2, y: centerY }
    ];

    ctx.strokeStyle = "rgba(52,211,153,0.85)";
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(pts[0].x, pts[0].y);
    for (let i = 1; i < pts.length; i++) {
      ctx.lineTo(pts[i].x, pts[i].y);
    }
    ctx.stroke();

    const tWrap = (Math.sin(time * 1.5) + 1) / 2;
    const segCount = pts.length - 1;
    let totalLen = 0;
    const segLen = [];
    for (let i = 0; i < segCount; i++) {
      const dx = pts[i + 1].x - pts[i].x;
      const dy = pts[i + 1].y - pts[i].y;
      const l = Math.sqrt(dx * dx + dy * dy);
      segLen.push(l);
      totalLen += l;
    }
    const pulseDist = tWrap * totalLen;

    let acc = 0;
    for (let i = 0; i < segCount; i++) {
      if (pulseDist <= acc + segLen[i]) {
        const localT = (pulseDist - acc) / segLen[i];
        const px = lerp(pts[i].x, pts[i + 1].x, localT);
        const py = lerp(pts[i].y, pts[i + 1].y, localT);

        ctx.fillStyle = "rgba(52,211,153,0.9)";
        ctx.beginPath();
        ctx.arc(px, py, 5 + loadSpike * 0.5, 0, Math.PI * 2);
        ctx.fill();
        break;
      }
      acc += segLen[i];
    }

    // labels
    ctx.font = "10px monospace";
    ctx.textAlign = "left";
    ctx.fillStyle = "rgba(148,163,184,0.9)";
    ctx.fillText("Tokens", leftX - 22, tokenTop - 12);
    ctx.fillText("Attention heads", block1X - 46, block1Y - 10);
    ctx.fillText("Feed-forward / residual", block2X - 70, block2Y + blockHeight + 18);
    ctx.fillText("Next-token distribution", logitsX - 48, logitsY - 12);

    // HUD
    ctx.fillStyle = "rgba(15,23,42,0.82)";
    ctx.strokeStyle = "rgba(129,140,248,0.45)";
    ctx.lineWidth = 1;
    ctx.beginPath();
    drawRoundedRect(width - 180, 12, 160, 86, 12);
    ctx.fill();
    ctx.stroke();
    ctx.fillStyle = "rgba(226,232,240,0.9)";
    ctx.font = "10px monospace";
    ctx.fillText(`speed x${speed.toFixed(2)}`, width - 168, 36);
    ctx.fillStyle = "rgba(52,211,153,0.9)";
    ctx.fillText(`load ${loadSpike.toFixed(2)}`, width - 168, 54);
    ctx.fillStyle = "rgba(148,163,184,0.85)";
    ctx.fillText("click canvas = spike", width - 168, 72);

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

  if (bus) {
    bus.on("control:transformer", handleControl);
    bus.on("telemetry:spike", ({ source, intensity }) => {
      if (source === "transformer") return;
      loadSpike = Math.min(3, loadSpike + (intensity || 0.4));
    });
  }
})();
