(function () {
  const canvas = document.getElementById("llm-viz");
  if (!canvas) return;

  const ctx = canvas.getContext("2d");
  const bus = window.EventBus;
  let width, height, dpr;

  const TOKENS = ["I", "query", "an", "LLM"];
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

  function draw() {
    if (running) {
      time += 0.015 * speed;
      loadSpike = Math.max(0, loadSpike - 0.01);
    }

    ctx.clearRect(0, 0, width, height);

    // background panel / subtle grid
    ctx.fillStyle = "#020617";
    ctx.fillRect(0, 0, width, height);

    ctx.strokeStyle = "rgba(30,64,175,0.35)";
    ctx.lineWidth = 1;
    const gridSpacing = 18;
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

    // layout positions
    const leftX = width * 0.12;
    const embedX = width * 0.30;
    const block1X = width * 0.50;
    const block2X = width * 0.68;
    const logitsX = width * 0.85;
    const centerY = height * 0.52;

    // token positions
    const tokenAreaHeight = height * 0.5;
    const tokenTop = centerY - tokenAreaHeight / 2;
    const tokenSpacing = tokenAreaHeight / (TOKENS.length - 1);
    const tokenPos = TOKENS.map((_, i) => ({
      x: leftX,
      y: tokenTop + i * tokenSpacing
    }));

    // heads per block
    const headsPerBlock = 4;
    const headRadius = 5;

    const blockHeight = 90;
    const block1Y = centerY - blockHeight - 12;
    const block2Y = centerY + 12;

    // helper for animated alpha (pulse travelling)
    function pulse(offset) {
      return 0.4 + 0.6 * Math.max(0, Math.sin(time * 2 + offset));
    }

    // ---- Embedding box ----
    const embedW = 70;
    const embedH = 60;
    const embedY = centerY - embedH / 2;
    drawRoundedRect(embedX - embedW / 2, embedY, embedW, embedH, 10);
    ctx.fillStyle = "rgba(15,23,42,0.9)";
    ctx.fill();
    ctx.strokeStyle = "rgba(129,140,248,0.9)";
    ctx.lineWidth = 1.5;
    ctx.stroke();

    ctx.fillStyle = "rgba(148,163,184,0.9)";
    ctx.font = "10px monospace";
    ctx.textAlign = "center";
    ctx.fillText("Emb", embedX, embedY + embedH / 2 + 3);

    // ---- Transformer blocks ----
    function drawBlock(x, y, label, sublabel) {
      const w = 110;
      const h = blockHeight;
      drawRoundedRect(x - w / 2, y, w, h, 12);
      ctx.fillStyle = "rgba(15,23,42,0.95)";
      ctx.fill();
      ctx.strokeStyle = "rgba(79,70,229,0.9)";
      ctx.lineWidth = 1.6;
      ctx.stroke();

      ctx.fillStyle = "rgba(226,232,240,0.92)";
      ctx.font = "11px monospace";
      ctx.textAlign = "left";
      ctx.fillText(label, x - w / 2 + 10, y + 18);

      ctx.fillStyle = "rgba(148,163,184,0.9)";
      ctx.font = "9px monospace";
      ctx.fillText(sublabel, x - w / 2 + 10, y + 32);
    }

    drawBlock(block1X, block1Y, "Block 1", "Multi-Head Attention");
    drawBlock(block2X, block2Y, "Block 2", "FFN + Residual");

    // ---- logits box ----
    const logitsW = 70;
    const logitsH = 80;
    const logitsY = centerY - logitsH / 2;
    drawRoundedRect(logitsX - logitsW / 2, logitsY, logitsW, logitsH, 10);
    ctx.fillStyle = "rgba(15,23,42,0.9)";
    ctx.fill();
    ctx.strokeStyle = "rgba(52,211,153,0.9)";
    ctx.lineWidth = 1.5;
    ctx.stroke();

    ctx.fillStyle = "rgba(148,163,184,0.9)";
    ctx.font = "10px monospace";
    ctx.textAlign = "center";
    ctx.fillText("Logits", logitsX, logitsY + logitsH / 2 + 3);

    // tiny bars for logits
    const barCount = 5;
    for (let i = 0; i < barCount; i++) {
      const bx = logitsX - 20 + i * 10;
      const bh = 10 + 12 * Math.abs(Math.sin(time * 1.3 + i));
      ctx.fillStyle = `rgba(52,211,153,${0.5 + 0.4 * Math.random()})`;
      ctx.fillRect(bx, logitsY + logitsH - bh - 8, 6, bh);
    }

    // ---- Draw tokens ----
    ctx.font = "12px monospace";
    ctx.textAlign = "center";
    tokenPos.forEach((p, i) => {
      const alpha = 0.8;
      ctx.fillStyle = `rgba(15,23,42,${alpha})`;
      ctx.beginPath();
      ctx.arc(p.x, p.y, 12, 0, Math.PI * 2);
      ctx.fill();
      ctx.strokeStyle = "rgba(148,163,184,0.9)";
      ctx.lineWidth = 1.2;
      ctx.stroke();

      ctx.fillStyle = "rgba(226,232,240,0.95)";
      ctx.fillText(TOKENS[i], p.x, p.y + 4);
    });

    // ---- Lines: Tokens → Embedding ----
    tokenPos.forEach((p, i) => {
      const alpha = pulse(i * 0.6);
      ctx.strokeStyle = `rgba(129,140,248,${alpha})`;
      ctx.lineWidth = 1.4;
      ctx.beginPath();
      ctx.moveTo(p.x + 12, p.y);
      ctx.lineTo(embedX - embedW / 2, lerp(p.y, centerY, 0.4));
      ctx.stroke();
    });

    // ---- Heads in Block 1 (True multi-head attention visual) ----
    const heads = [];
    const headAreaY = block1Y + 46;
    const headSpacing = 20;

    for (let h = 0; h < headsPerBlock; h++) {
      const hx = block1X - 30 + h * headSpacing;
      const hy = headAreaY;
      heads.push({ x: hx, y: hy });

      const glow = 0.3 + 0.7 * Math.max(0, Math.sin(time * 2 + h));
      ctx.beginPath();
      ctx.fillStyle = `rgba(96,165,250,${glow})`;
      ctx.arc(hx, hy, headRadius, 0, Math.PI * 2);
      ctx.fill();

      const haloRadius = headRadius + 6;
      const gradient = ctx.createRadialGradient(hx, hy, 0, hx, hy, haloRadius);
      gradient.addColorStop(0, `rgba(96,165,250,${glow * 0.5})`);
      gradient.addColorStop(1, "rgba(15,23,42,0)");
      ctx.fillStyle = gradient;
      ctx.beginPath();
      ctx.arc(hx, hy, haloRadius, 0, Math.PI * 2);
      ctx.fill();
    }

    // Tokens → Heads attention lines
    tokenPos.forEach((tp, ti) => {
      heads.forEach((h, hi) => {
        const phase = time * 2 + ti * 0.5 + hi * 0.7;
        const alpha = 0.15 + 0.6 * Math.max(0, Math.sin(phase));
        ctx.strokeStyle = `rgba(129,140,248,${alpha})`;
        ctx.lineWidth = 1;
        ctx.beginPath();
        ctx.moveTo(lerp(embedX + embedW / 2, block1X - 55, 0.7), lerp(tp.y, h.y, 0.3));
        ctx.lineTo(h.x, h.y);
        ctx.stroke();
      });
    });

    // Heads → Block 1 output (residual)
    heads.forEach((h, hi) => {
      const alpha = pulse(hi);
      ctx.strokeStyle = `rgba(94,234,212,${alpha})`;
      ctx.lineWidth = 1.2;
      ctx.beginPath();
      ctx.moveTo(h.x, h.y);
      ctx.lineTo(block1X + 45, centerY - 6);
      ctx.stroke();
    });

    // Residual stream: Block1 → Block2 → Logits
    const pts = [
      { x: embedX + embedW / 2, y: centerY },
      { x: block1X + 55, y: centerY - 6 },
      { x: block2X - 55, y: centerY + blockHeight / 2 },
      { x: block2X + 55, y: centerY + blockHeight / 2 },
      { x: logitsX - logitsW / 2, y: centerY }
    ];

    ctx.strokeStyle = "rgba(52,211,153,0.8)";
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(pts[0].x, pts[0].y);
    for (let i = 1; i < pts.length; i++) {
      ctx.lineTo(pts[i].x, pts[i].y);
    }
    ctx.stroke();

    // moving pulse on residual
    const tWrap = (Math.sin(time * 1.5) + 1) / 2; // 0..1
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

        const pr = 6;
        const grad = ctx.createRadialGradient(px, py, 0, px, py, pr * 3);
        grad.addColorStop(0, "rgba(52,211,153,0.9)");
        grad.addColorStop(1, "rgba(52,211,153,0)");
        ctx.fillStyle = grad;
        ctx.beginPath();
        ctx.arc(px, py, pr * 3, 0, Math.PI * 2);
        ctx.fill();
        break;
      }
      acc += segLen[i];
    }

    // small "input / output" labels
    ctx.font = "10px monospace";
    ctx.textAlign = "left";
    ctx.fillStyle = "rgba(148,163,184,0.9)";
    ctx.fillText("Tokens", leftX - 20, tokenTop - 12);

    ctx.textAlign = "center";
    ctx.fillText("Heads", block1X, block1Y - 8);
    ctx.fillText("Residual Stream", (block1X + block2X) / 2, centerY + blockHeight + 18);
    ctx.fillText("Next-token distribution", logitsX, logitsY - 10);

    // HUD inset
    ctx.fillStyle = "rgba(15,23,42,0.82)";
    ctx.strokeStyle = "rgba(129,140,248,0.45)";
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.roundRect(width - 170, 12, 150, 70, 10);
    ctx.fill();
    ctx.stroke();
    ctx.fillStyle = "rgba(226,232,240,0.9)";
    ctx.font = "10px monospace";
    ctx.fillText(`speed x${speed.toFixed(2)}`, width - 160, 36);
    ctx.fillStyle = "rgba(52,211,153,0.9)";
    ctx.fillText(`load ${loadSpike.toFixed(2)}`, width - 160, 52);

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
