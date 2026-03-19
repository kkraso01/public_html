(function () {
  const canvas = document.getElementById("llm-viz");
  if (!canvas) return;

  const ctx = canvas.getContext("2d");
  const theme = window.UI_THEME;
  let width = 0;
  let height = 0;
  let hoveredStage = null;
  let hoveredToken = null;
  let selectedToken = 0;
  let tokenBoxes = [];
  let stageHitboxes = [];
  let needsRender = false;

  const pad = 16;
  const tokens = [
    "<bos>",
    "question",
    "retrieval",
    "router",
    "context",
    "evidence",
    "policy",
    "reasoning",
    "draft",
    "logits",
    "answer",
    "<eos>"
  ];

  const stages = [
    {
      id: "embed",
      label: "Embedding",
      dims: "seq × d_model",
      shape: "12 × 768",
      desc: "Token + position vectors"
    },
    {
      id: "qkv",
      label: "Q / K / V",
      dims: "3 × (seq × 64)",
      shape: "12 × 64 per head",
      desc: "Project to queries, keys, values"
    },
    {
      id: "attn",
      label: "Attention",
      dims: "heads=8, seq × seq",
      shape: "12 × 12 weights",
      desc: "Softmax weights across sequence",
      isAttention: true
    },
    {
      id: "res",
      label: "Residual",
      dims: "skip + layer norm",
      shape: "12 × 768",
      desc: "Adds skip path + normalize"
    },
    {
      id: "ffn",
      label: "FFN",
      dims: "GELU(3072) → 768",
      shape: "12 × 3072 → 12 × 768",
      desc: "Two-layer MLP with GELU"
    },
    {
      id: "logits",
      label: "Logits",
      dims: "seq × vocab",
      shape: "12 × 50k",
      desc: "Project to vocabulary scores"
    }
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
      accent: styles.getPropertyValue("--accent").trim()
    };
  }

  function setCanvasSize() {
    const rect = canvas.getBoundingClientRect();
    const dpr = window.devicePixelRatio || 1;
    canvas.width = Math.round(rect.width * dpr);
    canvas.height = Math.round(rect.height * dpr);
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    width = rect.width;
    height = rect.height;
  }

  function resize() {
    setCanvasSize();
    scheduleDraw();
  }

  window.addEventListener("resize", resize);
  window.addEventListener("themechange", resize);
  resize();

  const safe = () => ({
    left: pad,
    right: Math.max(pad, width - pad),
    top: pad,
    bottom: Math.max(pad, height - pad)
  });

  function clamp(val, min, max) {
    return Math.max(min, Math.min(max, val));
  }

  function drawWrappedText(text, x, y, maxWidth, lineHeight, maxLines) {
    const words = text.split(/\s+/);
    let line = "";
    let lineCount = 0;
    for (let i = 0; i < words.length; i += 1) {
      const testLine = line ? `${line} ${words[i]}` : words[i];
      const { width: tw } = ctx.measureText(testLine);
      if (tw > maxWidth && line) {
        ctx.fillText(line, x, y + lineCount * lineHeight);
        lineCount += 1;
        if (lineCount >= maxLines) return;
        line = words[i];
      } else {
        line = testLine;
      }
    }
    if (lineCount < maxLines) {
      ctx.fillText(line, x, y + lineCount * lineHeight);
    }
    return Math.min(maxLines, lineCount + 1);
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
  }

  function background(colors) {
    ctx.fillStyle = colors.surface;
    ctx.fillRect(0, 0, width, height);
    if (theme) {
      theme.drawGrid(ctx, 22, colors.border, theme.isDark() ? 0.12 : 0.08, width, height);
    }
  }

  function computeAttention() {
    const size = tokens.length;
    const matrix = Array.from({ length: size }, () => new Array(size).fill(0));
    for (let q = 0; q < size; q += 1) {
      let sum = 0;
      for (let k = 0; k < size; k += 1) {
        const proximity = Math.exp(-Math.abs(q - k) * 0.35);
        const tokenBias = tokens[k].length * 0.04;
        const contextBoost = tokens[k] === "context" ? 0.35 : 0;
        const answerBoost = tokens[k] === "answer" ? 0.25 : 0;
        const val = proximity + tokenBias + contextBoost + answerBoost;
        matrix[q][k] = val;
        sum += val;
      }
      for (let k = 0; k < size; k += 1) {
        matrix[q][k] = matrix[q][k] / sum;
      }
    }
    return matrix;
  }

  const attention = computeAttention();

  function topKAttention(queryIndex, k = 3) {
    const weights = attention[queryIndex] || [];
    const entries = weights.map((w, idx) => ({ token: tokens[idx], weight: w }));
    entries.sort((a, b) => b.weight - a.weight);
    return entries.slice(0, k);
  }

  function drawTokens(colors, fade) {
    const s = safe();
    const compact = width < 680;
    const headerY = s.top + 10;
    const stripY = headerY + 18;
    const padX = compact ? 8 : 12;
    const gap = compact ? 6 : 8;
    tokenBoxes = [];
    ctx.font = compact
      ? "600 10px ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace"
      : "600 11px ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace";
    ctx.textBaseline = "middle";

    const chips = tokens.map((tok) => {
      const label = tok.toUpperCase();
      const w = ctx.measureText(label).width + padX * 2;
      const h = compact ? 20 : 24;
      return { label, w, h };
    });

    const availW = s.right - s.left;
    const totalW = chips.reduce((acc, c) => acc + c.w, 0) + gap * (chips.length - 1);
    const twoRows = totalW > availW && height > 220;
    let cursorX = s.left;
    let cursorY = stripY;
    let maxRowHeight = 0;

    chips.forEach((chip, idx) => {
      if (cursorX + chip.w > s.right && twoRows) {
        cursorX = s.left;
        cursorY += maxRowHeight + 6;
        maxRowHeight = 0;
      }
      const isSelected = idx === selectedToken;
      tokenBoxes.push({ x: cursorX, y: cursorY - chip.h / 2, w: chip.w, h: chip.h, idx });

      ctx.globalAlpha = fade;
      ctx.fillStyle = isSelected
        ? (theme ? theme.rgba(colors.accent, 0.16) : colors.surfaceElevated)
        : colors.surfaceElevated;
      ctx.strokeStyle = isSelected ? colors.accent : colors.border;
      ctx.lineWidth = 1;
      ctx.beginPath();
      drawRoundedRect(cursorX, cursorY - chip.h / 2, chip.w, chip.h, 3);
      ctx.fill();
      ctx.stroke();

      ctx.fillStyle = isSelected ? colors.textStrong : colors.muted;
      ctx.textAlign = "center";
      ctx.fillText(chip.label, cursorX + chip.w / 2, cursorY);
      ctx.globalAlpha = 1;

      maxRowHeight = Math.max(maxRowHeight, chip.h);
      cursorX += chip.w + gap;
    });

    ctx.fillStyle = colors.muted;
    ctx.textAlign = "left";
    ctx.font = compact
      ? "600 10px ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace"
      : "600 11px ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace";
    ctx.fillText("Trace token", s.left, headerY);

    const bottomY = cursorY + maxRowHeight / 2;
    return { bottomY };
  }

  function drawPipeline(colors, activeStageId, topY, fade) {
    const s = safe();
    const compact = width < 760;

    let inspectorWidth = clamp(width * 0.28, 200, 280);
    let inspectorX = s.right - inspectorWidth;
    let pipeAreaLeft = s.left;
    let pipeAreaRight = inspectorX - pad;
    let stacked = false;

    if (pipeAreaRight - pipeAreaLeft < 420) {
      stacked = true;
      inspectorX = s.left;
      inspectorWidth = s.right - s.left;
      pipeAreaLeft = s.left;
      pipeAreaRight = s.right;
    }

    const areaWidth = pipeAreaRight - pipeAreaLeft;
    const nominalBox = compact ? 110 : 140;
    const minBox = compact ? 72 : 90;
    const boxWidth = clamp(areaWidth / stages.length - 12, minBox, nominalBox);
    const pipeBottomLimit = stacked ? s.bottom - 88 : s.bottom - 2;
    const availableHeight = Math.max(0, pipeBottomLimit - topY);
    const desiredBoxH = compact ? 120 : 130;
    const minBoxH = compact ? 96 : 106;
    const boxHeight = Math.min(desiredBoxH, Math.max(minBoxH, availableHeight));
    const centerStart = pipeAreaLeft + boxWidth / 2;
    const centerEnd = pipeAreaRight - boxWidth / 2;
    const xStep = Math.max(0, (centerEnd - centerStart) / Math.max(1, stages.length - 1));
    const yTop = topY;

    stageHitboxes = [];

    ctx.strokeStyle = theme ? theme.rgba(colors.border, 0.65) : colors.border;
    ctx.lineWidth = 2;
    ctx.lineCap = "round";
    const lineStartX = centerStart;
    const lineEndX = centerStart + xStep * (stages.length - 1);
    ctx.beginPath();
    ctx.moveTo(lineStartX, yTop + boxHeight / 2);
    ctx.lineTo(lineEndX, yTop + boxHeight / 2);
    ctx.stroke();

    for (let i = 0; i < stages.length - 1; i += 1) {
      const arrowX = centerStart + xStep * i + xStep / 2;
      const arrowY = yTop + boxHeight / 2;
      ctx.beginPath();
      ctx.moveTo(arrowX - 6, arrowY - 6);
      ctx.lineTo(arrowX + 6, arrowY);
      ctx.lineTo(arrowX - 6, arrowY + 6);
      ctx.closePath();
      ctx.fillStyle = colors.border;
      ctx.fill();
    }

    const isAttentionHero = activeStageId === "attn";

    stages.forEach((stage, idx) => {
      const x = centerStart + xStep * idx - boxWidth / 2;
      const y = yTop;
      const isHover = hoveredStage && hoveredStage.id === stage.id;
      const isAttention = stage.isAttention;

      if (isAttentionHero && !isAttention) {
        ctx.save();
        ctx.globalAlpha = 0.35;
      }

      stageHitboxes.push({ ...stage, x, y, w: boxWidth, h: boxHeight });

      ctx.globalAlpha = fade;
      ctx.fillStyle = isHover
        ? (theme ? theme.rgba(colors.accent, 0.14) : colors.surfaceElevated)
        : colors.surfaceElevated;
      ctx.strokeStyle = isHover ? colors.accent : colors.border;
      ctx.lineWidth = 1;
      ctx.beginPath();
      drawRoundedRect(x, y, boxWidth, boxHeight, 4);
      ctx.fill();
      ctx.stroke();

      ctx.fillStyle = colors.textStrong;
      ctx.font = "700 12px ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace";
      ctx.textAlign = "center";
      ctx.fillText(stage.label.toUpperCase(), x + boxWidth / 2, y + 16);

      const textPad = 8;
      ctx.fillStyle = colors.muted;
      ctx.font = "500 10px ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace";
      ctx.textAlign = "left";
      const descLines = drawWrappedText(stage.desc || "", x + textPad, y + 30, Math.max(60, boxWidth - textPad * 2 - 8), 11, 3) || 0;
      const descBlock = Math.max(0, descLines * 11 + 4);

      if (isAttention) {
        const matrixSize = Math.min(6, tokens.length);
        const baseTop = compact ? 48 : 56;
        const descPad = Math.min(22, descBlock);
        const topBlock = Math.min(boxHeight - 32, baseTop + descPad);
        const bottomPad = 10;
        const availW = boxWidth - textPad * 2;
        const availH = Math.max(28, boxHeight - topBlock - bottomPad);
        const cell = Math.max(4, Math.min(14, Math.min(availW / matrixSize, availH / matrixSize)));
        const gridWidth = matrixSize * cell;
        const gridHeight = matrixSize * cell;
        const gridX = x + (boxWidth - gridWidth) / 2;
        const gridY = y + topBlock + Math.max(0, (availH - gridHeight) / 2);
        const isDark = theme ? theme.isDark() : document.documentElement.dataset.theme === "dark";
        const selectedRow = selectedToken % matrixSize;
        const rowWeights = attention[selectedRow] || [];
        const strongest = rowWeights.slice(0, matrixSize).reduce(
          (best, val, idx) => (val > best.value ? { idx, value: val } : best),
          { idx: 0, value: -1 }
        );

        ctx.fillStyle = theme ? theme.rgba(colors.accent, 0.1) : "rgba(255, 45, 45, 0.08)";
        ctx.fillRect(gridX, gridY + selectedRow * cell, gridWidth, cell);

        for (let r = 0; r < matrixSize; r += 1) {
          for (let c = 0; c < matrixSize; c += 1) {
            const v = attention[r][c];
            const base = isDark ? 30 : 220;
            const range = isDark ? 180 : -160;
            const shade = Math.max(0, Math.min(255, Math.floor(base + v * range)));
            ctx.fillStyle = `rgb(${shade},${shade},${shade})`;
            ctx.fillRect(gridX + c * cell, gridY + r * cell, cell - 1, cell - 1);
          }
        }

        ctx.strokeStyle = colors.accent;
        ctx.lineWidth = 1.2;
        ctx.strokeRect(gridX + strongest.idx * cell - 0.5, gridY + selectedRow * cell - 0.5, cell + 1, cell + 1);
      }

      ctx.globalAlpha = 1;

      if (isAttentionHero && !isAttention) {
        ctx.restore();
      }
    });

    return { inspectorX, inspectorWidth, stacked, yTop, boxHeight };
  }

  function drawInspector(colors, layout, fade) {
    const { inspectorX, inspectorWidth, stacked, yTop, boxHeight } = layout;
    const s = safe();
    const panelX = inspectorX;
    const stackedY = stacked ? yTop + boxHeight + 22 : s.top + 110;
    const panelY = Math.max(stackedY, s.top + 8);
    const panelW = Math.max(140, inspectorWidth - 24);
    const availableH = s.bottom - panelY - 8;
    const panelH = Math.max(120, Math.min(height * 0.52, availableH));
    const stage = hoveredStage || stages.find((s) => s.id === "attn") || stages[0];

    ctx.globalAlpha = fade;
    ctx.fillStyle = colors.surfaceElevated;
    ctx.strokeStyle = theme ? theme.rgba(colors.border, 0.8) : colors.border;
    ctx.lineWidth = 1;
    ctx.beginPath();
    drawRoundedRect(panelX, panelY, panelW, panelH, 4);
    ctx.fill();
    ctx.stroke();

    ctx.fillStyle = colors.muted;
    ctx.font = "600 11px ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace";
    ctx.textAlign = "left";
    ctx.fillText("Inspector", panelX + 12, panelY + 16);

    ctx.fillStyle = colors.textStrong;
    ctx.font = "700 14px ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace";
    ctx.fillText(stage.label.toUpperCase(), panelX + 12, panelY + 36);

    ctx.fillStyle = colors.muted;
    ctx.font = "600 11px ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace";
    ctx.fillText(`Stage: ${stage.label}`, panelX + 12, panelY + 56);
    ctx.fillText(`Shape: ${stage.shape}`, panelX + 12, panelY + 72);
    ctx.fillText(`Flow: ${stage.dims}`, panelX + 12, panelY + 88);

    let cursorY = panelY + 112;

    if (stage.isAttention) {
      ctx.fillStyle = colors.textStrong;
      ctx.font = "700 12px ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace";
      ctx.fillText("Top-3 attended", panelX + 12, cursorY);
      cursorY += 18;

      const tops = topKAttention(selectedToken, 3);
      ctx.fillStyle = colors.muted;
      ctx.font = "600 11px ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace";
      tops.forEach((item, idx) => {
        const weight = (item.weight * 100).toFixed(1);
        ctx.fillText(`${idx + 1}. ${item.token} (${weight}%)`, panelX + 12, cursorY + idx * 16);
      });
      cursorY += tops.length * 16;

      const lead = tops[0];
      if (lead) {
        ctx.fillStyle = colors.textStrong;
        ctx.font = "600 11px ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace";
        const summary = `${tokens[selectedToken].toUpperCase()} pulls ${lead.token.toUpperCase()} (${(lead.weight * 100).toFixed(0)}%)`;
        const lines = drawWrappedText(summary, panelX + 12, cursorY + 8, panelW - 24, 12, 2) || 0;
        cursorY += 8 + lines * 12;
      }
    } else {
      ctx.fillStyle = colors.textStrong;
      ctx.font = "700 12px ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace";
      ctx.fillText("Details", panelX + 12, panelY + 112);

      ctx.fillStyle = colors.muted;
      ctx.font = "600 11px ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace";
      ctx.fillText("LayerNorm ✓", panelX + 12, panelY + 132);
      ctx.fillText("Batch size: 1", panelX + 12, panelY + 150);
      ctx.fillText(`Sequence: ${tokens.length} tokens`, panelX + 12, panelY + 168);
    }

    const traceY = Math.max(panelY + panelH - 58, cursorY + 16);
    ctx.fillStyle = colors.textStrong;
    ctx.font = "700 12px ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace";
    ctx.fillText("Trace token", panelX + 12, traceY);

    ctx.fillStyle = colors.muted;
    ctx.font = "600 11px ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace";
    ctx.fillText(tokens[selectedToken].toUpperCase(), panelX + 12, traceY + 18);
    ctx.globalAlpha = 1;
  }

  function drawTrace(colors) {
    const stage = hoveredStage || stages.find((s) => s.id === "attn") || stages[0];
    if (!stageHitboxes.length) return;
    const target = stageHitboxes.find((b) => b.id === stage.id) || stageHitboxes[0];
    const tokenBox = tokenBoxes[selectedToken];
    if (!tokenBox || !target) return;

    const startX = tokenBox.x + tokenBox.w / 2;
    const startY = tokenBox.y + tokenBox.h;
    const endX = target.x + target.w / 2;
    const endY = target.y;

    ctx.strokeStyle = colors.accent;
    ctx.lineWidth = 2;
    ctx.setLineDash([6, 6]);
    ctx.beginPath();
    ctx.moveTo(startX, startY + 6);
    ctx.lineTo(endX, endY - 8);
    ctx.stroke();
    ctx.setLineDash([]);

    ctx.fillStyle = colors.accent;
    ctx.beginPath();
    ctx.arc(endX, endY - 8, 4, 0, Math.PI * 2);
    ctx.fill();
  }

  function draw() {
    const colors = palette();
    const introProgress = mode === MODE.INTRO ? clamp(demoElapsed / INTRO_DURATION, 0, 1) : 1;
    ctx.save();
    ctx.beginPath();
    ctx.rect(0, 0, width, height);
    ctx.clip();

    ctx.clearRect(0, 0, width, height);
    background(colors);
    const tokenLayout = drawTokens(colors, introProgress);
    const activeStage = hoveredStage
      ? hoveredStage.id
      : stages[demoStageIndex]?.id || stages.find((s) => s.id === "attn")?.id || stages[0].id;
    const layout = drawPipeline(colors, activeStage, tokenLayout.bottomY + 18, introProgress);
    drawTrace(colors);
    drawInspector(colors, layout, introProgress);

    ctx.fillStyle = colors.muted;
    ctx.font = "600 10px ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace";
    const stageLabel = stages.find((s) => s.id === activeStage)?.label || "";
    const stageIndex = stages.findIndex((s) => s.id === activeStage) + 1;
    ctx.fillText(`Mode: ${mode} · Stage ${stageIndex}/${stages.length} (${stageLabel})`, 12, height - 14);

    ctx.restore();
  }

  const MODE = { IDLE: "IDLE", INTRO: "INTRO", DEMO: "DEMO", HOLD: "HOLD" };
  let mode = MODE.IDLE;
  let rafId = null;
  let lastTs = 0;
  let demoElapsed = 0;
  let demoStageIndex = 0;
  const INTRO_DURATION = 1.0;

  const controlsRoot = canvas.closest(".ui-canvas-frame")?.querySelector(".ui-canvas-title") || null;
  const playBtn = controlsRoot?.querySelector('[data-llmctl="play"]') || null;
  const stepBtn = controlsRoot?.querySelector('[data-llmctl="step"]') || null;
  const resetBtn = controlsRoot?.querySelector('[data-llmctl="reset"]') || null;

  function setMode(next) {
    mode = next;
    if (playBtn) {
      playBtn.textContent = mode === MODE.DEMO || mode === MODE.INTRO ? "Pause" : "Play";
    }
  }

  function resetDemo() {
    demoElapsed = 0;
    demoStageIndex = 0;
    hoveredStage = null;
    hoveredToken = null;
    setMode(MODE.INTRO);
    scheduleDraw();
  }

  function stepDemo() {
    demoStageIndex = (demoStageIndex + 1) % stages.length;
    hoveredStage = stages[demoStageIndex];
    setMode(MODE.HOLD);
    stopRunning(false);
    scheduleDraw();
  }

  function startRunning() {
    if (rafId != null) return;
    lastTs = 0;
    rafId = requestAnimationFrame(tick);
  }

  function stopRunning(setIdle) {
    if (rafId == null) return;
    cancelAnimationFrame(rafId);
    rafId = null;
    lastTs = 0;
    if (setIdle) {
      setMode(MODE.IDLE);
    }
  }

  function scheduleDraw() {
    if (needsRender) return;
    needsRender = true;
    requestAnimationFrame(() => {
      needsRender = false;
      draw();
    });
  }

  function tick(ts) {
    if (rafId == null) return;
    if (!lastTs) lastTs = ts;
    const dt = Math.min(0.05, (ts - lastTs) / 1000);
    lastTs = ts;

    const reduceMotion = window.matchMedia?.("(prefers-reduced-motion: reduce)")?.matches;

    if (mode === MODE.INTRO) {
      demoElapsed += dt;
      hoveredStage = stages[0];
      scheduleDraw();

      if (demoElapsed >= INTRO_DURATION || reduceMotion) {
        demoElapsed = 0;
        demoStageIndex = 0;
        if (reduceMotion) {
          setMode(MODE.HOLD);
          stopRunning(false);
          scheduleDraw();
          return;
        }
        setMode(MODE.DEMO);
      }
    } else if (mode === MODE.DEMO) {
      demoElapsed += dt;
      const STEP_SECONDS = 1.0;
      if (demoElapsed >= STEP_SECONDS) {
        demoElapsed = 0;
        demoStageIndex += 1;

        if (demoStageIndex >= stages.length) {
          demoStageIndex = stages.findIndex((s) => s.id === "attn");
          hoveredStage = stages[demoStageIndex] || stages[0];
          setMode(MODE.HOLD);
          stopRunning(false);
          scheduleDraw();
          return;
        }
        hoveredStage = stages[demoStageIndex];
      }

      draw();
    }

    if (mode === MODE.INTRO || mode === MODE.DEMO) {
      rafId = requestAnimationFrame(tick);
    }
  }

  function tokenHit(x, y) {
    return tokenBoxes.find((b) => x >= b.x && x <= b.x + b.w && y >= b.y && y <= b.y + b.h);
  }

  function stageHit(x, y) {
    return stageHitboxes.find((b) => x >= b.x && x <= b.x + b.w && y >= b.y && y <= b.y + b.h);
  }

  const control = {
    isRunning: false,
    onEnter: () => {
      control.isRunning = true;
      const reduceMotion = window.matchMedia?.("(prefers-reduced-motion: reduce)")?.matches;
      resetDemo();
      if (!reduceMotion) {
        startRunning();
      } else {
        setMode(MODE.HOLD);
        scheduleDraw();
      }
    },
    onExit: () => {
      control.isRunning = false;
      stopRunning(true);
      scheduleDraw();
    }
  };

  if (window.ViewportObserver && typeof window.ViewportObserver.observe === "function") {
    window.ViewportObserver.observe(canvas, control, 0.1);
  } else {
    control.onEnter();
  }

  const observerFallback = setInterval(() => {
    if (control.isRunning && rafId == null && mode !== MODE.HOLD) {
      startRunning();
    }
    if (!control.isRunning && rafId != null) {
      stopRunning(true);
      scheduleDraw();
    }
  }, 1200);

  if (playBtn) {
    playBtn.addEventListener("click", () => {
      if (mode === MODE.DEMO || mode === MODE.INTRO) {
        setMode(MODE.HOLD);
        stopRunning(false);
        scheduleDraw();
        return;
      }
      resetDemo();
      startRunning();
    });
  }

  if (stepBtn) stepBtn.addEventListener("click", stepDemo);
  if (resetBtn)
    resetBtn.addEventListener("click", () => {
      resetDemo();
      startRunning();
    });

  canvas.addEventListener("mousemove", (event) => {
    const rect = canvas.getBoundingClientRect();
    const x = event.clientX - rect.left;
    const y = event.clientY - rect.top;
    hoveredStage = stageHit(x, y) || null;
    hoveredToken = tokenHit(x, y) || null;
    scheduleDraw();
  });

  canvas.addEventListener("mouseleave", () => {
    hoveredStage = null;
    hoveredToken = null;
    scheduleDraw();
  });

  canvas.addEventListener("click", (event) => {
    const rect = canvas.getBoundingClientRect();
    const x = event.clientX - rect.left;
    const y = event.clientY - rect.top;
    const hit = tokenHit(x, y);
    if (hit) {
      selectedToken = hit.idx;
      scheduleDraw();
      return;
    }
    const stage = stageHit(x, y);
    if (stage) {
      hoveredStage = stage;
      scheduleDraw();
    }
  });

  window.addEventListener("beforeunload", () => clearInterval(observerFallback));

  scheduleDraw();
})();
