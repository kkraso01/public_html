(function () {
  const canvas = document.getElementById("attentionCanvas");
  if (!canvas) {
    console.warn("[Attention] canvas not found");
    return;
  }

  const ctx = canvas.getContext("2d");
  const theme = window.UI_THEME;
  let width = 0;
  let height = 0;
  let hoverCell = null;
  let hoverRow = null;
  let selectedRow = null;
  let selectedHead = 0;
  let needsRender = false;
  let lastLayout = null;
  let rafId = null;
  let lastTs = 0;
  let demoElapsed = 0;
  let mode = "IDLE";
  let pendingPlay = false;

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
  const size = tokens.length;

  const heads = [
    { name: "Self" },
    { name: "Prev" },
    { name: "Next" },
    { name: "Context" },
    { name: "Keyword" },
    { name: "Positional" },
    { name: "Summary" },
    { name: "Mixed" }
  ];

  function displayToken(tok) {
    if (tok.length <= 8) return tok;
    return `${tok.slice(0, 5)}…`;
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

  resize();
  window.addEventListener("resize", resize);
  window.addEventListener("themechange", resize);

  const safe = () => ({
    left: pad,
    right: Math.max(pad, width - pad),
    top: pad,
    bottom: Math.max(pad, height - pad)
  });

  function clamp(val, min, max) {
    return Math.max(min, Math.min(max, val));
  }

  function drawRoundedRect(x, y, widthValue, heightValue, radius) {
    if (ctx.roundRect) {
      ctx.roundRect(x, y, widthValue, heightValue, radius);
      return;
    }

    const r = Math.min(radius, widthValue / 2, heightValue / 2);
    ctx.beginPath();
    ctx.moveTo(x + r, y);
    ctx.lineTo(x + widthValue - r, y);
    ctx.quadraticCurveTo(x + widthValue, y, x + widthValue, y + r);
    ctx.lineTo(x + widthValue, y + heightValue - r);
    ctx.quadraticCurveTo(x + widthValue, y + heightValue, x + widthValue - r, y + heightValue);
    ctx.lineTo(x + r, y + heightValue);
    ctx.quadraticCurveTo(x, y + heightValue, x, y + heightValue - r);
    ctx.lineTo(x, y + r);
    ctx.quadraticCurveTo(x, y, x + r, y);
  }

  function normalizeRow(row) {
    const sum = row.reduce((acc, val) => acc + val, 0) || 1;
    return row.map((val) => val / sum);
  }

  function buildHeadMatrices() {
    const contextIndex = tokens.indexOf("context");
    const lastIndex = size - 1;
    const lengths = tokens.map((t) => Math.max(1, t.replace(/[<>]/g, "").length));
    const matrices = [];

    for (let h = 0; h < heads.length; h += 1) {
      const matrix = Array.from({ length: size }, () => new Array(size).fill(0));
      for (let q = 0; q < size; q += 1) {
        const row = new Array(size).fill(0.02);
        if (h === 0) {
          for (let k = 0; k < size; k += 1) {
            row[k] += Math.exp(-Math.abs(q - k) * 1.6);
          }
        } else if (h === 1) {
          const prev = Math.max(0, q - 1);
          row[prev] += 1.2;
        } else if (h === 2) {
          const next = Math.min(size - 1, q + 1);
          row[next] += 1.2;
        } else if (h === 3) {
          const sink = contextIndex >= 0 ? contextIndex : 0;
          row[sink] += 1.6;
        } else if (h === 4) {
          for (let k = 0; k < size; k += 1) {
            row[k] += Math.pow(lengths[k], 1.2) * 0.08;
          }
        } else if (h === 5) {
          for (let k = 0; k < size; k += 1) {
            row[k] += 1 / (1 + k * 0.6);
          }
        } else if (h === 6) {
          row[0] += 1.1;
          row[lastIndex] += 0.7;
        } else if (h === 7) {
          const next = Math.min(size - 1, q + 1);
          const sink = contextIndex >= 0 ? contextIndex : 0;
          row[q] += 0.7;
          row[next] += 0.5;
          row[sink] += 0.6;
        }

        matrix[q] = normalizeRow(row);
      }
      matrices.push(matrix);
    }

    return matrices;
  }

  const headMatrices = buildHeadMatrices();

  function topLinks(rowIndex, headIndex, k = 5) {
    if (rowIndex == null) return [];
    const weights = headMatrices[headIndex][rowIndex] || [];
    const entries = weights.map((w, idx) => ({ token: tokens[idx], weight: w }));
    entries.sort((a, b) => b.weight - a.weight);
    return entries.slice(0, k);
  }

  function computeLayout() {
    const s = safe();

    const panelWidth = clamp(width * 0.24, 160, 210);
    const panelHeight = clamp(height * 0.28, 140, 220);
    const hasSidePanel = s.right - s.left - panelWidth - pad >= 360;
    const stacked = !hasSidePanel;

    const gridAreaWidth = stacked ? s.right - s.left : s.right - s.left - panelWidth - pad;
    const gridAreaHeight = stacked ? s.bottom - s.top - panelHeight - pad : s.bottom - s.top;

    ctx.font = "500 10px ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace";
    const labelTokens = tokens.map((t) => displayToken(t));
    const maxLeftLabelW = Math.min(
      140,
      Math.max(64, Math.ceil(Math.max(...labelTokens.map((t) => ctx.measureText(t).width)) + 10))
    );

    const topLabelH = 22;
    const titleH = 26;

    const gridX = s.left + maxLeftLabelW + 8;
    const gridY = s.top + titleH + topLabelH + 6;

    const rawGridW = Math.max(180, gridAreaWidth - (gridX - s.left) - 6);
    const rawGridH = Math.max(160, gridAreaHeight - (gridY - s.top) - 6);

    const cellW = Math.max(22, rawGridW / size);
    const cellH = Math.max(16, rawGridH / size);
    const gridW = cellW * size;
    const gridH = cellH * size;

    const panelX = stacked ? s.left : s.left + gridAreaWidth + pad;
    const panelY = stacked ? gridY + gridH + 10 : gridY;
    const panelW = stacked ? s.right - s.left : panelWidth;
    const panelH = stacked ? s.bottom - panelY : gridH;

    return {
      gridX,
      gridY,
      gridW,
      gridH,
      cellW,
      cellH,
      leftLabelW: maxLeftLabelW,
      topLabelH,
      panelX,
      panelY,
      panelW,
      panelH,
      stacked,
      bounds: s
    };
  }

  function drawHeadSelector(colors, layout) {
    return { headBoxes: [], selectorHeight: 0 };
  }

  function drawMatrix(colors, layout) {
    const { gridX, gridY, gridW, gridH, cellW, cellH, leftLabelW } = layout;

    ctx.fillStyle = colors.surface;
    ctx.fillRect(0, 0, width, height);
    if (theme) theme.drawGrid(ctx, 22, colors.border, theme.isDark() ? 0.08 : 0.05, width, height);

    ctx.font = "700 11px ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace";
    ctx.fillStyle = colors.textStrong;
    ctx.textAlign = "left";
    ctx.fillText("Semantic Attention Map", gridX - 4, gridY - 22);

    const matrix = headMatrices[selectedHead];
    for (let row = 0; row < size; row += 1) {
      for (let col = 0; col < size; col += 1) {
        const value = matrix[row][col];
        const lightness = Math.pow(clamp(value, 0, 1), 0.7);
        const shade = Math.round(40 + 215 * lightness);
        ctx.fillStyle = `rgb(${shade},${shade},${shade})`;
        ctx.fillRect(gridX + col * cellW, gridY + row * cellH, cellW - 1, cellH - 1);
      }
    }

    if (selectedRow != null) {
      ctx.fillStyle = theme ? theme.rgba(colors.accent, 0.08) : colors.surface;
      ctx.fillRect(gridX, gridY + selectedRow * cellH, gridW, cellH);
    }

    if (hoverRow != null) {
      ctx.fillStyle = theme ? theme.rgba(colors.accent, 0.08) : colors.surface;
      ctx.fillRect(gridX, gridY + hoverRow * cellH, gridW, cellH);
    }

    ctx.save();
    ctx.font = "500 10px ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace";
    ctx.fillStyle = colors.muted;
    ctx.textBaseline = "middle";

    tokens.forEach((tok, i) => {
      const label = displayToken(tok);

      const xTop = gridX + i * cellW + cellW / 2;
      const yTop = gridY - 8;
      ctx.textAlign = "center";
      ctx.fillText(label, xTop, yTop);

      const xLeft = gridX - 8;
      const yLeft = gridY + i * cellH + cellH / 2;
      ctx.textAlign = "right";
      ctx.fillText(label, xLeft, yLeft);
    });

    ctx.restore();

    if (hoverCell) {
      ctx.strokeStyle = colors.accent;
      ctx.lineWidth = 1;
      ctx.strokeRect(gridX + hoverCell.col * cellW, gridY + hoverCell.row * cellH, cellW, cellH);
      ctx.setLineDash([4, 4]);
      ctx.strokeRect(gridX, gridY + hoverCell.row * cellH, gridW, cellH);
      ctx.strokeRect(gridX + hoverCell.col * cellW, gridY, cellW, gridH);
      ctx.setLineDash([]);
    }
  }

  function drawLinks(colors, layout) {
    const panelX = layout.panelX;
    const panelY = layout.panelY;
    const panelW = layout.panelW;
    const panelH = layout.panelH;

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
    ctx.fillText("Top-5 attended", panelX + 10, panelY + 16);

    const links = topLinks(selectedRow ?? 0, selectedHead, 5);
    ctx.fillStyle = colors.textStrong;
    ctx.font = "700 12px ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace";
    const title = selectedRow != null ? tokens[selectedRow].toUpperCase() : "CLICK ROW";
    ctx.fillText(title, panelX + 10, panelY + 34);

    ctx.fillStyle = colors.muted;
    ctx.font = "600 11px ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace";
    links.forEach((item, idx) => {
      const weight = (item.weight * 100).toFixed(1);
      const y = panelY + 56 + idx * 18;
      if (y < panelY + panelH - 10) {
        ctx.fillText(`${idx + 1}. ${item.token} (${weight}%)`, panelX + 10, y);
      }
    });

    const legendWidth = 56;
    const legendSteps = 6;
    const legendBlockW = 7;
    const legendGap = 2;
    const legendX = panelX + panelW - legendWidth - 10;
    const contentBottom = panelY + 56 + links.length * 18;
    const legendY = Math.min(panelY + panelH - 20, contentBottom + 12);
    if (legendX > panelX + 8 && legendY > panelY + 12) {
      ctx.fillStyle = colors.muted;
      ctx.textAlign = "left";
      ctx.font = "600 10px ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace";
      ctx.fillText("Weight", legendX, legendY - 6);
      for (let i = 0; i < legendSteps; i += 1) {
        const val = i / (legendSteps - 1);
        const shade = Math.floor(255 * val);
        ctx.fillStyle = `rgb(${shade},${shade},${shade})`;
        ctx.fillRect(legendX + i * (legendBlockW + legendGap), legendY, legendBlockW, 10);
      }
      ctx.fillStyle = colors.muted;
      ctx.fillText("0", legendX, legendY + 18);
      ctx.fillText("1", legendX + (legendSteps - 1) * (legendBlockW + legendGap), legendY + 18);
    }
  }

  function drawTooltip(colors, layout) {
    if (!hoverCell) return;
    const { gridX, gridY, cellW, cellH } = layout;
    const matrix = headMatrices[selectedHead];
    const weight = matrix[hoverCell.row][hoverCell.col];
    const text = `${tokens[hoverCell.row]} → ${tokens[hoverCell.col]} (${weight.toFixed(3)})`;

    ctx.font = "600 11px ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace";
    const w = ctx.measureText(text).width + 12;
    const h = 24;
    let x = gridX + hoverCell.col * cellW + cellW + 8;
    let y = gridY + hoverCell.row * cellH + cellH / 2 - 12;

    if (x + w > width - pad) x = width - pad - w;
    if (x < pad) x = pad;
    if (y + h > height - pad) y = height - pad - h;
    if (y < pad) y = pad;

    ctx.fillStyle = colors.surfaceElevated;
    ctx.strokeStyle = theme ? theme.rgba(colors.border, 0.9) : colors.border;
    ctx.lineWidth = 1;
    ctx.beginPath();
    drawRoundedRect(x, y, w, h, 3);
    ctx.fill();
    ctx.stroke();

    ctx.fillStyle = colors.textStrong;
    ctx.textAlign = "left";
    ctx.textBaseline = "middle";
    ctx.fillText(text, x + 6, y + h / 2);
  }

  function draw() {
    const colors = palette();
    lastLayout = computeLayout();

    ctx.save();
    ctx.beginPath();
    ctx.rect(0, 0, width, height);
    ctx.clip();

    drawMatrix(colors, lastLayout);
    const selector = drawHeadSelector(colors, lastLayout);
    lastLayout.headBoxes = selector.headBoxes;
    drawLinks(colors, lastLayout);
    drawTooltip(colors, lastLayout);

    ctx.restore();
  }

  function scheduleDraw() {
    if (needsRender) return;
    needsRender = true;
    requestAnimationFrame(() => {
      needsRender = false;
      draw();
    });
  }

  function cellFromPoint(x, y) {
    if (!lastLayout) return null;
    const { gridX, gridY, gridW, gridH, cellW, cellH } = lastLayout;
    if (x < gridX || y < gridY || x > gridX + gridW || y > gridY + gridH) return null;

    const col = Math.floor((x - gridX) / cellW);
    const row = Math.floor((y - gridY) / cellH);

    if (col >= 0 && col < size && row >= 0 && row < size) return { row, col };
    return null;
  }

  function rowLabelFromPoint(x, y) {
    if (!lastLayout) return null;
    const { gridX, gridY, gridH, cellH, leftLabelW } = lastLayout;

    const labelAreaX0 = gridX - (leftLabelW + 16);
    const labelAreaX1 = gridX - 4;

    if (x < labelAreaX0 || x > labelAreaX1) return null;
    if (y < gridY || y > gridY + gridH) return null;

    const row = Math.floor((y - gridY) / cellH);
    return row >= 0 && row < size ? row : null;
  }

  function stopRunning() {
    if (rafId == null) return;
    cancelAnimationFrame(rafId);
    rafId = null;
    lastTs = 0;
  }

  function startDemo() {
    if (rafId != null) return;
    selectedHead = 0;
    syncHeadControl();
    demoElapsed = 0;
    mode = "DEMO";
    rafId = requestAnimationFrame(tick);
  }

  function tick(ts) {
    if (rafId == null) return;
    if (!lastTs) lastTs = ts;
    const dt = Math.min(0.05, (ts - lastTs) / 1000);
    lastTs = ts;

    demoElapsed += dt;
    const STEP_SECONDS = 0.9;
    if (demoElapsed >= STEP_SECONDS) {
      demoElapsed = 0;
      selectedHead = (selectedHead + 1) % heads.length;
      syncHeadControl();
      if (selectedHead === heads.length - 1) {
        mode = "HOLD";
        stopRunning();
        scheduleDraw();
        return;
      }
    }

    draw();
    rafId = requestAnimationFrame(tick);
  }

  const control = {
    isRunning: false,
    onEnter: () => {
      control.isRunning = true;
      if (pendingPlay) {
        pendingPlay = false;
        startDemo();
      } else {
        mode = "HOLD";
        scheduleDraw();
      }
    },
    onExit: () => {
      control.isRunning = false;
      stopRunning();
      mode = "IDLE";
    }
  };

  const controlsRoot = canvas.closest(".ui-canvas-frame")?.querySelector(".ui-canvas-title") || null;
  const headCtl = controlsRoot?.querySelector('[data-attnctl="head"]') || null;
  const playBtn = controlsRoot?.querySelector('[data-attnctl="play"]') || null;
  const resetBtn = controlsRoot?.querySelector('[data-attnctl="reset"]') || null;

  function syncHeadControl() {
    if (!headCtl) return;
    if (headCtl.tagName === "SELECT") {
      headCtl.innerHTML = "";
      heads.forEach((head, idx) => {
        const option = document.createElement("option");
        option.value = idx;
        option.textContent = head.name;
        headCtl.appendChild(option);
      });
      headCtl.value = String(selectedHead);
    } else {
      headCtl.textContent = `Head: ${heads[selectedHead].name}`;
    }
  }

  syncHeadControl();

  if (headCtl) {
    if (headCtl.tagName === "SELECT") {
      headCtl.addEventListener("change", (event) => {
        selectedHead = Number(event.target.value) || 0;
        syncHeadControl();
        scheduleDraw();
      });
    } else {
      headCtl.addEventListener("click", () => {
        selectedHead = (selectedHead + 1) % heads.length;
        syncHeadControl();
        scheduleDraw();
      });
    }
  }

  if (playBtn) {
    playBtn.addEventListener("click", () => {
      if (!control.isRunning) {
        pendingPlay = true;
        return;
      }
      if (mode === "DEMO") {
        mode = "HOLD";
        stopRunning();
        scheduleDraw();
        return;
      }
      startDemo();
    });
  }

  if (resetBtn) {
    resetBtn.addEventListener("click", () => {
      selectedRow = null;
      hoverCell = null;
      hoverRow = null;
      mode = "HOLD";
      stopRunning();
      scheduleDraw();
    });
  }

  if (window.ViewportObserver && typeof window.ViewportObserver.observe === "function") {
    window.ViewportObserver.observe(canvas, control, 0.1);
  } else {
    control.onEnter();
  }

  const observerFallback = setInterval(() => {
    if (control.isRunning && pendingPlay) {
      pendingPlay = false;
      startDemo();
    }
    if (!control.isRunning && rafId != null) {
      stopRunning();
    }
  }, 1200);

  canvas.addEventListener("mousemove", (event) => {
    const rect = canvas.getBoundingClientRect();
    const x = event.clientX - rect.left;
    const y = event.clientY - rect.top;

    hoverCell = cellFromPoint(x, y);
    hoverRow = hoverCell ? hoverCell.row : rowLabelFromPoint(x, y);
    scheduleDraw();
  });

  canvas.addEventListener("mouseleave", () => {
    hoverCell = null;
    hoverRow = null;
    scheduleDraw();
  });

  canvas.addEventListener("click", (event) => {
    const rect = canvas.getBoundingClientRect();
    const x = event.clientX - rect.left;
    const y = event.clientY - rect.top;

    if (lastLayout?.headBoxes) {
      const hitHead = lastLayout.headBoxes.find((b) => x >= b.x && x <= b.x + b.w && y >= b.y && y <= b.y + b.h);
      if (hitHead) {
        selectedHead = hitHead.idx;
        syncHeadControl();
        scheduleDraw();
        return;
      }
    }

    const rowLabel = rowLabelFromPoint(x, y);
    if (rowLabel != null) {
      selectedRow = rowLabel;
      scheduleDraw();
      return;
    }

    const cell = cellFromPoint(x, y);
    if (cell) {
      selectedRow = cell.row;
      scheduleDraw();
    }
  });

  window.addEventListener("beforeunload", () => clearInterval(observerFallback));

  scheduleDraw();
})();
