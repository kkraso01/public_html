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
    const panelWidth = clamp(width * 0.26, 170, 230);
    const panelHeight = clamp(height * 0.26, 120, 180);
    const hasSidePanel = s.right - s.left - panelWidth - pad >= 360;
    const stacked = !hasSidePanel;

    const gridAreaWidth = stacked ? s.right - s.left : s.right - s.left - panelWidth - pad;
    const gridAreaHeight = stacked ? s.bottom - s.top - panelHeight - pad : s.bottom - s.top;

    const labelLeft = clamp(Math.round(gridAreaWidth * 0.2), 56, 92);
    const labelTop = 42;
    const labelBottom = 24;
    const gridSize = Math.max(80, Math.min(gridAreaWidth - labelLeft, gridAreaHeight - labelTop - labelBottom));
    const gridX = s.left + labelLeft;
    const gridY = s.top + labelTop;

    const panelX = stacked ? s.left : s.left + gridAreaWidth + pad;
    const panelY = stacked ? gridY + gridSize + labelBottom + pad : gridY;
    const panelW = stacked ? s.right - s.left : panelWidth;
    const panelH = stacked ? s.bottom - panelY : gridSize;

    return {
      gridX,
      gridY,
      gridSize,
      cell: gridSize / size,
      labelLeft,
      labelTop,
      labelBottom,
      panelX,
      panelY,
      panelW,
      panelH,
      stacked,
      bounds: s
    };
  }

  function drawHeadSelector(colors, layout) {
    const { gridX, gridY } = layout;
    const chipW = 64;
    const chipH = 20;
    const gap = 6;
    const maxPerRow = Math.max(1, Math.floor((width - gridX - pad) / (chipW + gap)));
    const rows = Math.ceil(heads.length / maxPerRow);
    const startX = gridX;
    const startY = layout.bounds.top + 6;
    const headBoxes = [];

    ctx.font = "600 10px ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace";
    ctx.textAlign = "center";
    ctx.textBaseline = "middle";

    heads.forEach((head, idx) => {
      const row = Math.floor(idx / maxPerRow);
      const col = idx % maxPerRow;
      const x = startX + col * (chipW + gap);
      const y = startY + row * (chipH + 6);
      const isActive = idx === selectedHead;
      headBoxes.push({ x, y, w: chipW, h: chipH, idx });

      ctx.fillStyle = isActive
        ? (theme ? theme.rgba(colors.accent, 0.16) : colors.surfaceElevated)
        : colors.surfaceElevated;
      ctx.strokeStyle = isActive ? colors.accent : colors.border;
      ctx.lineWidth = 1;
      ctx.beginPath();
      drawRoundedRect(x, y, chipW, chipH, 3);
      ctx.fill();
      ctx.stroke();

      ctx.fillStyle = isActive ? colors.textStrong : colors.muted;
      ctx.fillText(head.name, x + chipW / 2, y + chipH / 2);
    });

    return { headBoxes, selectorHeight: rows * (chipH + 6) };
  }

  function drawMatrix(colors, layout) {
    const { gridX, gridY, gridSize, cell } = layout;

    ctx.fillStyle = colors.surface;
    ctx.fillRect(0, 0, width, height);
    if (theme) {
      theme.drawGrid(ctx, 22, colors.border, theme.isDark() ? 0.1 : 0.06, width, height);
    }

    ctx.fillStyle = colors.surfaceElevated;
    ctx.strokeStyle = theme ? theme.rgba(colors.border, 0.8) : colors.border;
    ctx.lineWidth = 1;
    ctx.beginPath();
    drawRoundedRect(gridX - 12, gridY - 16, gridSize + 24, gridSize + 32, 4);
    ctx.fill();
    ctx.stroke();

    ctx.font = "700 12px ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace";
    ctx.fillStyle = colors.textStrong;
    ctx.textAlign = "left";
    ctx.fillText("Semantic Attention Map", gridX - 6, gridY - 26);

    const matrix = headMatrices[selectedHead];

    for (let row = 0; row < size; row += 1) {
      for (let col = 0; col < size; col += 1) {
        const value = matrix[row][col];
        const shade = Math.max(18, Math.min(240, Math.floor(255 * value)));
        ctx.fillStyle = `rgb(${shade},${shade},${shade})`;
        ctx.fillRect(gridX + col * cell, gridY + row * cell, cell - 1, cell - 1);
      }
    }

    if (selectedRow != null) {
      ctx.fillStyle = theme ? theme.rgba(colors.accent, 0.08) : colors.surface;
      ctx.fillRect(gridX, gridY + selectedRow * cell, gridSize, cell);
    }

    if (hoverRow != null) {
      ctx.fillStyle = theme ? theme.rgba(colors.accent, 0.08) : colors.surface;
      ctx.fillRect(gridX, gridY + hoverRow * cell, gridSize, cell);
    }

    ctx.strokeStyle = theme ? theme.rgba(colors.border, 0.8) : colors.border;
    ctx.lineWidth = 1;
    ctx.strokeRect(gridX, gridY, gridSize, gridSize);

    ctx.save();
    ctx.font = "600 10px ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace";
    ctx.fillStyle = colors.muted;
    ctx.textAlign = "center";
    tokens.forEach((tok, i) => {
      const labelRaw = tok.length > 7 ? `${tok.slice(0, 6)}…` : tok;
      const label = labelRaw.toUpperCase();
      ctx.fillText(label, gridX + i * cell + cell / 2, gridY - 10);

      ctx.save();
      ctx.translate(gridX - 16, gridY + i * cell + cell / 2);
      ctx.rotate(-Math.PI / 2);
      ctx.fillText(label, 0, 0);
      ctx.restore();
    });
    ctx.restore();

    if (hoverCell) {
      ctx.strokeStyle = colors.accent;
      ctx.lineWidth = 1;
      ctx.strokeRect(gridX + hoverCell.col * cell, gridY + hoverCell.row * cell, cell, cell);
      ctx.setLineDash([4, 4]);
      ctx.strokeRect(gridX, gridY + hoverCell.row * cell, gridSize, cell);
      ctx.strokeRect(gridX + hoverCell.col * cell, gridY, cell, gridSize);
      ctx.setLineDash([]);
    }

    const legendX = gridX + gridSize + 12;
    const legendY = gridY + gridSize - 10;
    if (legendX + 90 <= width - pad) {
      ctx.fillStyle = colors.muted;
      ctx.textAlign = "left";
      ctx.font = "600 10px ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace";
      ctx.fillText("Weight", legendX, legendY - 6);
      for (let i = 0; i <= 8; i += 1) {
        const val = i / 8;
        const shade = Math.floor(255 * val);
        ctx.fillStyle = `rgb(${shade},${shade},${shade})`;
        ctx.fillRect(legendX + i * 10, legendY, 8, 10);
      }
      ctx.fillStyle = colors.muted;
      ctx.fillText("0", legendX, legendY + 22);
      ctx.fillText("1", legendX + 80, legendY + 22);
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
  }

  function drawTooltip(colors, layout) {
    if (!hoverCell) return;
    const { gridX, gridY, cell } = layout;
    const matrix = headMatrices[selectedHead];
    const weight = matrix[hoverCell.row][hoverCell.col];
    const text = `${tokens[hoverCell.row]} → ${tokens[hoverCell.col]} (${weight.toFixed(3)})`;

    ctx.font = "600 11px ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace";
    const w = ctx.measureText(text).width + 12;
    const h = 24;
    let x = gridX + hoverCell.col * cell + cell + 8;
    let y = gridY + hoverCell.row * cell + cell / 2 - 12;

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
    const { gridX, gridY, gridSize, cell } = lastLayout;
    if (x < gridX || y < gridY || x > gridX + gridSize || y > gridY + gridSize) return null;
    const col = Math.floor((x - gridX) / cell);
    const row = Math.floor((y - gridY) / cell);
    if (col >= 0 && col < size && row >= 0 && row < size) {
      return { row, col };
    }
    return null;
  }

  function rowLabelFromPoint(x, y) {
    if (!lastLayout) return null;
    const { gridX, gridY, gridSize, cell } = lastLayout;
    const labelWidth = Math.min(48, lastLayout.labelLeft - 8);
    const labelAreaX = gridX - labelWidth;
    if (x < labelAreaX || x > gridX - 4) return null;
    if (y < gridY || y > gridY + gridSize) return null;
    const row = Math.floor((y - gridY) / cell);
    if (row >= 0 && row < size) return row;
    return null;
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
