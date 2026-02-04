(function () {
  const canvas = document.getElementById("attentionCanvas");
  if (!canvas) {
    console.warn("[Attention] canvas not found");
    return;
  }

  const ctx = canvas.getContext("2d");
  const theme = window.UI_THEME;
  let W = 0;
  let H = 0;
  let dpr = 1;
  let hoverCell = null;
  let selectedRow = null;
  let selectedHead = 0;
  let needsRender = false;
  let headBoxes = [];

  const tokens = ["<bos>", "query", "retrieval", "router", "context", "policy", "logits", "answer"];
  const size = tokens.length;
  const heads = [
    { name: "Head A", seed: 0.9 },
    { name: "Head B", seed: 1.6 },
    { name: "Head C", seed: 2.4 }
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

  function resize() {
    if (theme) {
      const metrics = theme.setDPR(canvas, ctx);
      W = metrics.width;
      H = metrics.height;
      dpr = metrics.dpr;
    } else {
      dpr = window.devicePixelRatio || 1;
      const rect = canvas.getBoundingClientRect();
      W = rect.width;
      H = rect.height;
      canvas.width = W * dpr;
      canvas.height = H * dpr;
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    }
    scheduleDraw();
  }

  resize();
  window.addEventListener("resize", resize);
  window.addEventListener("themechange", resize);

  const control = { isRunning: true };
  window.ViewportObserver.observe(canvas, control, 0.1);

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

  function computeHead(seed) {
    const matrix = Array.from({ length: size }, () => new Array(size).fill(0));
    for (let q = 0; q < size; q += 1) {
      let sum = 0;
      for (let k = 0; k < size; k += 1) {
        const base = Math.cos((q + 1) * seed + (k + 2) * 0.7) + Math.sin((q - k + 1) * 0.6 + seed * 0.8) + (q === k ? 0.7 : 0);
        const val = Math.exp(base);
        matrix[q][k] = val;
        sum += val;
      }
      for (let k = 0; k < size; k += 1) {
        matrix[q][k] = matrix[q][k] / sum;
      }
    }
    return matrix;
  }

  const headMatrices = heads.map((h) => computeHead(h.seed));

  function topLinks(rowIndex, headIndex, k = 5) {
    if (rowIndex == null) return [];
    const weights = headMatrices[headIndex][rowIndex] || [];
    const entries = weights.map((w, idx) => ({ token: tokens[idx], weight: w }));
    entries.sort((a, b) => b.weight - a.weight);
    return entries.slice(0, k);
  }

  function drawHeadSelector(colors, startX, startY) {
    headBoxes = [];
    const chipW = 74;
    const chipH = 22;
    const gap = 8;
    heads.forEach((head, idx) => {
      const x = startX + idx * (chipW + gap);
      const y = startY;
      const isActive = idx === selectedHead;
      headBoxes.push({ x, y, w: chipW, h: chipH, idx });

      ctx.fillStyle = isActive ? (theme ? theme.rgba(colors.accent, 0.16) : colors.surfaceElevated) : colors.surfaceElevated;
      ctx.strokeStyle = isActive ? colors.accent : colors.border;
      ctx.lineWidth = 1;
      ctx.beginPath();
      drawRoundedRect(x, y, chipW, chipH, 3);
      ctx.fill();
      ctx.stroke();

      ctx.fillStyle = isActive ? colors.textStrong : colors.muted;
      ctx.font = "600 11px ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace";
      ctx.textAlign = "center";
      ctx.textBaseline = "middle";
      ctx.fillText(head.name, x + chipW / 2, y + chipH / 2);
    });
  }

  function drawMatrix(colors) {
    const marginLeft = 88;
    const marginTop = 70;
    const marginRight = 160;
    const marginBottom = 50;
    const gridSize = Math.min(W - marginLeft - marginRight, H - marginTop - marginBottom);
    const cell = gridSize / size;
    const startX = marginLeft;
    const startY = marginTop;

    ctx.fillStyle = colors.surface;
    ctx.fillRect(0, 0, W, H);
    if (theme) {
      theme.drawGrid(ctx, 22, colors.border, theme.isDark() ? 0.1 : 0.06, W, H);
    }

    ctx.fillStyle = colors.surfaceElevated;
    ctx.strokeStyle = theme ? theme.rgba(colors.border, 0.8) : colors.border;
    ctx.lineWidth = 1;
    ctx.beginPath();
    drawRoundedRect(startX - 12, startY - 16, gridSize + 24, gridSize + 32, 4);
    ctx.fill();
    ctx.stroke();

    ctx.font = "700 13px ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace";
    ctx.fillStyle = colors.textStrong;
    ctx.textAlign = "left";
    ctx.fillText("Attention Heatmap", startX - 8, startY - 26);

    drawHeadSelector(colors, startX + gridSize - 10, startY - 44);

    const matrix = headMatrices[selectedHead];

    for (let row = 0; row < size; row += 1) {
      for (let col = 0; col < size; col += 1) {
        const value = matrix[row][col];
        const shade = Math.max(18, Math.min(240, Math.floor(255 * value)));
        ctx.fillStyle = `rgb(${shade},${shade},${shade})`;
        ctx.fillRect(startX + col * cell, startY + row * cell, cell - 1, cell - 1);
      }
    }

    if (selectedRow != null) {
      ctx.fillStyle = theme ? theme.rgba(colors.accent, 0.08) : colors.surface;
      ctx.fillRect(startX, startY + selectedRow * cell, gridSize, cell);
    }

    ctx.strokeStyle = theme ? theme.rgba(colors.border, 0.8) : colors.border;
    ctx.lineWidth = 1;
    ctx.strokeRect(startX, startY, gridSize, gridSize);

    ctx.save();
    ctx.font = "600 11px ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace";
    ctx.fillStyle = colors.muted;
    ctx.textAlign = "center";
    tokens.forEach((tok, i) => {
      const labelRaw = tok.length > 6 ? `${tok.slice(0, 5)}…` : tok;
      const label = labelRaw.toUpperCase();
      ctx.fillText(label, startX + i * cell + cell / 2, startY - 12);

      ctx.save();
      ctx.translate(startX - 16, startY + i * cell + cell / 2);
      ctx.rotate(-Math.PI / 2);
      ctx.fillText(label, 0, 0);
      ctx.restore();
    });
    ctx.restore();

    if (hoverCell) {
      ctx.strokeStyle = colors.accent;
      ctx.lineWidth = 1;
      ctx.strokeRect(startX + hoverCell.col * cell, startY + hoverCell.row * cell, cell, cell);
      ctx.setLineDash([4, 4]);
      ctx.strokeRect(startX, startY + hoverCell.row * cell, gridSize, cell);
      ctx.strokeRect(startX + hoverCell.col * cell, startY, cell, gridSize);
      ctx.setLineDash([]);
    }

    const legendX = startX + gridSize + 14;
    const legendY = startY + gridSize - 10;
    ctx.fillStyle = colors.muted;
    ctx.textAlign = "left";
    ctx.font = "600 11px ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace";
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

    return { startX, startY, cell, gridSize, legendX, legendY };
  }

  function drawLinks(colors) {
    const panelX = W - 140;
    const panelY = 70;
    const panelW = 120;
    const panelH = H - panelY - 40;

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
    ctx.fillText("Top-5 links", panelX + 10, panelY + 16);

    const links = topLinks(selectedRow ?? 0, selectedHead, 5);
    ctx.fillStyle = colors.textStrong;
    ctx.font = "700 12px ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace";
    ctx.fillText(selectedRow != null ? tokens[selectedRow].toUpperCase() : "SELECT ROW", panelX + 10, panelY + 34);

    ctx.fillStyle = colors.muted;
    ctx.font = "600 11px ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace";
    links.forEach((item, idx) => {
      const weight = (item.weight * 100).toFixed(1);
      const y = panelY + 56 + idx * 18;
      ctx.fillText(`${idx + 1}. ${item.token} (${weight}%)`, panelX + 10, y);
    });
  }

  function drawTooltip(colors, grid) {
    if (!hoverCell) return;
    const { startX, startY, cell } = grid;
    const x = startX + hoverCell.col * cell + cell + 8;
    const y = startY + hoverCell.row * cell + cell / 2 - 10;
    const matrix = headMatrices[selectedHead];
    const weight = matrix[hoverCell.row][hoverCell.col];
    const text = `${tokens[hoverCell.row]} → ${tokens[hoverCell.col]} (${weight.toFixed(3)})`;

    ctx.font = "600 11px ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace";
    const w = ctx.measureText(text).width + 12;
    const h = 24;
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
    const grid = drawMatrix(colors);
    drawLinks(colors);
    drawTooltip(colors, grid);
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
    const marginLeft = 88;
    const marginTop = 54;
    const marginRight = 160;
    const marginBottom = 50;
    const gridSize = Math.min(W - marginLeft - marginRight, H - marginTop - marginBottom);
    const cell = gridSize / size;
    const startX = marginLeft;
    const startY = marginTop;
    const col = Math.floor((x - startX) / cell);
    const row = Math.floor((y - startY) / cell);
    if (col >= 0 && col < size && row >= 0 && row < size) {
      return { row, col };
    }
    return null;
  }

  canvas.addEventListener("mousemove", (event) => {
    const rect = canvas.getBoundingClientRect();
    const x = event.clientX - rect.left;
    const y = event.clientY - rect.top;

    hoverCell = cellFromPoint(x, y);
    scheduleDraw();
  });

  canvas.addEventListener("mouseleave", () => {
    hoverCell = null;
    scheduleDraw();
  });

  canvas.addEventListener("click", (event) => {
    const rect = canvas.getBoundingClientRect();
    const x = event.clientX - rect.left;
    const y = event.clientY - rect.top;

    const hitHead = headBoxes.find((b) => x >= b.x && x <= b.x + b.w && y >= b.y && y <= b.y + b.h);
    if (hitHead) {
      selectedHead = hitHead.idx;
      scheduleDraw();
      return;
    }

    const cell = cellFromPoint(x, y);
    if (cell) {
      selectedRow = cell.row;
      scheduleDraw();
    }
  });

  scheduleDraw();
})();
