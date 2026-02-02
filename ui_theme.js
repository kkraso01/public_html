(function () {
  const root = document.documentElement;

  function getVar(name, fallback = "") {
    const value = getComputedStyle(root).getPropertyValue(name).trim();
    return value || fallback;
  }

  function parseColor(value) {
    if (!value) return { r: 0, g: 0, b: 0, a: 1 };
    const trimmed = value.trim();
    if (trimmed.startsWith("#")) {
      const hex = trimmed.replace("#", "");
      const size = hex.length === 3 ? 1 : 2;
      const read = (idx) => parseInt(hex.substr(idx * size, size).padEnd(2, hex[idx] || "0"), 16);
      return {
        r: read(0),
        g: read(1),
        b: read(2),
        a: 1,
      };
    }
    const match = trimmed.match(/rgba?\(([^)]+)\)/);
    if (match) {
      const parts = match[1].split(",").map((part) => parseFloat(part.trim()));
      return {
        r: parts[0] ?? 0,
        g: parts[1] ?? 0,
        b: parts[2] ?? 0,
        a: parts[3] ?? 1,
      };
    }
    return { r: 0, g: 0, b: 0, a: 1 };
  }

  function rgba(value, alpha = 1) {
    const { r, g, b, a } = parseColor(value);
    const nextAlpha = Math.max(0, Math.min(1, alpha * (a ?? 1)));
    return `rgba(${Math.round(r)}, ${Math.round(g)}, ${Math.round(b)}, ${nextAlpha})`;
  }

  function setDPR(canvas, ctx) {
    const dpr = window.devicePixelRatio || 1;
    const rect = canvas.getBoundingClientRect();
    const width = rect.width;
    const height = rect.height;
    canvas.width = width * dpr;
    canvas.height = height * dpr;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    return { width, height, dpr };
  }

  function drawHairline(ctx, x1, y1, x2, y2, color, alpha = 1) {
    const snap = (v) => Math.round(v) + 0.5;
    ctx.save();
    ctx.strokeStyle = rgba(color, alpha);
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(snap(x1), snap(y1));
    ctx.lineTo(snap(x2), snap(y2));
    ctx.stroke();
    ctx.restore();
  }

  function drawGrid(ctx, spacing, color, alpha = 0.12, width = ctx.canvas.width, height = ctx.canvas.height) {
    ctx.save();
    for (let x = 0; x <= width; x += spacing) {
      drawHairline(ctx, x, 0, x, height, color, alpha);
    }
    for (let y = 0; y <= height; y += spacing) {
      drawHairline(ctx, 0, y, width, y, color, alpha);
    }
    ctx.restore();
  }

  function drawLabel(ctx, text, x, y, color, align = "left") {
    ctx.save();
    ctx.fillStyle = color;
    ctx.font = "600 10px ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace";
    ctx.textAlign = align;
    ctx.textBaseline = "middle";
    ctx.fillText(text, x, y);
    ctx.restore();
  }

  window.UI_THEME = {
    get vars() {
      return {
        bg: getVar("--bg"),
        surface: getVar("--surface"),
        surfaceElevated: getVar("--surface-elevated"),
        text: getVar("--text"),
        textStrong: getVar("--text-strong"),
        muted: getVar("--muted"),
        border: getVar("--border"),
        accent: getVar("--accent"),
        overlay: getVar("--bg-overlay"),
      };
    },
    palette() {
      return this.vars;
    },
    isDark() {
      return root.dataset.theme === "dark";
    },
    getVar,
    parseColor,
    rgba,
    setDPR,
    drawHairline,
    drawGrid,
    drawLabel,
  };

  window.refreshCanvases = () => {
    window.dispatchEvent(new Event("themechange"));
  };
})();
