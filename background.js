(function () {
  const canvas = document.getElementById("nn-bg");
  if (!canvas) return;
  const ctx = canvas.getContext("2d");
  const theme = window.UI_THEME;

  let width = window.innerWidth;
  let height = window.innerHeight;
  let dpr = window.devicePixelRatio || 1;

  const NODE_COUNT = 48;
  const MAX_LINK_DIST = 160;
  const nodes = [];
  const pointer = { x: 0, y: 0, active: false, lastMove: 0 };

  function randRange(min, max) {
    return Math.random() * (max - min) + min;
  }

  function initNodes() {
    nodes.length = 0;
    for (let i = 0; i < NODE_COUNT; i++) {
      nodes.push({
        x: Math.random() * width,
        y: Math.random() * height,
        z: Math.random(),
        vx: randRange(-0.08, 0.08),
        vy: randRange(-0.08, 0.08)
      });
    }
  }

  function resize() {
    width = window.innerWidth;
    height = window.innerHeight;
    dpr = window.devicePixelRatio || 1;
    canvas.width = width * dpr;
    canvas.height = height * dpr;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    initNodes();
  }

  window.addEventListener("resize", resize);
  window.addEventListener("mousemove", (event) => {
    pointer.x = event.clientX;
    pointer.y = event.clientY;
    pointer.active = true;
    pointer.lastMove = performance.now();
  });
  window.addEventListener("mouseleave", () => {
    pointer.active = false;
  });
  window.addEventListener("themechange", resize);
  initNodes();
  resize();

  // Viewport observer for performance
  const control = { isRunning: true };
  window.ViewportObserver.observe(canvas, control, 0.1);

  function palette() {
    if (theme) return theme.palette();
    const styles = getComputedStyle(document.documentElement);
    return {
      bg: styles.getPropertyValue("--bg").trim(),
      surface: styles.getPropertyValue("--surface").trim(),
      text: styles.getPropertyValue("--text").trim(),
      muted: styles.getPropertyValue("--muted").trim(),
      border: styles.getPropertyValue("--border").trim(),
      accent: styles.getPropertyValue("--accent").trim(),
    };
  }

  function distance(a, b) {
    const dx = a.x - b.x;
    const dy = a.y - b.y;
    return Math.sqrt(dx * dx + dy * dy);
  }

  function draw() {
    if (!control.isRunning) {
      requestAnimationFrame(draw);
      return;
    }
    ctx.clearRect(0, 0, width, height);

    const colors = palette();
    if (theme) {
      theme.drawGrid(ctx, 48, colors.border, theme.isDark() ? 0.08 : 0.06, width, height);
    }

    const now = performance.now();
    const pointerActive = pointer.active && now - pointer.lastMove < 1200;
    let nearestNode = null;
    let nearestDist = Infinity;

    if (pointerActive) {
      nodes.forEach((node) => {
        const dist = Math.hypot(node.x - pointer.x, node.y - pointer.y);
        if (dist < nearestDist) {
          nearestDist = dist;
          nearestNode = node;
        }
      });
    }

    // Connect nodes
    for (let i = 0; i < NODE_COUNT; i++) {
      const n1 = nodes[i];
      for (let j = i + 1; j < NODE_COUNT; j++) {
        const n2 = nodes[j];
        const dist = distance(n1, n2);
        const showLines = pointerActive ? (dist < MAX_LINK_DIST) : dist < 70;
        if (showLines) {
          const alpha = 1 - dist / MAX_LINK_DIST;
          ctx.strokeStyle = theme
            ? theme.rgba(colors.border, alpha * (theme.isDark() ? 0.35 : 0.28))
            : `rgba(148,163,184,${alpha * 0.4})`;
          ctx.lineWidth = 1;

          ctx.beginPath();
          ctx.moveTo(n1.x, n1.y);
          ctx.lineTo(n2.x, n2.y);
          ctx.stroke();
        }
      }
    }

    // Draw nodes as subtle squares (no glow)
    nodes.forEach((n) => {
      const size = 2 + n.z * 1.6;
      const baseColor = theme
        ? theme.rgba(colors.text, theme.isDark() ? 0.7 : 0.55)
        : "rgba(226,232,240,0.7)";
      ctx.fillStyle = baseColor;
      ctx.fillRect(n.x - size / 2, n.y - size / 2, size, size);
      ctx.strokeStyle = theme
        ? theme.rgba(colors.border, theme.isDark() ? 0.5 : 0.35)
        : "rgba(148,163,184,0.6)";
      ctx.lineWidth = 1;
      ctx.strokeRect(n.x - size / 2, n.y - size / 2, size, size);
    });

    if (nearestNode && nearestDist < 140) {
      const pulse = Math.max(0, 1 - (nearestDist / 140));
      const accentAlpha = 0.2 + pulse * 0.6;
      ctx.strokeStyle = theme ? theme.rgba(colors.accent, accentAlpha) : `rgba(255,45,45,${accentAlpha})`;
      ctx.lineWidth = 1;
      ctx.beginPath();
      ctx.arc(nearestNode.x, nearestNode.y, 10 + pulse * 6, 0, Math.PI * 2);
      ctx.stroke();
    }

    // Move nodes
    nodes.forEach((n) => {
      n.x += n.vx * (0.5 + n.z);
      n.y += n.vy * (0.5 + n.z);

      if (n.x < -50) n.x = width + 50;
      if (n.x > width + 50) n.x = -50;
      if (n.y < -50) n.y = height + 50;
      if (n.y > height + 50) n.y = -50;
    });

    requestAnimationFrame(draw);
  }

  draw();
})();
