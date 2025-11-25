// 3D-Style Neural Galaxy Visualizer re-focused on LLM shards and retrieval planes
function init3DVisualizer() {
  const canvas3d = document.getElementById("viz3d");
  if (!canvas3d) {
    console.warn("Canvas viz3d not found");
    return;
  }

  const ctx = canvas3d.getContext("2d");
  const bus = window.EventBus;
  let w, h, dpr;
  let t = 0;
  let running = true;
  let speed = 1;
  let highlight = 0;

  function resize() {
    dpr = window.devicePixelRatio || 1;
    const rect = canvas3d.getBoundingClientRect();
    w = rect.width;
    h = rect.height;
    canvas3d.width = w * dpr;
    canvas3d.height = h * dpr;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  }

  resize();
  window.addEventListener("resize", resize);

  const layers = [
    { id: "embed", label: "Embedding plane", z: 0.1, color: "#22d3ee" },
    { id: "attn", label: "Multi-head lattice", z: 0.35, color: "#a78bfa" },
    { id: "shards", label: "LLM shards", z: 0.6, color: "#38bdf8" },
    { id: "retrieval", label: "Vector retrieval", z: 0.8, color: "#34d399" }
  ];

  const nodeGrid = layers.map((layer, i) => {
    const count = 6 - i; // fewer nodes as we go up
    const nodes = [];
    for (let x = 0; x < count; x++) {
      for (let y = 0; y < count; y++) {
        nodes.push({
          x: x / (count - 1),
          y: y / (count - 1),
          pulse: Math.random(),
          jitter: Math.random()
        });
      }
    }
    return { ...layer, nodes };
  });

  const beams = [];
  for (let i = 0; i < nodeGrid.length - 1; i++) {
    const lower = nodeGrid[i].nodes;
    const upper = nodeGrid[i + 1].nodes;
    for (let j = 0; j < 12; j++) {
      beams.push({
        from: lower[Math.floor(Math.random() * lower.length)],
        to: upper[Math.floor(Math.random() * upper.length)],
        startLayer: nodeGrid[i],
        endLayer: nodeGrid[i + 1],
        offset: Math.random(),
        speed: 0.002 + Math.random() * 0.002,
        hue: nodeGrid[i + 1].color
      });
    }
  }

  function project(x, y, z) {
    const scale = 1 - z * 0.45;
    const px = w / 2 + (x - 0.5) * w * scale;
    const py = h * 0.2 + (y - 0.5) * h * 0.5 + z * 160;
    return { x: px, y: py, scale };
  }

  function drawBackground() {
    const grad = ctx.createLinearGradient(0, 0, 0, h);
    grad.addColorStop(0, "#0b1224");
    grad.addColorStop(1, "#050910");
    ctx.fillStyle = grad;
    ctx.fillRect(0, 0, w, h);

    ctx.strokeStyle = "rgba(79,70,229,0.35)";
    ctx.lineWidth = 1;
    const spacing = 34;
    ctx.beginPath();
    for (let x = 0; x < w; x += spacing) {
      ctx.moveTo(x, 0);
      ctx.lineTo(x, h);
    }
    for (let y = 0; y < h; y += spacing) {
      ctx.moveTo(0, y);
      ctx.lineTo(w, y);
    }
    ctx.stroke();
  }

  function drawLayerPlanes() {
    nodeGrid.forEach((layer) => {
      const base = project(0.5, 0.5, layer.z);
      const planeW = w * 0.75 * base.scale;
      const planeH = h * 0.28 * base.scale;
      ctx.save();
      ctx.translate(base.x, base.y);
      ctx.rotate(-0.06);
      ctx.fillStyle = "rgba(15,23,42,0.9)";
      ctx.strokeStyle = `${layer.color}55`;
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.rect(-planeW / 2, -planeH / 2, planeW, planeH);
      ctx.fill();
      ctx.stroke();
      ctx.restore();

      ctx.fillStyle = "rgba(226,232,240,0.9)";
      ctx.font = "11px monospace";
      ctx.textAlign = "center";
      ctx.fillText(layer.label, base.x, base.y - planeH * 0.55);
    });
  }

  function drawNodes() {
    nodeGrid.forEach((layer) => {
      layer.nodes.forEach((n) => {
        const pr = project(n.x, n.y, layer.z);
        const alpha = 0.5 + 0.5 * Math.sin(t * 0.01 + n.pulse * 6);
        const size = 6 * pr.scale + highlight;
        ctx.fillStyle = `${layer.color}${Math.floor(alpha * 255)
          .toString(16)
          .padStart(2, "0")}`;
        ctx.beginPath();
        ctx.rect(pr.x - size / 2, pr.y - size / 2, size, size);
        ctx.fill();
      });
    });
  }

  function drawBeams() {
    beams.forEach((b) => {
      if (running) b.offset = (b.offset + b.speed * speed) % 1;
      const from = project(b.from.x, b.from.y, b.startLayer.z);
      const to = project(b.to.x, b.to.y, b.endLayer.z);

      ctx.strokeStyle = `${b.hue}55`;
      ctx.lineWidth = 1.4;
      ctx.beginPath();
      ctx.moveTo(from.x, from.y);
      ctx.lineTo(to.x, to.y);
      ctx.stroke();

      // moving packet
      const px = from.x + (to.x - from.x) * b.offset;
      const py = from.y + (to.y - from.y) * b.offset;
      ctx.fillStyle = b.hue;
      ctx.beginPath();
      ctx.arc(px, py, 4 + highlight * 0.8, 0, Math.PI * 2);
      ctx.fill();
    });
  }

  function drawHUD() {
    ctx.save();
    ctx.translate(16, 16);
    ctx.fillStyle = "rgba(15,23,42,0.82)";
    ctx.strokeStyle = "rgba(129,140,248,0.5)";
    ctx.lineWidth = 1.2;
    ctx.beginPath();
    ctx.roundRect(0, 0, 200, 96, 12);
    ctx.fill();
    ctx.stroke();

    ctx.fillStyle = "rgba(226,232,240,0.92)";
    ctx.font = "11px monospace";
    ctx.fillText("Neural Galaxy (LLM stack)", 12, 20);
    ctx.fillStyle = "rgba(148,163,184,0.9)";
    ctx.fillText(`speed x${speed.toFixed(2)}`, 12, 38);
    ctx.fillText(`highlight ${highlight.toFixed(2)}`, 12, 54);
    ctx.fillText(`planes ${layers.length}`, 12, 70);
    ctx.fillText("drag = orbit • click = spike", 12, 86);
    ctx.restore();
  }

  function animate() {
    if (running) {
      t += 16;
      highlight = Math.max(0, highlight - 0.01);
    }

    ctx.clearRect(0, 0, w, h);
    drawBackground();
    drawLayerPlanes();
    drawBeams();
    drawNodes();
    drawHUD();

    requestAnimationFrame(animate);
  }

  function handleControl(payload) {
    if (!payload) return;
    if (payload.action === "toggle") running = !payload.value;
    if (payload.action === "speed" && typeof payload.value === "number") speed = payload.value;
    if (payload.action === "spike") {
      highlight = Math.min(3, highlight + (payload.value || 1));
      if (bus) bus.emit("telemetry:spike", { source: "galaxy3d", intensity: highlight });
    }
  }

  canvas3d.addEventListener("click", () => {
    highlight = Math.min(3, highlight + 0.8);
    if (bus) bus.emit("telemetry:spike", { source: "galaxy3d", intensity: highlight });
  });

  if (bus) {
    bus.on("control:galaxy3d", handleControl);
    bus.on("telemetry:spike", ({ source, intensity }) => {
      if (source === "galaxy3d") return;
      highlight = Math.min(3, highlight + (intensity || 0.4));
    });
  }

  animate();
}

// Initialize when DOM is ready
if (document.readyState === "loading") {
  document.addEventListener("DOMContentLoaded", init3DVisualizer);
} else {
  init3DVisualizer();
}
