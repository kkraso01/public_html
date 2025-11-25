// 3D-Style Neural Galaxy Visualizer (Canvas-based 2.5D)
function init3DVisualizer() {
  const canvas3d = document.getElementById("viz3d");
  if (!canvas3d) {
    console.warn("Canvas viz3d not found");
    return;
  }

  const ctx = canvas3d.getContext("2d");
  let w, h, dpr;
  let t = 0;

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

  // Neural nodes
  const nodeCount = 8;
  const nodes = [];
  for (let i = 0; i < nodeCount; i++) {
    const angle = (i / nodeCount) * Math.PI * 2;
    nodes.push({
      baseAngle: angle,
      orbitRadius: 80,
      size: 8 + Math.random() * 6,
      speed: 0.002 + Math.random() * 0.002,
      hue: 190 + i * 30,
      wobble: Math.random() * Math.PI * 2
    });
  }

  // Particle nebula
  const particles = [];
  for (let i = 0; i < 150; i++) {
    particles.push({
      x: (Math.random() - 0.5) * w,
      y: (Math.random() - 0.5) * h,
      vx: (Math.random() - 0.5) * 0.5,
      vy: (Math.random() - 0.5) * 0.5,
      hue: 200 + Math.random() * 100,
      size: Math.random() * 2 + 0.5,
      depth: Math.random()
    });
  }

  // Neural connections
  const connections = [];
  for (let i = 0; i < nodes.length; i++) {
    for (let j = i + 1; j < nodes.length; j++) {
      if (Math.random() > 0.55) {
        connections.push({ from: i, to: j, alpha: 0 });
      }
    }
  }

  function drawParticles() {
    particles.forEach(p => {
      p.x += p.vx;
      p.y += p.vy;

      // Wrap around
      if (p.x < -w / 2) p.x = w / 2;
      if (p.x > w / 2) p.x = -w / 2;
      if (p.y < -h / 2) p.y = h / 2;
      if (p.y > h / 2) p.y = -h / 2;

      const opacity = 0.3 + p.depth * 0.4;
      ctx.fillStyle = `hsla(${p.hue}, 80%, 60%, ${opacity})`;
      ctx.beginPath();
      ctx.arc(w / 2 + p.x, h / 2 + p.y, p.size, 0, Math.PI * 2);
      ctx.fill();
    });
  }

  function drawConnections() {
    const cx = w / 2;
    const cy = h / 2;

    connections.forEach((conn, idx) => {
      const node1 = nodes[conn.from];
      const node2 = nodes[conn.to];

      const angle1 = node1.baseAngle + t * node1.speed;
      const angle2 = node2.baseAngle + t * node2.speed;

      const x1 = cx + Math.cos(angle1) * node1.orbitRadius;
      const y1 = cy + Math.sin(angle1) * node1.orbitRadius;

      const x2 = cx + Math.cos(angle2) * node2.orbitRadius;
      const y2 = cy + Math.sin(angle2) * node2.orbitRadius;

      // Pulsing alpha
      const pulse = Math.sin(t * 0.01 + idx) * 0.5 + 0.5;
      ctx.strokeStyle = `rgba(167, 139, 250, ${pulse * 0.4})`;
      ctx.lineWidth = 1.2;
      ctx.beginPath();
      ctx.moveTo(x1, y1);
      ctx.lineTo(x2, y2);
      ctx.stroke();
    });
  }

  function drawNodes() {
    const cx = w / 2;
    const cy = h / 2;

    nodes.forEach((node, i) => {
      const angle = node.baseAngle + t * node.speed;
      const wobble = Math.sin(Date.now() * 0.001 + node.wobble) * 3;

      const x = cx + Math.cos(angle) * node.orbitRadius + wobble;
      const y = cy + Math.sin(angle) * node.orbitRadius + wobble;

      // Glow
      const glowGrad = ctx.createRadialGradient(x, y, 0, x, y, node.size * 4);
      glowGrad.addColorStop(0, `hsla(${node.hue}, 100%, 70%, 0.8)`);
      glowGrad.addColorStop(1, "rgba(0, 0, 0, 0)");

      ctx.fillStyle = glowGrad;
      ctx.beginPath();
      ctx.arc(x, y, node.size * 4, 0, Math.PI * 2);
      ctx.fill();

      // Node core
      ctx.fillStyle = `hsla(${node.hue}, 95%, 75%, 1)`;
      ctx.beginPath();
      ctx.arc(x, y, node.size, 0, Math.PI * 2);
      ctx.fill();

      // Rotating ring
      ctx.strokeStyle = `hsla(${node.hue}, 80%, 70%, 0.6)`;
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      ctx.arc(x, y, node.size * 1.8, 0, Math.PI * 2);
      ctx.stroke();
    });
  }

  function animate() {
    t += 1;
    ctx.clearRect(0, 0, w, h);

    drawParticles();
    drawConnections();
    drawNodes();

    requestAnimationFrame(animate);
  }

  animate();
}

// Initialize when DOM is ready
if (document.readyState === "loading") {
  document.addEventListener("DOMContentLoaded", init3DVisualizer);
} else {
  init3DVisualizer();
}
