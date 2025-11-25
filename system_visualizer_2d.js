// 2D GPT-4 System Visualizer Engine
const canvas2d = document.getElementById("viz2d");
if (!canvas2d) {
  console.warn("Canvas viz2d not found");
} else {
  const ctx2d = canvas2d.getContext("2d");

  let w, h, dpr;
  let t = 0;

  function resize() {
    dpr = window.devicePixelRatio || 1;
    const rect = canvas2d.getBoundingClientRect();
    w = rect.width;
    h = rect.height;

    canvas2d.width = w * dpr;
    canvas2d.height = h * dpr;
    ctx2d.setTransform(dpr, 0, 0, dpr, 0, 0);
  }

  resize();
  window.addEventListener("resize", resize);

  // Core orchestrator
  const core = { x: 0.5, y: 0.5, r: 30 };

  // LLM nodes
  const llmCount = 5;
  const llmNodes = Array.from({ length: llmCount }, (_, i) => ({
    orbit: 0.2 + i * 0.07,
    size: 12 + Math.random() * 5,
    speed: 0.005 + Math.random() * 0.004,
    hue: 200 + i * 35,
    active: Math.random() > 0.4
  }));

  // Token particles
  const tokenParticles = [];
  for (let i = 0; i < 100; i++) {
    tokenParticles.push({
      a: Math.random() * Math.PI * 2,
      r: Math.random() * 0.35 + 0.08,
      speed: 0.002 + Math.random() * 0.004,
      hue: 190 + Math.random() * 120,
      size: 1 + Math.random() * 2
    });
  }

  // Vector DB sphere
  const vectorDB = { x: 0.15, y: 0.75, r: 20, hue: 100 };

  function drawCore() {
    const x = w * core.x;
    const y = h * core.y;

    // Outer glow
    const glow = ctx2d.createRadialGradient(x, y, 0, x, y, core.r * 4);
    glow.addColorStop(0, "rgba(200, 220, 255, 0.6)");
    glow.addColorStop(0.5, "rgba(100, 150, 255, 0.2)");
    glow.addColorStop(1, "rgba(0, 0, 0, 0)");

    ctx2d.fillStyle = glow;
    ctx2d.beginPath();
    ctx2d.arc(x, y, core.r * 4, 0, Math.PI * 2);
    ctx2d.fill();

    // Core body
    ctx2d.fillStyle = "rgba(255, 255, 255, 0.9)";
    ctx2d.beginPath();
    ctx2d.arc(x, y, core.r, 0, Math.PI * 2);
    ctx2d.fill();

    // Rotating ring
    ctx2d.strokeStyle = `rgba(100, 200, 255, ${0.3 + 0.3 * Math.sin(t * 0.02)})`;
    ctx2d.lineWidth = 2;
    ctx2d.beginPath();
    ctx2d.arc(x, y, core.r * 1.3, 0, Math.PI * 2);
    ctx2d.stroke();
  }

  function drawLLMNodes() {
    const cx = w * core.x;
    const cy = h * core.y;

    llmNodes.forEach((n, i) => {
      const angle = t * n.speed + i * (Math.PI * 2 / llmNodes.length);
      const x = cx + Math.cos(angle) * (w * n.orbit);
      const y = cy + Math.sin(angle) * (h * n.orbit);

      // Glow
      const nodeGlow = ctx2d.createRadialGradient(x, y, 0, x, y, n.size * 4);
      nodeGlow.addColorStop(0, `hsla(${n.hue}, 100%, 70%, 0.7)`);
      nodeGlow.addColorStop(1, "rgba(0, 0, 0, 0)");

      ctx2d.fillStyle = nodeGlow;
      ctx2d.beginPath();
      ctx2d.arc(x, y, n.size * 4, 0, Math.PI * 2);
      ctx2d.fill();

      // Node core
      ctx2d.fillStyle = `hsla(${n.hue}, 95%, 75%, 1)`;
      ctx2d.beginPath();
      ctx2d.arc(x, y, n.size, 0, Math.PI * 2);
      ctx2d.fill();

      // Connection to core (alternating)
      if (i % 2 === 0) {
        ctx2d.strokeStyle = `hsla(${n.hue}, 80%, 70%, 0.5)`;
        ctx2d.lineWidth = 1.5;
        ctx2d.beginPath();
        ctx2d.moveTo(cx, cy);
        ctx2d.lineTo(x, y);
        ctx2d.stroke();
      }
    });
  }

  function drawVectorDB() {
    const x = w * vectorDB.x;
    const y = h * vectorDB.y;

    // Pulsing grid effect
    const pulse = Math.sin(t * 0.01) * 0.3 + 0.7;

    ctx2d.strokeStyle = `hsla(${vectorDB.hue}, 80%, 60%, ${pulse * 0.6})`;
    ctx2d.lineWidth = 1;
    const gridSize = vectorDB.r / 2;
    for (let i = -1; i <= 1; i++) {
      for (let j = -1; j <= 1; j++) {
        ctx2d.beginPath();
        ctx2d.rect(
          x + i * gridSize - gridSize / 2,
          y + j * gridSize - gridSize / 2,
          gridSize,
          gridSize
        );
        ctx2d.stroke();
      }
    }

    // DB node
    ctx2d.fillStyle = `hsla(${vectorDB.hue}, 90%, 70%, 0.9)`;
    ctx2d.beginPath();
    ctx2d.arc(x, y, vectorDB.r * 0.6, 0, Math.PI * 2);
    ctx2d.fill();
  }

  function drawTokenParticles() {
    const cx = w * core.x;
    const cy = h * core.y;

    tokenParticles.forEach(p => {
      p.a += p.speed;
      const x = cx + Math.cos(p.a) * (w * p.r);
      const y = cy + Math.sin(p.a) * (h * p.r);

      ctx2d.fillStyle = `hsla(${p.hue}, 85%, 70%, 0.6)`;
      ctx2d.beginPath();
      ctx2d.arc(x, y, p.size, 0, Math.PI * 2);
      ctx2d.fill();
    });
  }

  function animate() {
    t += 1;
    ctx2d.clearRect(0, 0, w, h);

    drawTokenParticles();
    drawVectorDB();
    drawLLMNodes();
    drawCore();

    requestAnimationFrame(animate);
  }

  animate();
}
