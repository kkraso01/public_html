(function () {
  const canvas = document.getElementById("nn-bg");
  if (!canvas) return;
  const ctx = canvas.getContext("2d");

  let width = window.innerWidth;
  let height = window.innerHeight;

  canvas.width = width;
  canvas.height = height;

  const NODE_COUNT = 70;
  const MAX_LINK_DIST = 180;
  const nodes = [];

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
        vx: randRange(-0.25, 0.25),
        vy: randRange(-0.25, 0.25)
      });
    }
  }

  function resize() {
    width = window.innerWidth;
    height = window.innerHeight;
    canvas.width = width;
    canvas.height = height;
    initNodes();
  }

  window.addEventListener("resize", resize);
  initNodes();

  function draw() {
    ctx.clearRect(0, 0, width, height);

    const gradient = ctx.createRadialGradient(
      width * 0.5, height * 0.2, 0,
      width * 0.5, height * 0.5, Math.max(width, height)
    );
    gradient.addColorStop(0, "rgba(15,23,42,0.7)");
    gradient.addColorStop(1, "rgba(2,6,23,1)");

    ctx.fillStyle = gradient;
    ctx.fillRect(0, 0, width, height);

    // Connect nodes
    for (let i = 0; i < NODE_COUNT; i++) {
      const n1 = nodes[i];
      for (let j = i + 1; j < NODE_COUNT; j++) {
        const n2 = nodes[j];

        const dx = n1.x - n2.x;
        const dy = n1.y - n2.y;
        const dist = Math.sqrt(dx * dx + dy * dy);

        if (dist < MAX_LINK_DIST) {
          const alpha = 1 - dist / MAX_LINK_DIST;
          ctx.strokeStyle = "rgba(129, 140, 248," + alpha * 0.7 + ")";
          ctx.lineWidth = alpha * 1.2;

          ctx.beginPath();
          ctx.moveTo(n1.x, n1.y);
          ctx.lineTo(n2.x, n2.y);
          ctx.stroke();
        }
      }
    }

    // Draw nodes
    nodes.forEach((n) => {
      const size = 1.2 + n.z * 2.5;
      const glow = 0.4 + n.z * 0.8;

      ctx.beginPath();
      ctx.fillStyle = "rgba(224, 231, 255," + glow + ")";
      ctx.arc(n.x, n.y, size, 0, Math.PI * 2);
      ctx.fill();

      const halo = ctx.createRadialGradient(n.x, n.y, 0, n.x, n.y, size * 4);
      halo.addColorStop(0, "rgba(129, 140, 248,0.65)");
      halo.addColorStop(1, "rgba(129, 140, 248,0)");
      ctx.fillStyle = halo;

      ctx.beginPath();
      ctx.arc(n.x, n.y, size * 4, 0, Math.PI * 2);
      ctx.fill();
    });

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
