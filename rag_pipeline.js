const ragCanvas = document.getElementById("ragCanvas");
const ctx = ragCanvas.getContext("2d");

let w, h, dpr;

function resizeRAG() {
  dpr = window.devicePixelRatio || 1;
  const rect = ragCanvas.getBoundingClientRect();
  w = rect.width;
  h = rect.height;
  ragCanvas.width = w * dpr;
  ragCanvas.height = h * dpr;
  ctx.scale(dpr, dpr);
}

resizeRAG();
window.addEventListener("resize", resizeRAG);

const stages = [
  { name: "Query", color: "#60a5fa" },
  { name: "Embed", color: "#818cf8" },
  { name: "Vector DB", color: "#34d399" },
  { name: "Retrieve", color: "#a78bfa" },
  { name: "LLM", color: "#f472b6" },
  { name: "Output", color: "#38bdf8" }
];

let t = 0;

function drawRAG() {
  t += 0.01;
  ctx.clearRect(0, 0, w, h);

  const boxW = w / 7;
  const centerY = h / 2;

  stages.forEach((stage, i) => {
    const x = boxW * (i + 1);

    ctx.fillStyle = "rgba(15,23,42,0.9)";
    ctx.strokeStyle = stage.color;
    ctx.lineWidth = 2;

    ctx.beginPath();
    ctx.roundRect(x - boxW / 2, centerY - 30, boxW, 60, 10);
    ctx.fill();
    ctx.stroke();

    ctx.fillStyle = stage.color;
    ctx.font = "14px monospace";
    ctx.textAlign = "center";
    ctx.fillText(stage.name, x, centerY + 5);

    if (i < stages.length - 1) {
      const ax = x + boxW / 2;
      const ay = centerY;

      ctx.strokeStyle = stage.color;
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(ax, ay);
      ctx.lineTo(ax + boxW / 1.2, ay);
      ctx.stroke();

      const pulseX = ax + ((t + i * 0.2) % 1) * boxW / 1.2;
      ctx.fillStyle = stage.color;
      ctx.beginPath();
      ctx.arc(pulseX, ay, 4, 0, Math.PI * 2);
      ctx.fill();
    }
  });

  requestAnimationFrame(drawRAG);
}

drawRAG();
