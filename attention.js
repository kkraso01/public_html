const canvas = document.getElementById("attentionCanvas");
const ctx = canvas.getContext("2d");

const W = canvas.width;
const H = canvas.height;

// Token positions
const TOKENS = ["I", "am", "an", "LLM"];
const tokenX = [60, 140, 220, 320];
const tokenY = 200;

// 4 attention heads
const HEADS = 4;

// random attention weights per head (4x4)
let att = [];

function randomMatrix() {
  let m = [];
  for (let h = 0; h < HEADS; h++) {
    const head = [];
    for (let i = 0; i < TOKENS.length; i++) {
      head.push(
        Array.from({ length: TOKENS.length }, () =>
          Math.random() * 0.9 + 0.1
        )
      );
    }
    m.push(head);
  }
  return m;
}

// regenerate every 1.2s
function rebuild() {
  att = randomMatrix();
}
setInterval(rebuild, 1200);
rebuild();

function draw() {
  ctx.clearRect(0, 0, W, H);

  // Draw tokens
  ctx.font = "18px monospace";
  ctx.textAlign = "center";
  ctx.fillStyle = "white";

  for (let i = 0; i < TOKENS.length; i++) {
    ctx.fillText(TOKENS[i], tokenX[i], tokenY);
  }

  // Draw heads title
  ctx.font = "14px monospace";
  ctx.fillStyle = "rgb(129,140,248)";
  ctx.fillText("Multi-Head Attention", W / 2, 22);

  // Draw attention for each head
  for (let h = 0; h < HEADS; h++) {
    const yOffset = 40 + h * 30;

    // head label
    ctx.fillStyle = `rgba(129,140,248,0.8)`;
    ctx.font = "12px monospace";
    ctx.fillText(`Head ${h + 1}`, 40, yOffset);

    // attention connections
    for (let i = 0; i < TOKENS.length; i++) {
      for (let j = 0; j < TOKENS.length; j++) {
        const w = att[h][i][j];

        ctx.strokeStyle = `rgba(79,70,229, ${w * 0.8})`;
        ctx.lineWidth = w * 2;

        ctx.beginPath();
        ctx.moveTo(tokenX[i], tokenY - 10);
        ctx.lineTo(tokenX[j], yOffset);
        ctx.stroke();
      }
    }

    // draw head output dots
    for (let j = 0; j < TOKENS.length; j++) {
      ctx.beginPath();
      ctx.fillStyle = "rgba(255,255,255,0.8)";
      ctx.arc(tokenX[j], yOffset, 4, 0, Math.PI * 2);
      ctx.fill();
    }
  }

  requestAnimationFrame(draw);
}

draw();
