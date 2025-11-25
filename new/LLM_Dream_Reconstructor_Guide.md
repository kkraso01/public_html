# LLM Dream Reconstructor — Portfolio Implementation Guide

**Concept:** Present your portfolio as a hallucinated reconstruction from a dreaming LLM.

---

## 🎯 Concept Overview

The LLM Dream Reconstructor simulates traversing a latent neural dream — where each project appears as a hallucinated memory fragment. The viewer explores unstable token predictions, entropy collapses, and reconstructed attention snapshots.

This concept merges sci-fi visual storytelling with LLM internals — turning your projects into echoes of forgotten prompts, unstable embeddings, and neural breakdowns.

---

## 🧬 Structure

```
/new/
├── index.html
├── dream.js
├── hallucination.css
├── assets/
│   ├── tokens/
│   ├── fragments/
│   └── sounds/
└── projects/
    ├── hallucination_01.md
    ├── hallucination_02.md
    └── hallucination_03.md
```

---

## ✨ Features

- Canvas-based hallucination layer — visual token streams and fading fragments
- Token-by-token animated reveal — no direct display of sections
- Entropy engine — visual instability before stabilization (e.g. pixel glitches)
- Latent scroll engine — scrolling shifts the depth of hallucination
- Text-to-canvas synthesis — project text slowly forms from chaos
- Dream fade logic — previous content decays before new appears

---

## 🛠 HTML (`index.html`)

Use a fullscreen `<canvas>` and a minimal HUD overlay:

```html
<canvas id="dream-core" class="absolute inset-0 z-0"></canvas>
<div id="hud" class="absolute top-4 left-4 text-sm text-green-300 font-mono z-10">
  <p>RECONSTRUCTING MEMORY STACK...</p>
  <p>Token entropy: <span id="entropy">0.98</span></p>
</div>
```

---

## 🎨 CSS (`hallucination.css`)

```css
body {
  margin: 0;
  background: black;
  color: #a0f0ff;
  font-family: monospace;
  overflow: hidden;
}
canvas {
  position: fixed;
  width: 100vw;
  height: 100vh;
  background: radial-gradient(circle at center, #0a0a0a, #000000);
}
#hud {
  backdrop-filter: blur(6px);
  background-color: rgba(0, 0, 0, 0.4);
  padding: 1rem;
  border: 1px solid #333;
}
```

---

## 🧠 JavaScript (`dream.js`)

Simulate token flows + hallucination collapse:

```js
const canvas = document.getElementById("dream-core");
const ctx = canvas.getContext("2d");
canvas.width = window.innerWidth;
canvas.height = window.innerHeight;

let tokens = [];
let entropy = 0.98;

function randomToken() {
  const chars = "abcdefghijklmnopqrstuvwxyz";
  return chars[Math.floor(Math.random() * chars.length)];
}

function draw() {
  ctx.fillStyle = "rgba(0, 0, 0, 0.1)";
  ctx.fillRect(0, 0, canvas.width, canvas.height);
  ctx.fillStyle = "#00ffff";
  ctx.font = "16px monospace";
  tokens.push({ x: Math.random() * canvas.width, y: 0, char: randomToken() });
  tokens = tokens.map(t => {
    ctx.fillText(t.char, t.x, t.y);
    return { ...t, y: t.y + 1 + Math.random() * 2 };
  }).filter(t => t.y < canvas.height);
  document.getElementById("entropy").textContent = entropy.toFixed(2);
  entropy *= 0.999;
  requestAnimationFrame(draw);
}
draw();
```

---

## 📁 Project Content (`/projects/hallucination_01.md`)

```md
# PROJECT: LLM-MS
This system merges GPT-3.5, GPT-4, and Claude via a routing orchestrator.

Visualize as:
- Entropic fog collapsing into three stable model streams
- Vector database pings shown as radial pulses
- Latency fields mapped to distance in canvas space
```

---

## 🔊 Optional Audio

Add subtle ambient sounds (e.g. warped neural hums or sonar pings) to increase immersion.

---

## ✅ Deployment Tips

- No frameworks required
- Works offline
- Add slight scroll lag to reinforce "vector fog" traversal
- Optional: use Web Audio API for deeper LLM hallucination immersion

---

## 🔚 Outcome

You now have a portfolio that feels like you're reconstructing dreams from an LLM’s corrupted memory matrix. Every element is narrative, animated, and impossible to confuse with any grid-based website.

---
