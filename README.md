# Konstantin Krasovitskiy — AI Portfolio & Visualization System

A static, production-ready portfolio site showcasing **LLM-MS research**, **AI system architecture**, and **transformer internals** through interactive Canvas visualizations.

**Live**: https://kkraso01.netlify.com/  
**Local**: `python -m http.server 8000` → http://localhost:8000/home.html

---

## 🎯 What's Inside

### 📊 10 Canvas-Based Visualizations

1. **Neural Constellation Background** — Particle network backdrop
2. **RAG Pipeline** — 6-stage retrieval-augmented generation flow
3. **2D System Visualizer** — Orbiting LLM nodes with token particles
4. **3D Neural Galaxy** — 2.5D cyberpunk neural field
5. **Multi-LLM Orchestrator** — Your LLM-MS architecture in action
6. **Transformer Internals** — Token flow through attention & residual blocks
7. **Multi-Head Attention** — Parallel attention heads visualization
8. **System Telemetry Dashboard** — Live metrics HUD
9. **Hero Metrics** — Status badges in hero section
10. **Plugin System** — Extensibility API for all visualizers

### 🏗️ Architecture Highlights

- **No build step** — Pure HTML + Vanilla JS + Canvas
- **Responsive design** — Tailwind CSS + device pixel ratio scaling
- **Mobile-friendly** — Performance mode, adaptive particles
- **Extensible** — `window.SystemPlugins` API for dynamic configuration
- **Portfolio-ready** — Deploys to GitHub Pages / Netlify instantly

---

## 📁 Project Structure

```
/public_html/
├── home.html                         # Main entry point
├── style.css                         # Global styles (Tailwind)
│
├── background.js                     # Neural constellation (background)
├── rag_pipeline.js                   # RAG pipeline stages
├── system_visualizer_2d.js           # 2D orbiting system
├── system_visualizer_3d.js           # 3D neural galaxy
├── system_orchestrator.js            # Multi-LLM orchestrator (LLM-MS)
├── llm_viz.js                        # Transformer block internals
├── attention.js                      # Multi-head attention
├── system_status.js                  # Hero metrics
├── system_dashboard.js               # Telemetry HUD
├── system_plugins.js                 # Plugin/extensibility API
│
├── .github/
│   └── copilot-instructions.md       # AI agent guidance for this codebase
│
├── docs/
│   ├── QUICK_START.md                # Developer quick-start guide
│   ├── VISUALIZER_IMPLEMENTATION_STATUS.md  # Complete feature checklist
│   └── CANVAS_VISUALIZERS_GUIDE.md   # Detailed visualizer docs (optional)
│
└── (deployment files)
    ├── .htaccess                     # Server configuration
    ├── KOSTNANTIN_KRASOVITSKIY_CV_NEW_2025.pdf
    └── Konstantin_Krasovitskiy_CV.md
```

---

## 🚀 Quick Start

### Local Development

```bash
cd public_html
python -m http.server 8000
# Open http://localhost:8000/home.html
```

### Browser Console Playground

```javascript
// Check system status
window.SystemPlugins.info();

// List all registered nodes
console.log(window.SystemPlugins.getNodes());

// Change rendering mode (for mobile)
window.SystemPlugins.setMode('performance');  // or 'normal', 'heavy'

// Simulate system load
window.SystemPlugins.simulateLoad(80);

// Register a custom node
window.SystemPlugins.registerNode('my-llm', {
  type: 'llm',
  model: 'My Model',
  capacity: 5000,
  latency: 45
});
```

---

## 📚 Documentation

### For Developers
- **[QUICK_START.md](docs/QUICK_START.md)** — Setup, debugging, customization, console commands
- **[VISUALIZER_IMPLEMENTATION_STATUS.md](docs/VISUALIZER_IMPLEMENTATION_STATUS.md)** — Feature checklist, integration table, performance notes

### For AI Agents / Copilot
- **[.github/copilot-instructions.md](.github/copilot-instructions.md)** — Architecture overview, conventions, patterns, extensibility

### For Deep Dives
- **[CANVAS_VISUALIZERS_GUIDE.md](CANVAS_VISUALIZERS_GUIDE.md)** — Detailed explanation of each visualizer (how it works, extending, integration)

---

## 🎨 Visualizer Breakdown

### Neural Constellation Background (`background.js`)
- **Purpose**: Subtle, always-on backdrop resembling a neural network
- **Tech**: 70 particles, distance-based connections, depth layering
- **Canvas**: `nn-bg` (fixed, behind content)

### RAG Pipeline (`rag_pipeline.js`)
- **Purpose**: Show retrieval-augmented generation stages
- **Stages**: Query → Embed → Vector DB → Retrieve → LLM → Output
- **Tech**: Animated boxes, dashed arrows, pulsing data packets
- **Canvas**: `ragCanvas`

### 2D System Visualizer (`system_visualizer_2d.js`)
- **Purpose**: Illustrate a central LLM with orbiting models
- **Components**: Core LLM, token particles, vector DB node
- **Tech**: Orbital mechanics, pulsing glows, rotating rings
- **Canvas**: `viz2d`

### 3D Neural Galaxy (`system_visualizer_3d.js`)
- **Purpose**: Represent LLM internal computation as a cosmic field
- **Components**: Orbiting nodes, particle nebula, dynamic connections
- **Tech**: 2.5D projection, wobble animation, pulsing edges
- **Canvas**: `viz3d`

### Multi-LLM Orchestrator (`system_orchestrator.js`)
- **Purpose**: **Your LLM-MS research** — show multi-model routing in action
- **Flow**: Client → Router → (Vector DB + LLM Pool) → Scorer → Response
- **Modes**: Cost-aware (LLM-A), Balanced (LLM-B), Quality (LLM-C)
- **Metrics**: Tokens/s, Latency, Cost profile
- **Tech**: Rounded rectangles, dashed animations, mode cycling
- **Canvas**: `orchestratorCanvas`

### Transformer Internals (`llm_viz.js`)
- **Purpose**: Show token flow through transformer architecture
- **Stages**: Tokens → Embedding → Block1 (Attention) → Block2 (FFN) → Logits
- **Residual**: Animated stream with traveling pulse glow
- **Heads**: 4 attention heads with individual activation
- **Tech**: Grid background, rounded boxes, pulsing connections
- **Canvas**: `llm-viz`

### Multi-Head Attention (`attention.js`)
- **Purpose**: Visualize parallel attention weights
- **Setup**: 4 tokens ("I", "am", "an", "LLM") → 4 heads
- **Interaction**: Attention weights regenerate every 1.2s
- **Tech**: Dynamic line weights, alpha modulation
- **Canvas**: `attentionCanvas`

### System Dashboard (`system_dashboard.js`)
- **Purpose**: Live telemetry overlay showing system state
- **Metrics**: Node (ACTIVE/IDLE), Latency, Tokens/s, GPU Load, Vector Ops
- **Update**: Every 500ms with simulated jitter
- **Position**: Fixed top-right corner

### Hero Metrics (`system_status.js`)
- **Purpose**: Display system status in hero section
- **Updates**: Tokens/s and latency badges
- **Frequency**: Every 600ms

### Plugin System (`system_plugins.js`)
- **Purpose**: Enable runtime extensibility
- **API**: `registerNode()`, `registerAnimation()`, `setMode()`, `simulateLoad()`, `export()`, `info()`
- **Global**: `window.SystemPlugins` namespace

---

## 🔧 Key Patterns & Conventions

### Canvas Setup (Every Visualizer)

```javascript
const canvas = document.getElementById("myCanvas");
const ctx = canvas.getContext("2d");

let w, h, dpr;

function resize() {
  dpr = window.devicePixelRatio || 1;
  const rect = canvas.getBoundingClientRect();
  w = rect.width;
  h = rect.height;
  canvas.width = w * dpr;
  canvas.height = h * dpr;
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);  // Critical: scale transforms
}

resize();
window.addEventListener("resize", resize);

function animate() {
  ctx.clearRect(0, 0, w, h);
  // Draw here
  requestAnimationFrame(animate);
}

animate();
```

### Colors

HSL-based for dynamic theming:
- **Indigo**: `rgba(129, 140, 248, α)`
- **Cyan**: `rgba(34, 211, 238, α)`
- **Mint**: `rgba(52, 211, 153, α)`
- **Pink**: `rgba(244, 114, 182, α)`

### HTML Canvas Integration

```html
<section id="my-viz" class="py-24 px-6 relative max-w-6xl mx-auto">
  <h2 class="text-3xl font-bold mb-6">My Visualizer</h2>
  <p class="text-gray-400 mb-6">Description</p>
  <canvas id="myCanvas" class="w-full h-[420px] rounded-2xl bg-gray-950/40 border border-gray-700"></canvas>
</section>

<script src="my-visualizer.js"></script>
```

---

## 🚀 Performance & Mobile

### Device Pixel Ratio (DPR) Scaling
- All canvases use `dpr = window.devicePixelRatio || 1`
- Essential for crisp rendering on retina displays
- Applied via `ctx.setTransform(dpr, 0, 0, dpr, 0, 0)`

### Performance Mode
```javascript
// For low-end devices
window.SystemPlugins.setMode('performance');  // Reduces particles & nodes
```

### Mobile Testing
1. Open DevTools (`F12`)
2. Toggle device toolbar (`Ctrl+Shift+M`)
3. Test different viewport sizes
4. Check FPS with `setMode('performance')`

---

## 🎓 How This Showcases Your Work

| Visualizer | Demonstrates |
|------------|--------------|
| RAG Pipeline | Your RAG/LLM expertise |
| 2D System | Multi-component orchestration |
| 3D Galaxy | Cyberpunk aesthetic research brand |
| **Orchestrator** | **Your core LLM-MS research** |
| Transformer | Deep LLM architecture knowledge |
| Attention | Understanding of attention mechanisms |
| Dashboard | Systems thinking & metrics |
| Plugin System | Software engineering maturity |

---

## 🔄 Deployment

### GitHub Pages / Netlify

1. Push repo to GitHub
2. Connect to Netlify
3. Build command: (leave empty — static site)
4. Publish directory: `/public_html`

### Custom Domain

Update `netlify.toml` or GitHub Pages settings to point to your domain.

---

## 🤝 Extending the Visualizers

### Add a New Visualizer

1. Create `my_visualizer.js` in `/public_html`
2. Add HTML section to `home.html` with `<canvas id="myCanvas">`
3. Add script tag at bottom of `home.html`
4. Use standard DPR + resize + requestAnimationFrame pattern

### Customize Colors

Find HSL values in any visualizer and replace, e.g.:
```javascript
// Old
ctx.strokeStyle = "rgba(129,140,248,0.9)";

// New (magenta theme)
ctx.strokeStyle = "rgba(236,72,153,0.9)";
```

### Connect to Real Data

```javascript
// Example: Fetch real metrics
fetch('/api/system-metrics')
  .then(r => r.json())
  .then(metrics => {
    // Update visualizer based on real data
    telemetry.latency = metrics.latency;
  });
```

---

## 📖 Resources

- **Tailwind CSS**: https://tailwindcss.com (already via CDN)
- **Canvas API**: https://developer.mozilla.org/en-US/docs/Web/API/Canvas_API
- **RequestAnimationFrame**: https://developer.mozilla.org/en-US/docs/Web/API/window/requestAnimationFrame

---

## ✅ Checklist for Agents

- [ ] Understand all 10 visualizers are self-contained & modular
- [ ] Know that `.github/copilot-instructions.md` has specific patterns
- [ ] Always include DPR scaling in canvas setup
- [ ] Always add resize handlers
- [ ] Use `window.SystemPlugins` for extensibility instead of modifying core
- [ ] Test on mobile via DevTools device toolbar
- [ ] Keep static (no server required)
- [ ] Verify no console errors before committing

---

## 🎯 Summary

This is a **portfolio-grade visualization system** that:

✅ Runs **instantly** (no build step)  
✅ Scales to **mobile** (responsive + DPR)  
✅ **Extensible** via plugins  
✅ **Showcases your research** (LLM-MS orchestrator)  
✅ **Production-ready** (deploys to GitHub Pages / Netlify)  
✅ **Well-documented** for AI agents & developers  

---

**Questions?** See `docs/QUICK_START.md` or check `.github/copilot-instructions.md` for AI agent workflows.

**Last Updated**: November 16, 2025
