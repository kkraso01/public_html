# AI Agent Instructions — Konstantin Krasovitskiy Portfolio

This repo is a static portfolio site (HTML + Vanilla JS + Canvas). Keep changes lightweight and portable — the site is designed to run without a build step so it can be easily hosted on GitHub Pages / Netlify.

## Big Picture (Architecture)
- Static site with self-contained canvas visualizations in JS files:
  - `home.html` — main page with sections and embedded canvases
  - `background.js` — neural network background animation
  - `rag_pipeline.js` — RAG pipeline stages with pulsing arrows
  - `system_visualizer_2d.js` — 2D GPT-4 inspired system with orbiting LLM nodes and token particles
  - `system_visualizer_3d.js` — 2.5D neural galaxy with particles and connections
  - `system_orchestrator.js` — Multi-LLM orchestration layer visualization (LLM-MS architecture demo)
  - `llm_viz.js` — Transformer block internals showing token flow, attention heads, residual stream, logits
  - `attention.js` — Multi-head attention mechanism with dynamic head-to-head connections
  - `system_plugins.js` — plugin API for extensibility
  - `system_status.js` & `system_dashboard.js` — simulated telemetry HUD
  - `new/dream.js` — LLM Dream Reconstructor with token hallucination engine for CV visualization
- Approach: each visual script manages its own canvas, `dpr` scaling (e.g., `dpr = window.devicePixelRatio || 1; canvas.width = w * dpr; ctx.setTransform(dpr, 0, 0, dpr, 0, 0);`), `resize` handler, and `requestAnimationFrame` loop.

## Developer Workflows
- Local test server: `cd public_html && python -m http.server 8000`, open `http://localhost:8000/home.html` or `http://localhost:8000/new/index.html` for the LLM Dream Reconstructor.
- Debugging: use browser DevTools (console logs present; `window.SystemPlugins.info()` for plugin state).
- No CI or unit tests; verify visually in browser. No build step.

## Conventions & Patterns
- Canvas setup: always include `dpr` scaling and `window.addEventListener("resize", resize);` for responsiveness.
- Animations: use `requestAnimationFrame` loops; clear canvas with `ctx.clearRect(0, 0, w, h);`.
- Extensibility: use `window.SystemPlugins.registerNode(name, config)` or `registerAnimation` instead of modifying core files.
- Colors: prefer HSL for dynamic hues (e.g., `hsla(${hue}, 100%, 70%, 0.8)`).
- Modularity: scripts are self-contained; avoid global pollution beyond `window.SystemPlugins`.

## Extensibility (Plugin API)
- Register nodes: `window.SystemPlugins.registerNode('llm-gpt4', {type:'llm', model:'GPT-4', capacity:1000});`
- Test modes: `window.SystemPlugins.setMode('performance');` or `simulateLoad(80);`
- Console example:
  ```js
  window.SystemPlugins.registerNode('demo-node', {type:'llm', model:'test', latency:10});
  window.SystemPlugins.simulateLoad(80);
  window.SystemPlugins.info();
  ```

## Performance & Mobile
- Use `setMode('performance')` to reduce particles on mobile/low-end devices.
- For heavy visuals, check `window.devicePixelRatio` and adjust particle counts.

## Typical Tasks for Agents
- Small: add new visual elements, tweak palettes, improve mobile responsiveness.
- Medium: integrate optional CDN (e.g., Three.js) with fallbacks.
- Large: add build step (e.g., for modules) with `package.json`, README, and CI.

## What *Not* to Do
- Avoid heavy tooling or frameworks without maintainer approval.
- Don’t assume server-side; keep static and lightweight.

If unclear (e.g., adding Three.js or CI), ask for guidance.

— End of Instructions
