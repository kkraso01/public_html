# AI Agent Instructions — LLM Dream Reconstructor

## Overview
Experimental **token hallucination engine** that visualizes CV content as a "dream sequence" from an LLM's perspective. Features **entropy-driven particle systems**, **scroll-reactive animations**, and **hallucinated text fragments** that reconstruct the portfolio owner's experience and achievements.

## Purpose
- **Artistic Visualization**: Represent LLM token generation as visual art
- **CV Presentation**: Display professional experience as "hallucinated memories"
- **Interactive Narrative**: Scroll-driven reveal of project fragments
- **Experimental UI**: Non-traditional portfolio presentation

## Conceptual Framework

### The Metaphor
Imagine an LLM "dreaming" about the portfolio owner's CV. As it generates tokens, it hallucinates fragments of experience, projects, and skills. The visualization shows:
- **Tokens**: Particles representing neural activations
- **Entropy**: Randomness in token generation (temperature)
- **Reconstruction**: Gradual assembly of coherent memories (CV sections)
- **Hallucination**: Visual distortion of reality (glitch effects)

### Visual Language
```
High Entropy (top of page):
  → Chaotic particle motion
  → Fragmented text
  → Rapid color shifts
  → "LLM is confused"

Low Entropy (scrolled down):
  → Coherent structures
  → Complete sentences
  → Stable colors
  → "LLM has locked onto memory"
```

## Architecture

### System Flow
```
┌──────────────┐
│ Scroll Event │ → User scrolls down page
└──────┬───────┘   Updates scrollDepth (0-1)
       │
       ▼
┌──────────────┐
│ Entropy Calc │ → entropy = 0.98 - scrollDepth * 0.7
│              │   Higher scroll = lower entropy
└──────┬───────┘   Affects particle behavior
       │
       ▼
┌──────────────┐
│ Particle Sys │ → Spawn/update tokens (particles)
│              │   Motion influenced by entropy
└──────┬───────┘   Color shifts, blur effects
       │
       ▼
┌──────────────┐
│ Text Reveal  │ → Hallucinate CV fragments
│              │   Typewriter effect
└──────┬───────┘   Project cards appear
       │
       ▼
┌──────────────┐
│ Canvas Render│ → Draw particles, connections
└──────────────┘   Apply visual effects
```

## Files

### `dream.js` (214 lines) — Main Engine
- **Lines 1-50**: Canvas setup, particle initialization
- **Lines 50-100**: Scroll handler, entropy calculation
- **Lines 100-150**: Particle update loop
- **Lines 150-214**: Rendering (particles, connections, effects)

### `index.html` — Page Structure
- Hero section (title, tagline)
- Scrollable CV content (projects, skills, experience)
- Canvas overlay (fullscreen, pointer-events: none)

### `hallucination.css` — Styling
- Glitch animations
- Fade-in effects
- Responsive layout
- Dark theme (CV as "dream in darkness")

### `projects/hallucination_XX.md` — Content Fragments
- Individual project descriptions
- Formatted as "hallucinated memories"
- Injected into DOM on scroll

## Particle System

### Token Representation
```javascript
class Token {
  constructor(x, y) {
    this.x = x;
    this.y = y;
    this.vx = (Math.random() - 0.5) * 2;
    this.vy = (Math.random() - 0.5) * 2;
    this.lifetime = Math.random() * 100 + 50; // frames
    this.age = 0;
    this.size = Math.random() * 3 + 1;
    this.hue = Math.random() * 360; // Rainbow
  }
  
  update(entropy) {
    // High entropy = chaotic motion
    const chaos = entropy * 5;
    this.vx += (Math.random() - 0.5) * chaos;
    this.vy += (Math.random() - 0.5) * chaos;
    
    // Friction (low entropy = particles settle)
    const friction = 1 - (1 - entropy) * 0.1;
    this.vx *= friction;
    this.vy *= friction;
    
    // Update position
    this.x += this.vx;
    this.y += this.vy;
    
    // Age
    this.age++;
    
    // Boundary wrap
    if (this.x < 0) this.x = canvas.width;
    if (this.x > canvas.width) this.x = 0;
    if (this.y < 0) this.y = canvas.height;
    if (this.y > canvas.height) this.y = 0;
  }
  
  draw(ctx, entropy) {
    const alpha = 1 - (this.age / this.lifetime);
    const blurAmount = entropy * 3; // High entropy = more blur
    
    ctx.save();
    ctx.filter = `blur(${blurAmount}px)`;
    ctx.fillStyle = `hsla(${this.hue}, 100%, 60%, ${alpha * 0.8})`;
    ctx.beginPath();
    ctx.arc(this.x, this.y, this.size, 0, Math.PI * 2);
    ctx.fill();
    ctx.restore();
  }
  
  isDead() {
    return this.age >= this.lifetime;
  }
}
```

### Particle Spawning
```javascript
function spawnTokens(count) {
  for (let i = 0; i < count; i++) {
    const x = Math.random() * canvas.width;
    const y = Math.random() * canvas.height;
    tokens.push(new Token(x, y));
  }
}

// Adaptive spawn rate
const spawnRate = entropy > 0.5 ? 5 : 2; // More chaos = more particles
if (Math.random() < spawnRate / 60) {
  spawnTokens(1);
}
```

### Connections (Neural Network Aesthetic)
```javascript
function drawConnections(ctx, tokens, entropy) {
  const maxDistance = 100; // Connect nearby tokens
  
  for (let i = 0; i < tokens.length; i++) {
    for (let j = i + 1; j < tokens.length; j++) {
      const dx = tokens[i].x - tokens[j].x;
      const dy = tokens[i].y - tokens[j].y;
      const dist = Math.sqrt(dx * dx + dy * dy);
      
      if (dist < maxDistance) {
        const alpha = (1 - dist / maxDistance) * (1 - entropy) * 0.3;
        ctx.strokeStyle = `rgba(100, 200, 255, ${alpha})`;
        ctx.lineWidth = 1;
        ctx.beginPath();
        ctx.moveTo(tokens[i].x, tokens[i].y);
        ctx.lineTo(tokens[j].x, tokens[j].y);
        ctx.stroke();
      }
    }
  }
}
```

## Entropy-Driven Behavior

### Scroll to Entropy Mapping
```javascript
window.addEventListener('scroll', () => {
  const maxScroll = document.body.scrollHeight - window.innerHeight;
  scrollDepth = Math.min(1, window.scrollY / maxScroll);
  
  // Entropy decreases as user scrolls (LLM "locks on" to memories)
  entropy = 0.98 - scrollDepth * 0.7; // Range: 0.98 → 0.28
});
```

### Effects of Entropy

**High Entropy (0.9+)**: "LLM is confused"
- Rapid particle motion (high chaos factor)
- Few connections (particles too scattered)
- Rapid color shifts
- Heavy blur
- Text fragments incomplete

**Medium Entropy (0.5-0.7)**: "LLM is searching"
- Moderate particle motion
- Some connections form
- Colors stabilize
- Text becomes readable

**Low Entropy (0.3-)**: "LLM has clarity"
- Particles settle into stable patterns
- Dense connection network (neural lattice)
- Fixed color palette
- No blur
- Complete sentences, project cards

## Text Hallucination

### CV Content Structure
```javascript
const hallucinations = [
  {
    title: "IDENTITY FRAGMENT — KONSTANTIN KRASOVITSKIY",
    section: "contact",
    content: "Software Engineer | Researcher | AI Systems Architect..."
  },
  {
    title: "NEURAL MILESTONE — LLM-MS",
    section: "publication",
    content: "Multi-Model LLM Search Engine — published at..."
  },
  // ... more fragments
];
```

### Progressive Reveal
```javascript
let currentProjectIndex = 0;

function revealNextProject() {
  if (currentProjectIndex >= hallucinations.length) return;
  
  const project = hallucinations[currentProjectIndex];
  const card = createProjectCard(project);
  
  // Typewriter effect
  let charIndex = 0;
  const interval = setInterval(() => {
    if (charIndex < project.content.length) {
      card.textContent += project.content[charIndex];
      charIndex++;
    } else {
      clearInterval(interval);
    }
  }, 20); // 20ms per character
  
  document.getElementById('projects').appendChild(card);
  currentProjectIndex++;
}

// Trigger on scroll milestones
if (scrollDepth > 0.2 && currentProjectIndex === 0) revealNextProject();
if (scrollDepth > 0.4 && currentProjectIndex === 1) revealNextProject();
// ...
```

### Glitch Effect (High Entropy)
```css
@keyframes glitch {
  0% { transform: translate(0); }
  20% { transform: translate(-2px, 2px); }
  40% { transform: translate(2px, -2px); }
  60% { transform: translate(-1px, 1px); }
  80% { transform: translate(1px, -1px); }
  100% { transform: translate(0); }
}

.project-card.high-entropy {
  animation: glitch 0.3s infinite;
  filter: blur(2px);
}
```

## Rendering Loop

### Main Animation Loop
```javascript
function animate() {
  // Clear canvas with fade (motion blur effect)
  ctx.fillStyle = `rgba(10, 10, 20, ${entropy > 0.5 ? 0.2 : 0.05})`;
  ctx.fillRect(0, 0, canvas.width, canvas.height);
  
  // Spawn new tokens
  if (Math.random() < (entropy > 0.5 ? 5 : 2) / 60) {
    spawnTokens(1);
  }
  
  // Update particles
  tokens = tokens.filter(token => {
    token.update(entropy);
    token.draw(ctx, entropy);
    return !token.isDead();
  });
  
  // Draw connections (only when entropy is low)
  if (entropy < 0.7) {
    drawConnections(ctx, tokens, entropy);
  }
  
  // Update project cards
  updateProjectVisuals(entropy);
  
  requestAnimationFrame(animate);
}
```

## Interactive Features

### Scroll-Driven Narrative
- **0% scroll**: Title, abstract concept
- **20% scroll**: First project appears
- **40% scroll**: Second project, skills section
- **60% scroll**: Experience timeline
- **80% scroll**: Publications, achievements
- **100% scroll**: Contact, final clarity

### Hover Effects
- Particles repelled by cursor (future enhancement)
- Project cards glow on hover
- Interactive entropy control (slider in footer)

## Common Tasks for AI Agents

### Easy
- Add new CV sections (education, awards)
- Change color palette (hue ranges)
- Adjust entropy curve (different scroll mapping)
- Add sound effects (token spawn, connections)

### Medium
- Implement cursor interaction (particle repulsion)
- Add 3D particles (Three.js integration)
- Create branching narratives (choose-your-own-adventure CV)
- Implement LLM API integration (real GPT-4 token generation)

### Hard
- Real-time LLM streaming (tokens generated live)
- Semantic clustering (group related projects)
- VR mode (immersive hallucination space)
- Generative music (sonify token patterns)

## Performance Optimization

### Particle Limits
```javascript
const MAX_TOKENS = 300; // Cap particle count
if (tokens.length > MAX_TOKENS) {
  tokens = tokens.slice(-MAX_TOKENS); // Keep newest
}
```

### Render Culling
```javascript
// Only draw particles in viewport
tokens = tokens.filter(token => {
  const inViewport = token.x >= 0 && token.x <= canvas.width &&
                     token.y >= 0 && token.y <= canvas.height;
  return inViewport || !token.isDead();
});
```

### Mobile Optimization
```javascript
// Reduce particle count on mobile
const isMobile = window.innerWidth < 768;
const particleBudget = isMobile ? 100 : 300;
```

## Artistic Decisions

### Why "Hallucination"?
- LLMs generate text through probabilistic sampling (hallucination)
- Visualization mirrors this: particles = tokens, chaos = uncertainty
- CV as "reconstructed memory" from neural activations

### Color Symbolism
- **Blue/Cyan**: Technical skills (cold, logical)
- **Purple/Magenta**: Creative projects (warm, artistic)
- **Green**: Experience (growth, progression)
- **Red**: Errors, edge cases (warning, attention)

### Entropy as Narrative Device
- High entropy = abstract, conceptual (viewer unsure what they're seeing)
- Low entropy = concrete, factual (clear CV presentation)
- Scroll = "model learning about me" (confidence increases)

## Testing Checklist

- [ ] Particles spawn and move smoothly (60 FPS)
- [ ] Entropy decreases with scroll
- [ ] Text reveals progressively
- [ ] No memory leaks (particle count stable)
- [ ] Responsive on mobile (fewer particles)
- [ ] Canvas resizes on window resize
- [ ] Project cards appear at correct scroll milestones
- [ ] Glitch effects apply at high entropy

## Known Issues

### Issue #1: Particles Accumulate (Memory Leak)
**Symptom**: FPS drops over time  
**Cause**: Dead particles not removed  
**Fix**: Filter `tokens` array on each frame (already implemented)

### Issue #2: Text Illegible at High Entropy
**Symptom**: Can't read CV content at top of page  
**Cause**: Blur too strong  
**Fix**: Reduce blur amount or disable for text elements

### Issue #3: Connections Too Dense
**Symptom**: Canvas becomes white mesh  
**Cause**: Too many particles within connection distance  
**Fix**: Increase `maxDistance` or reduce particle count

## Dependencies

- **Canvas API**: 2D particle rendering
- **Vanilla JS**: No frameworks (lightweight)
- **CSS Animations**: Glitch effects, fades
- No external libraries (self-contained)

## Future Enhancements

- [ ] Real LLM integration (OpenAI API)
- [ ] WebGL renderer (particle instancing)
- [ ] Audio visualization (frequency → particle motion)
- [ ] Multiplayer mode (shared hallucination space)
- [ ] Export as video (screen recording API)

## References

- **LLM Hallucinations**: Ji et al., "Survey of Hallucination in NLP" (2023)
- **Particle Systems**: Reeves, "Particle Systems—A Technique for Modeling Fuzzy Objects" (1983)
- **Generative Art**: Pearson, "Generative Art: A Practical Guide" (2011)
- **Creative Coding**: Shiffman, "The Nature of Code" (2012)

---

**Last Updated**: Dec 2025  
**Maintainer**: Konstantin Krasovitskiy  
**Status**: Experimental (artistic portfolio)
