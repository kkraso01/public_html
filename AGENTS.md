# AI Agent Instructions — Master Index

## Overview
This repository is **Konstantin Krasovitskiy's portfolio** featuring interactive robotics simulations, control algorithms, and creative visualizations. All code is **static** (no build step), **lightweight** (vanilla JS), and designed to run on **GitHub Pages**.

## Repository Structure

```
public_html/
├── .github/
│   └── copilot-instructions.md    ← Root instructions (read this first!)
│
├── drones/                         ← Quadrotor simulations (main focus)
│   ├── physics/AGENTS.md          → Core physics engine (shared by all)
│   ├── stationary/AGENTS.md       → Hover control demo (ETH controller)
│   ├── race/AGENTS.md             → High-speed racing (MPC, trajectory opt)
│   ├── cave/AGENTS.md             → Autonomous SLAM exploration
│   └── obstacles/AGENTS.md        → Reactive avoidance (potential fields)
│
├── maze-solver/AGENTS.md          ← Pathfinding algorithms (A*, LPA*, BFS)
├── line-follower/AGENTS.md        ← PID control simulation (IR sensors)
├── new/AGENTS.md                  ← LLM Dream Reconstructor (creative CV)
│
├── home.html                       → Main portfolio landing page
├── style.css                       → Global styles
└── [various viz scripts]          → Background animations, system visualizers
```

## Quick Navigation

### For New AI Agents
1. **Start here**: Read `.github/copilot-instructions.md` (global conventions)
2. **Working on drones?**: Read `drones/physics/AGENTS.md` first (critical physics conventions)
3. **Specific demo?**: Jump to relevant subfolder AGENTS.md

### For Users
- **Live site**: https://kkraso01.github.io/public_html/
- **Local testing**: `python -m http.server 8000` then open `http://localhost:8000/home.html`
- **Demos**:
  - Stationary hover: `drones/stationary/stationary_test.html`
  - Drone race: Link from `home.html`
  - Cave explorer: Link from `home.html`
  - Maze solver: Link from `home.html`
  - Line follower: Link from `home.html`
  - Dream reconstructor: `new/index.html`

## Critical Global Conventions (MUST FOLLOW)

### 1. Coordinate System (+Z UP Everywhere)
**ALL** drone code uses **+Z UP** (ETH/ENU convention):
```
+X: arbitrary horizontal (often forward)
+Y: perpendicular horizontal (often left)
+Z: UPWARD (altitude)

Gravity: (0, 0, -9.81) m/s² (acts downward in -Z)
```

**DO NOT** use NED (+Z down) without explicit conversion.

### 2. Motor Configuration (Crazyflie X-Frame)
```
Motor layout (all demos):
     0 (CW)          1 (CCW)
       ╲            ╱
        ╲  +X ↑   ╱
         ╲      ╱
     +Y ← ●
         ╱      ╲
        ╱        ╲
     3 (CCW)      2 (CW)

Torque equations (DO NOT CHANGE):
  τ_x = (L/√2)*kF*((ω₀²+ω₃²) - (ω₁²+ω₂²))  [roll]
  τ_y = (L/√2)*kF*((ω₀²+ω₁²) - (ω₂²+ω₃²))  [pitch]
  τ_z = kM*(-ω₀²+ω₁²-ω₂²+ω₃²)              [yaw]
```

### 3. No Build Step (Keep Static)
- Pure vanilla JS (ES6 modules okay)
- No npm, webpack, or bundlers
- Load libraries via CDN with fallbacks
- All paths relative (works on GitHub Pages)

### 4. Performance & Mobile
- Use `dpr` scaling for canvas: `canvas.width = w * dpr; ctx.setTransform(dpr,0,0,dpr,0,0);`
- Implement viewport observer (pause when off-screen)
- Test on mobile (reduce particles, disable shadows)

### 5. Do Not Create Summary Docs
After making changes, **DO NOT** create markdown summaries unless explicitly requested. Just implement the changes.

## Drone Simulations (Primary Focus)

### Shared Physics Engine (`drones/physics/`)
**Read `drones/physics/AGENTS.md` FIRST** before modifying any drone code.

**Files**:
- `drone_physics_engine.js` — 6-DOF dynamics, quaternion integration
- `motor_model.js` — First-order lag (realistic motor response)
- `params_crazyflie.js` — Real hardware specs (DO NOT MODIFY without reason)

**Key Points**:
- Uses +Z up (critical!)
- X-configuration torque equations (correct for Crazyflie)
- Motor spin directions: 0=CW, 1=CCW, 2=CW, 3=CCW
- Thrust limits: `T_max = kF * ω_max²` per motor

### Stationary Hover (`drones/stationary/`)
**Purpose**: Test hovering, waypoint tracking, gain tuning

**Files**:
- `eth_controller.js` — **Primary controller** (ETH SE(3) geometric)
- `mpc_controller.js` — Experimental MPC (not used by default)
- `stationary_controller.js` — Deprecated PID (legacy)
- `stationary_test.html` — Interactive demo page

**Recent Bug Fixes** (Dec 2025):
- ✅ Fixed thrust saturation (was using artificial limits)
- ✅ Removed premature thrust clamp (prevented descent)

**Default Gains**:
```javascript
Kp = [4.0, 4.0, 8.0]   // Position
Kd = [3.0, 3.0, 4.0]   // Velocity
Ki = [0.05, 0.05, 0.20] // Integral
KR = [4.5, 4.5, 1.0]   // Attitude
Kω = [0.10, 0.10, 0.05] // Angular rate
```

### Drone Racing (`drones/race/`)
**Purpose**: High-speed gate navigation, trajectory optimization

**Key Features**:
- Minimum-snap trajectory generation
- Time-optimal planning (binary search on segment times)
- MPC with tilt constraints
- Real-time lap timing

**Tracks**: loop, slalom, helix, figure8

### Cave Exploration (`drones/cave/`)
**Purpose**: Autonomous SLAM-based exploration

**Pipeline**:
1. Lidar scanning (360° × 180°, 3m range)
2. ICP odometry (scan matching)
3. Particle filter localization
4. Occupancy grid + TSDF mapping
5. Frontier detection
6. NBV planning (next-best-view)
7. A* pathfinding
8. ETH controller (nose-first mode)

**Performance Toggles**:
```javascript
window.caveDemo.enableParticleFilter = false; // Use ICP only
window.caveDemo.enableTSDF = false;          // Disable dense reconstruction
window.caveDemo.lidarDensity = 'low';        // 32 rays instead of 128
```

### Obstacle Avoidance (`drones/obstacles/`)
**Purpose**: Reactive collision avoidance (potential fields)

**Features**:
- Drag-and-drop obstacles
- Physics simulation (falling objects)
- Artificial potential field controller
- Real-time safety visualization

## Other Simulations

### Maze Solver (`maze-solver/`)
**Purpose**: Pathfinding algorithm visualization

**Algorithms**:
- BFS (breadth-first, unweighted)
- Dijkstra (uniform cost)
- A* (heuristic-guided)
- LPA* (incremental, dynamic obstacles)

**Features**:
- Oblique pathfinding (8-way movement)
- Motion profiles (trapezoidal, S-curve)
- Real-time algorithm animation

### Line Follower (`line-follower/`)
**Purpose**: PID control education

**Features**:
- 5-sensor IR array simulation
- Real-time gain tuning (Kp, Ki, Kd)
- Anti-windup protection
- Derivative filtering
- Sensor noise simulation

### LLM Dream Reconstructor (`new/`)
**Purpose**: Experimental artistic CV visualization

**Concept**:
- CV content as "hallucinated memories"
- Entropy-driven particle system
- Scroll-reactive narrative
- Token generation aesthetic

## Common Bug Patterns to Avoid

### ❌ Drone Bug #1: Wrong Gravity Sign
```javascript
// WRONG (makes drone fall up):
F_gravity = new THREE.Vector3(0, 0, +mass * gravity);

// CORRECT:
F_gravity = new THREE.Vector3(0, 0, -mass * gravity);
```

### ❌ Drone Bug #2: Flipped Yaw Torque Signs
```javascript
// WRONG (yaw won't work):
τ_z = kM * (+ω₀² - ω₁² + ω₂² - ω₃²);

// CORRECT (matches Crazyflie):
τ_z = kM * (-ω₀² + ω₁² - ω₂² + ω₃²);
```

### ❌ Drone Bug #3: Thrust Limit Too Low
```javascript
// WRONG (artificial saturation):
this.maxThrustPerMotor = (mass * gravity / 4) * 2.0;

// CORRECT (use actual physics):
this.maxThrustPerMotor = params.maxThrustPerMotor ??
                          (this.kF * this.omegaMax * this.omegaMax);
```

### ❌ Canvas Bug: Forgetting DPR Scaling
```javascript
// WRONG (blurry on retina displays):
canvas.width = container.clientWidth;
canvas.height = container.clientHeight;

// CORRECT:
const dpr = window.devicePixelRatio || 1;
canvas.width = container.clientWidth * dpr;
canvas.height = container.clientHeight * dpr;
ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
```

## Development Workflow

### Local Testing
```bash
cd public_html
python -m http.server 8000
# Open http://localhost:8000/home.html
```

### Debugging
- **Browser DevTools**: Console logs, network tab
- **Physics**: Enable debug logging in controller files (uncomment lines)
- **Performance**: Use Chrome DevTools Performance profiler

### Making Changes
1. Read relevant AGENTS.md file first
2. Test locally
3. Verify on mobile (responsive)
4. Check performance (60 FPS target)
5. Commit (no summary docs unless requested)

## Typical Tasks for AI Agents

### Easy (Can Do Immediately)
- Add new trajectory pattern (hover, race)
- Adjust controller gains (tune PID)
- Change visual appearance (colors, models)
- Add telemetry display (HUD elements)
- Implement new maze generation algorithm
- Add new track layout (racing)

### Medium (Need Planning)
- Implement new pathfinding algorithm
- Add obstacle avoidance to existing demo
- Create new visualization mode
- Optimize performance (reduce particles, spatial hashing)
- Add real-time plotting (error graphs)

### Hard (Multi-File Changes)
- Add new control algorithm (iLQR, NMPC)
- Implement multi-drone coordination
- Create 3D terrain navigation
- Add machine learning component
- Build VR mode (immersive demos)

## Testing Checklist (Before Committing)

- [ ] Runs without errors (check browser console)
- [ ] Performance: 30+ FPS on target hardware
- [ ] Mobile: Works on 768px width screen
- [ ] No memory leaks (profiler stable over time)
- [ ] Viewport observer works (pauses when off-screen)
- [ ] All interactive controls functional
- [ ] Reset button returns to initial state
- [ ] Coordinate system matches conventions (+Z up for drones)

## Key References

### Robotics & Control
- **ETH SE(3)**: Lee, "Geometric Tracking Control of a Quadrotor" (2010)
- **Minimum Snap**: Mellinger & Kumar, "Minimum Snap Trajectory Generation" (2011)
- **MPC**: Kamel et al., "Model Predictive Control for Trajectory Tracking" (2017)
- **A\***: Hart et al., "A Formal Basis for Heuristic Pathfinding" (1968)
- **SLAM**: Thrun et al., "Probabilistic Robotics" (2005)

### Creative Coding
- **Particle Systems**: Reeves, "Modeling Fuzzy Objects" (1983)
- **Generative Art**: Pearson, "Generative Art: A Practical Guide" (2011)
- **Nature of Code**: Shiffman (2012)

### Hardware
- **Crazyflie 2.x**: https://www.bitcraze.io/documentation/

## Contact & Support

**Maintainer**: Konstantin Krasovitskiy  
**Email**: kkraso01@ucy.ac.cy  
**GitHub**: https://github.com/kkraso01  
**Institution**: University of Cyprus  
**Degree**: M.Sc. Artificial Intelligence (current), B.Sc. Computer Science (2025)

## Final Notes for AI Agents

1. **Always read the subfolder AGENTS.md** before working in that area
2. **Respect physics conventions** (+Z up, motor layout, torque signs)
3. **Keep it lightweight** (no build step, vanilla JS)
4. **Test on mobile** (performance mode, responsive design)
5. **DO NOT create summary docs** after changes (unless requested)
6. **Use the existing code style** (see copilot-instructions.md)

When in doubt, ask for clarification rather than guessing. The coordinate system and motor conventions are **critical** and getting them wrong breaks everything.

---

**Last Updated**: Dec 2025  
**Status**: Production (actively maintained portfolio)  
**License**: Educational use (portfolio showcase)
