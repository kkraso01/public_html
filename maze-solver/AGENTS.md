# AI Agent Instructions — Maze Solver

## Overview
Advanced pathfinding visualization featuring **multiple search algorithms**, **oblique path planning**, **smooth motion profiles**, and **real-time maze generation**. Demonstrates various pathfinding techniques from simple BFS to sophisticated LPA*.

## Purpose
- **Algorithm Comparison**: BFS, Dijkstra, A*, LPA* (incremental search)
- **Path Optimization**: Oblique pathfinding (diagonal movement)
- **Motion Control**: Smooth velocity profiles with acceleration limits
- **Interactive Visualization**: Real-time algorithm animation, editable mazes

## Architecture

### System Components
```
┌─────────────┐
│ Maze Gen    │ → Recursive backtracker, Prim's algorithm
└──────┬──────┘   Configurable density, size
       │
       ▼
┌─────────────┐
│ Pathfinding │ → BFS (breadth-first, unweighted)
│             │   Dijkstra (uniform cost)
│             │   A* (heuristic-guided)
└──────┬──────┘   LPA* (incremental, dynamic)
       │
       ▼
┌─────────────┐
│ Path Smooth │ → Oblique motion (8-way movement)
│             │   Corner cutting (diagonal shortcuts)
└──────┬──────┘   Waypoint reduction
       │
       ▼
┌─────────────┐
│ Motion Plan │ → Trapezoidal velocity profile
│             │   Acceleration constraints
└──────┬──────┘   Smooth curves (Bézier splines)
       │
       ▼
┌─────────────┐
│ Rendering   │ → Canvas 2D visualization
└─────────────┘   Color-coded cells, animated agent
```

## Files and Components

### Core Files
**`maze-explorer.js`** (885 lines) — Main pathfinding algorithms
- MinHeap priority queue (Dijkstra, A*)
- Diagonal pathfinding (8-directional)
- Heuristic functions (Manhattan, Euclidean, Chebyshev)
- Path reconstruction from parent map

**`maze.js`** — Maze generation and representation
- Recursive backtracker algorithm
- Prim's algorithm (minimum spanning tree)
- Eller's algorithm (row-by-row generation)
- Grid data structure (cells, walls)

**`solver.js`** — High-level solver interface
- Algorithm selection (BFS, Dijkstra, A*, LPA*)
- Step-by-step execution
- Metrics tracking (nodes explored, path length)

**`lpa-solver.js`** — Lifelong Planning A*
- Incremental search (reuses previous results)
- Dynamic obstacle updates
- Efficiently handles maze changes

**`path-planner.js`** — Standard grid-based planning
- 4-way movement (cardinal directions)
- Simple A* implementation
- No diagonal movement

**`path-planner-oblique.js`** — Advanced diagonal planning
- 8-way movement (includes diagonals)
- Corner-cutting detection
- Smooth path generation

**`motion-profile.js`** — Velocity profile generation
- Trapezoidal profiles (accel → cruise → decel)
- S-curve smoothing (jerk limits)
- Time-optimal planning

**`view.js`** — Visualization and rendering
- Canvas drawing (cells, walls, paths)
- Animation loop (agent movement)
- Color schemes (heat maps, visited nodes)

**`advanced-renderer.js`** — Enhanced visuals
- Gradient rendering (cost-to-go visualization)
- Particle effects (exploration wavefront)
- Path smoothing display

## Pathfinding Algorithms

### BFS (Breadth-First Search)
**Type**: Unweighted graph search  
**Complexity**: O(V + E) time, O(V) space  
**Guarantees**: Shortest path (unweighted)

```javascript
function bfs(start, goal, grid) {
  const queue = [start];
  const visited = new Set();
  const parent = new Map();
  
  while (queue.length > 0) {
    const current = queue.shift();
    if (current === goal) break;
    
    for (const neighbor of getNeighbors(current)) {
      if (!visited.has(neighbor) && !isWall(neighbor)) {
        visited.add(neighbor);
        parent.set(neighbor, current);
        queue.push(neighbor);
      }
    }
  }
  
  return reconstructPath(parent, start, goal);
}
```

**Pros**: Simple, complete, optimal (unweighted)  
**Cons**: No heuristic, explores uniformly

### Dijkstra's Algorithm
**Type**: Uniform cost search  
**Complexity**: O((V + E) log V) with binary heap  
**Guarantees**: Shortest path (weighted)

```javascript
function dijkstra(start, goal, grid) {
  const heap = new MinHeap();
  heap.push([0, start]); // [cost, node]
  
  const costs = new Map([[start, 0]]);
  const parent = new Map();
  
  while (!heap.isEmpty()) {
    const [cost, current] = heap.pop();
    if (current === goal) break;
    
    for (const neighbor of getNeighbors(current)) {
      const newCost = cost + edgeCost(current, neighbor);
      if (newCost < (costs.get(neighbor) ?? Infinity)) {
        costs.set(neighbor, newCost);
        parent.set(neighbor, current);
        heap.push([newCost, neighbor]);
      }
    }
  }
  
  return reconstructPath(parent, start, goal);
}
```

**Pros**: Optimal, handles weighted graphs  
**Cons**: Slower than A* (no heuristic)

### A\* (A-Star)
**Type**: Heuristic search  
**Complexity**: O(b^d) worst case, O(d) with good heuristic  
**Guarantees**: Optimal (with admissible heuristic)

```javascript
function aStar(start, goal, grid, heuristic) {
  const heap = new MinHeap();
  heap.push([heuristic(start, goal), 0, start]); // [f, g, node]
  
  const gScores = new Map([[start, 0]]);
  const parent = new Map();
  
  while (!heap.isEmpty()) {
    const [f, g, current] = heap.pop();
    if (current === goal) break;
    
    for (const neighbor of getNeighbors(current)) {
      const tentativeG = g + edgeCost(current, neighbor);
      if (tentativeG < (gScores.get(neighbor) ?? Infinity)) {
        gScores.set(neighbor, tentativeG);
        parent.set(neighbor, current);
        const h = heuristic(neighbor, goal);
        heap.push([tentativeG + h, tentativeG, neighbor]);
      }
    }
  }
  
  return reconstructPath(parent, start, goal);
}
```

**Heuristics**:
```javascript
// Manhattan (4-way movement)
h = |x1 - x2| + |y1 - y2|

// Euclidean (any movement)
h = √((x1 - x2)² + (y1 - y2)²)

// Chebyshev (8-way movement)
h = max(|x1 - x2|, |y1 - y2|)

// Octile (8-way with diagonal cost √2)
dx = |x1 - x2|
dy = |y1 - y2|
h = (dx + dy) + (√2 - 2) * min(dx, dy)
```

**Pros**: Fast, optimal, goal-directed  
**Cons**: Requires good heuristic, not incremental

### LPA\* (Lifelong Planning A\*)
**Type**: Incremental heuristic search  
**Complexity**: O(d log d) per update (amortized)  
**Guarantees**: Optimal, efficient for dynamic environments

```javascript
class LPAStar {
  constructor(start, goal, grid) {
    this.start = start;
    this.goal = goal;
    this.g = new Map(); // Actual cost
    this.rhs = new Map(); // One-step lookahead cost
    this.open = new MinHeap();
    
    this.rhs.set(start, 0);
    this.open.push([this.key(start), start]);
  }
  
  key(node) {
    const gVal = this.g.get(node) ?? Infinity;
    const rhsVal = this.rhs.get(node) ?? Infinity;
    return [
      Math.min(gVal, rhsVal) + this.heuristic(node, this.goal),
      Math.min(gVal, rhsVal)
    ];
  }
  
  updateVertex(node) {
    if (node !== this.start) {
      const minRhs = Math.min(...this.getSuccessors(node).map(s =>
        (this.g.get(s) ?? Infinity) + this.cost(node, s)
      ));
      this.rhs.set(node, minRhs);
    }
    
    // Update priority queue
    this.open.remove(node);
    if (this.g.get(node) !== this.rhs.get(node)) {
      this.open.push([this.key(node), node]);
    }
  }
  
  computeShortestPath() {
    while (!this.open.isEmpty() && 
           this.open.topKey() < this.key(this.goal) ||
           this.rhs.get(this.goal) !== this.g.get(this.goal)) {
      const [k, u] = this.open.pop();
      
      if (this.g.get(u) > this.rhs.get(u)) {
        this.g.set(u, this.rhs.get(u));
        for (const s of this.getSuccessors(u)) {
          this.updateVertex(s);
        }
      } else {
        this.g.set(u, Infinity);
        this.updateVertex(u);
        for (const s of this.getSuccessors(u)) {
          this.updateVertex(s);
        }
      }
    }
  }
  
  updateMap(changedCells) {
    // Efficiently handle obstacles appearing/disappearing
    for (const cell of changedCells) {
      for (const neighbor of this.getNeighbors(cell)) {
        this.updateVertex(neighbor);
      }
    }
    this.computeShortestPath();
  }
}
```

**Pros**: Efficient for dynamic maps, reuses computation  
**Cons**: More complex, higher memory usage

## Oblique Pathfinding

### Diagonal Movement
```javascript
const DIAGONAL_DIRS = [
  { dx: 0, dy: -1 },  // North
  { dx: 1, dy: 0 },   // East
  { dx: 0, dy: 1 },   // South
  { dx: -1, dy: 0 },  // West
  { dx: 1, dy: -1 },  // NorthEast (diagonal)
  { dx: 1, dy: 1 },   // SouthEast (diagonal)
  { dx: -1, dy: 1 },  // SouthWest (diagonal)
  { dx: -1, dy: -1 }  // NorthWest (diagonal)
];

const DIAGONAL_COST = Math.sqrt(2); // ~1.414
const CARDINAL_COST = 1.0;
```

### Corner Cutting Detection
```javascript
function isDiagonalBlocked(grid, x, y, dx, dy) {
  // For diagonal move (dx, dy), check if either cardinal is blocked
  // Example: NorthEast (1, -1) requires North (0, -1) AND East (1, 0) to be free
  const horizontal = grid[y][x + dx];
  const vertical = grid[y + dy][x];
  
  return horizontal === WALL || vertical === WALL;
}
```

### Path Smoothing
```javascript
function smoothPath(path, grid) {
  if (path.length <= 2) return path;
  
  const smoothed = [path[0]];
  let current = 0;
  
  while (current < path.length - 1) {
    // Try to skip intermediate waypoints (line-of-sight)
    let farthest = current + 1;
    for (let i = current + 2; i < path.length; i++) {
      if (hasLineOfSight(grid, path[current], path[i])) {
        farthest = i;
      } else {
        break;
      }
    }
    smoothed.push(path[farthest]);
    current = farthest;
  }
  
  return smoothed;
}

function hasLineOfSight(grid, from, to) {
  // Bresenham's line algorithm
  const dx = Math.abs(to.x - from.x);
  const dy = Math.abs(to.y - from.y);
  const sx = from.x < to.x ? 1 : -1;
  const sy = from.y < to.y ? 1 : -1;
  let err = dx - dy;
  
  let x = from.x, y = from.y;
  while (true) {
    if (grid[y][x] === WALL) return false;
    if (x === to.x && y === to.y) return true;
    
    const e2 = 2 * err;
    if (e2 > -dy) { err -= dy; x += sx; }
    if (e2 < dx) { err += dx; y += sy; }
  }
}
```

## Motion Profiles

### Trapezoidal Velocity Profile
```javascript
function trapezoidalProfile(distance, vMax, aMax) {
  // Time to accelerate to max velocity
  const tAccel = vMax / aMax;
  const dAccel = 0.5 * aMax * tAccel * tAccel;
  
  if (2 * dAccel >= distance) {
    // Triangular profile (never reach vMax)
    const tPeak = Math.sqrt(distance / aMax);
    return {
      phases: [
        { type: 'accel', duration: tPeak, vFinal: aMax * tPeak },
        { type: 'decel', duration: tPeak, vFinal: 0 }
      ],
      totalTime: 2 * tPeak
    };
  } else {
    // True trapezoidal (cruise phase exists)
    const dCruise = distance - 2 * dAccel;
    const tCruise = dCruise / vMax;
    return {
      phases: [
        { type: 'accel', duration: tAccel, vFinal: vMax },
        { type: 'cruise', duration: tCruise, vFinal: vMax },
        { type: 'decel', duration: tAccel, vFinal: 0 }
      ],
      totalTime: 2 * tAccel + tCruise
    };
  }
}
```

### S-Curve (Jerk-Limited)
```javascript
function sCurveProfile(distance, vMax, aMax, jMax) {
  // Time to reach max acceleration
  const tJerk = aMax / jMax;
  const dJerk = (1/6) * jMax * tJerk * tJerk * tJerk;
  
  // 7-phase profile: jerk-accel-cruise-jerk-constant-jerk-decel-jerk
  // (Simplified for brevity)
}
```

## Interactive Features

### Maze Editing
- **Left Click**: Toggle wall/floor
- **Right Click**: Set start/goal
- **Drag**: Paint walls
- **Shift + Drag**: Erase walls

### Algorithm Controls
- **Algorithm Selector**: Dropdown (BFS, Dijkstra, A*, LPA*)
- **Speed Slider**: Animation speed (1x - 100x)
- **Step Button**: Single-step through algorithm
- **Reset**: Clear visited nodes, keep maze
- **New Maze**: Generate random maze

### Visualization Modes
- **Visited Nodes**: Color by exploration order (gradient)
- **Cost Map**: Heat map of g-scores
- **Parent Arrows**: Show tree structure
- **Path Highlight**: Thick line for final path

## Common Tasks for AI Agents

### Easy
- Add new maze generation algorithm
- Implement different heuristics
- Change color scheme
- Add telemetry display (nodes explored, time)

### Medium
- Implement D* Lite (similar to LPA*)
- Add bidirectional search
- Create animated path following
- Implement jump point search (JPS)

### Hard
- Any-angle pathfinding (Theta*)
- Multi-agent pathfinding (conflict resolution)
- 3D maze solver
- Learning-based heuristic (neural network)

## Performance Considerations

### Large Mazes
- **Grid Size**: 100×100 = 10,000 cells (comfortable)
- **Max Recommended**: 200×200 = 40,000 cells
- **Bottleneck**: Heap operations in Dijkstra/A*

### Optimization
```javascript
// Use typed arrays for large grids
const grid = new Uint8Array(width * height);

// Spatial hashing for neighbor lookup
const cellIndex = y * width + x;

// Pre-allocate data structures
const visited = new Uint8Array(width * height);
```

## Debugging Tips

### Visualize Search Frontier
```javascript
// Render open set (blue), closed set (gray)
for (const node of openSet) {
  ctx.fillStyle = 'rgba(0, 0, 255, 0.3)';
  ctx.fillRect(node.x * cellSize, node.y * cellSize, cellSize, cellSize);
}
```

### Log Algorithm Metrics
```javascript
console.log(`Nodes explored: ${visited.size}`);
console.log(`Path length: ${path.length}`);
console.log(`Time: ${elapsedTime}ms`);
console.log(`Optimality: ${path.length / optimalLength}`);
```

## References

- **A\***: Hart et al., "A Formal Basis for the Heuristic Determination of Minimum Cost Paths" (1968)
- **LPA\***: Koenig & Likhachev, "Lifelong Planning A*" (2004)
- **D\* Lite**: Koenig & Likhachev, "Fast Replanning for Navigation in Unknown Terrain" (2005)
- **JPS**: Harabor & Grastien, "Online Graph Pruning for Pathfinding on Grid Maps" (2011)

---

**Last Updated**: Dec 2025  
**Maintainer**: Konstantin Krasovitskiy  
**Status**: Production (educational demo)
