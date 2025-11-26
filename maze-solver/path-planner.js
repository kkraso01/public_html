(function (global) {
  const DIRS = global.MAZE_DIRS;
  const DELTAS = global.MAZE_DELTAS;

  /**
   * PathPlanner: Computes time-optimal paths using full maze knowledge
   * Builds graph over (x,y,heading) with time-based edge costs
   */
  class PathPlanner {
    constructor(maze, motionProfile) {
      this.maze = maze;
      this.motionProfile = motionProfile;
      this.size = maze.size;
      this.knownWalls = null; // Will be set from solver
      this.graph = null;
    }

    /**
     * Set the wall map from exploration
     */
    setWallMap(wallMap) {
      this.knownWalls = wallMap;
      this._buildGraph();
    }

    /**
     * Build graph: nodes are {x, y, heading}, edges are moves with time costs
     */
    _buildGraph() {
      this.graph = new Map(); // key: "x,y,heading", value: {neighbors: [{to: "x,y,h", cost: time, action: desc}]}

      for (let y = 0; y < this.size; y++) {
        for (let x = 0; x < this.size; x++) {
          for (const heading of DIRS) {
            const nodeKey = `${x},${y},${heading}`;
            const neighbors = [];

            // Straight moves: 1 to 3 cells ahead (reduce to avoid long jumps)
            for (let len = 1; len <= 3; len++) {
              const nx = x + DELTAS[heading].x * len;
              const ny = y + DELTAS[heading].y * len;
              if (nx < 0 || ny < 0 || nx >= this.size || ny >= this.size) break;

              // Check if path is clear. Unknown walls (undefined) are treated as blocked
              // so we never "guess" a shortcut through unexplored cells.
              let blocked = false;
              for (let i = 0; i < len; i++) {
                const cx = x + DELTAS[heading].x * i;
                const cy = y + DELTAS[heading].y * i;
                const cellWalls = this.knownWalls?.[cy]?.[cx];
                const hasWall = cellWalls ? cellWalls[heading] : undefined;
                if (hasWall !== false) { // true or unknown -> block
                  blocked = true;
                  break;
                }
              }
              if (blocked) break;

              // Time cost for straight segment
              const profile = this.motionProfile.profileSegment(len, false);
              const cost = profile.totalTime;
              const toKey = `${nx},${ny},${heading}`;
              neighbors.push({ to: toKey, cost, action: `straight ${len} ${heading}` });
            }

            // Turns: 90 degrees left/right (in place for simplicity)
            const turnCost = 0.2; // seconds for a turn, estimate
            const leftHeading = DIRS[(DIRS.indexOf(heading) + 3) % 4]; // left turn
            const rightHeading = DIRS[(DIRS.indexOf(heading) + 1) % 4]; // right turn

            neighbors.push({ to: `${x},${y},${leftHeading}`, cost: turnCost, action: `turn left to ${leftHeading}` });
            neighbors.push({ to: `${x},${y},${rightHeading}`, cost: turnCost, action: `turn right to ${rightHeading}` });

            this.graph.set(nodeKey, neighbors);
          }
        }
      }
    }

    /**
     * Compute shortest time path from start to any goal cell
     */
    computeOptimalPath(startX = 0, startY = 0, startHeading = "east") {
      if (!this.graph) return null;

      const startKey = `${startX},${startY},${startHeading}`;
      const dist = new Map();
      const prev = new Map();
      const pq = new PriorityQueue();

      // Initialize
      for (const key of this.graph.keys()) {
        dist.set(key, Infinity);
      }
      dist.set(startKey, 0);
      pq.enqueue([0, startKey]);

      while (!pq.isEmpty()) {
        const [cost, current] = pq.dequeue();
        if (cost > dist.get(current)) continue;

        // Check if at goal
        const [x, y] = current.split(',').map(Number);
        if (this.maze.goalCells.has(`${x},${y}`)) {
          return this._reconstructPath(prev, current);
        }

        for (const neighbor of this.graph.get(current) || []) {
          const alt = cost + neighbor.cost;
          if (alt < dist.get(neighbor.to)) {
            dist.set(neighbor.to, alt);
            prev.set(neighbor.to, { from: current, action: neighbor.action });
            pq.enqueue([alt, neighbor.to]);
          }
        }
      }

      return null; // No path found
    }

    _reconstructPath(prev, goalKey) {
      const path = [];
      let current = goalKey;
      while (current) {
        const entry = prev.get(current);
        if (!entry) break;
        path.unshift({ state: current, action: entry.action });
        current = entry.from;
      }
      return path;
    }

    /**
     * Compress path into macro segments for execution
     */
    compressPath(statePath) {
      const segments = [];
      if (!statePath || statePath.length < 2) return segments;

      let currentStraight = null;

      for (const step of statePath) {
        const [x, y, heading] = step.state.split(',');
        if (step.action && step.action.startsWith('straight')) {
          const len = parseInt(step.action.split(' ')[1]);
          if (currentStraight && currentStraight.dir === heading) {
            currentStraight.cells += len;
          } else {
            if (currentStraight) segments.push(currentStraight);
            currentStraight = { type: 'straight', dir: heading, cells: len };
          }
        } else {
          if (currentStraight) {
            segments.push(currentStraight);
            currentStraight = null;
          }
          if (step.action && step.action.includes('turn')) {
            segments.push({ type: 'turn', dir: step.action.includes('left') ? 'left' : 'right' });
          }
        }
      }

      if (currentStraight) segments.push(currentStraight);

      return segments;
    }
  }

  // Simple Priority Queue
  class PriorityQueue {
    constructor() {
      this.items = [];
    }

    enqueue(item) {
      this.items.push(item);
      this.items.sort((a, b) => a[0] - b[0]);
    }

    dequeue() {
      return this.items.shift();
    }

    isEmpty() {
      return this.items.length === 0;
    }
  }

  global.PathPlanner = PathPlanner;
})(window);