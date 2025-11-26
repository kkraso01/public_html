(function (global) {
  const DIRS = global.MAZE_DIRS;
  const DELTAS = global.MAZE_DELTAS;

  const DIAGONAL_DIRS = [
    { name: "north", dx: 0, dy: -1, diagonal: false, components: ["north"] },
    { name: "east", dx: 1, dy: 0, diagonal: false, components: ["east"] },
    { name: "south", dx: 0, dy: 1, diagonal: false, components: ["south"] },
    { name: "west", dx: -1, dy: 0, diagonal: false, components: ["west"] },
    { name: "northEast", dx: 1, dy: -1, diagonal: true, components: ["north", "east"] },
    { name: "southEast", dx: 1, dy: 1, diagonal: true, components: ["south", "east"] },
    { name: "southWest", dx: -1, dy: 1, diagonal: true, components: ["south", "west"] },
    { name: "northWest", dx: -1, dy: -1, diagonal: true, components: ["north", "west"] }
  ];

  /**
   * PathPlanner: Computes time-optimal paths using full maze knowledge
   * Builds graph over (x,y,heading) with time-based edge costs
   */
  class PathPlanner {
    constructor(maze, motionProfile, options = {}) {
      this.maze = maze;
      this.motionProfile = motionProfile;
      this.allowOblique = Boolean(options.allowOblique);
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

    _diagonalMoveAllowed(x, y, dir) {
      const primary = dir.components;
      const nx = x + dir.dx;
      const ny = y + dir.dy;
      if (nx < 0 || ny < 0 || nx >= this.size || ny >= this.size) return false;

      if (primary.some(d => this.knownWalls[y][x][d])) return false;

      const checks = [];
      if (dir.name === "northEast") {
        checks.push([x + 1, y, "north"], [x, y - 1, "east"]);
      } else if (dir.name === "southEast") {
        checks.push([x + 1, y, "south"], [x, y + 1, "east"]);
      } else if (dir.name === "southWest") {
        checks.push([x - 1, y, "south"], [x, y + 1, "west"]);
      } else if (dir.name === "northWest") {
        checks.push([x - 1, y, "north"], [x, y - 1, "west"]);
      }

      return checks.every(([cx, cy, facing]) => cx >= 0 && cy >= 0 && cx < this.size && cy < this.size && !this.knownWalls[cy][cx][facing]);
    }

    /**
     * Build graph: nodes are {x, y, heading}, edges are moves with time costs
     */
    _buildGraph() {
      this.graph = new Map(); // key: "x,y,heading", value: {neighbors: [{to: "x,y,h", cost: time, action: desc}]}

      const allDirs = this.allowOblique ? DIAGONAL_DIRS : DIRS.map(name => ({ name, dx: DELTAS[name].x, dy: DELTAS[name].y, diagonal: false, components: [name] }));

      for (let y = 0; y < this.size; y++) {
        for (let x = 0; x < this.size; x++) {
          for (const heading of allDirs) {
            const nodeKey = `${x},${y},${heading.name}`;
            const neighbors = [];

            // Straight moves: 1 to 3 cells ahead
            for (let len = 1; len <= 3; len++) {
              const nx = x + heading.dx * len;
              const ny = y + heading.dy * len;
              if (nx < 0 || ny < 0 || nx >= this.size || ny >= this.size) break;

              // Check if path is clear
              let blocked = false;
              for (let i = 0; i < len; i++) {
                const cx = x + heading.dx * i;
                const cy = y + heading.dy * i;
                const cellWalls = this.knownWalls?.[cy]?.[cx];
                if (heading.diagonal) {
                  if (len > 1) {
                    blocked = true;
                    break;
                  }
                  if (!this._diagonalMoveAllowed(cx, cy, heading)) {
                    blocked = true;
                    break;
                  }
                } else {
                  const hasWall = cellWalls ? cellWalls[heading.name] : undefined;
                  if (hasWall !== false) {
                    blocked = true;
                    break;
                  }
                }
              }
              if (blocked) break;

              // Time cost for straight segment
              const profile = this.motionProfile.profileSegment(len, false);
              const cost = profile.totalTime;
              const toKey = `${nx},${ny},${heading.name}`;
              neighbors.push({ to: toKey, cost, action: `straight ${len} ${heading.name}` });
            }

            // Turns: 90 degrees left/right (only for cardinal headings)
            if (!heading.diagonal) {
              const turnCost = this.motionProfile.getTurnTime(90);
              const leftHeading = DIRS[(DIRS.indexOf(heading.name) + 3) % 4];
              const rightHeading = DIRS[(DIRS.indexOf(heading.name) + 1) % 4];

              neighbors.push({ to: `${x},${y},${leftHeading}`, cost: turnCost, action: `turn left to ${leftHeading}` });
              neighbors.push({ to: `${x},${y},${rightHeading}`, cost: turnCost, action: `turn right to ${rightHeading}` });
            }

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

  // Min Heap for Priority Queue
  class MinHeap {
    constructor() {
      this.heap = [];
    }

    push(item) {
      this.heap.push(item);
      this._bubbleUp(this.heap.length - 1);
    }

    pop() {
      if (this.heap.length === 1) return this.heap.pop();
      const root = this.heap[0];
      this.heap[0] = this.heap.pop();
      this._sinkDown(0);
      return root;
    }

    isEmpty() {
      return this.heap.length === 0;
    }

    _bubbleUp(index) {
      while (index > 0) {
        const parentIndex = Math.floor((index - 1) / 2);
        if (this.heap[index][0] >= this.heap[parentIndex][0]) break;
        [this.heap[index], this.heap[parentIndex]] = [this.heap[parentIndex], this.heap[index]];
        index = parentIndex;
      }
    }

    _sinkDown(index) {
      const length = this.heap.length;
      while (true) {
        let left = 2 * index + 1;
        let right = 2 * index + 2;
        let smallest = index;
        if (left < length && this.heap[left][0] < this.heap[smallest][0]) smallest = left;
        if (right < length && this.heap[right][0] < this.heap[smallest][0]) smallest = right;
        if (smallest === index) break;
        [this.heap[index], this.heap[smallest]] = [this.heap[smallest], this.heap[index]];
        index = smallest;
      }
    }
  }

  // Simple Priority Queue (replaced with MinHeap)
  class PriorityQueue {
    constructor() {
      this.heap = new MinHeap();
    }

    enqueue(item) {
      this.heap.push(item);
    }

    dequeue() {
      return this.heap.pop();
    }

    isEmpty() {
      return this.heap.isEmpty();
    }
  }

  global.PathPlanner = PathPlanner;
})(window);