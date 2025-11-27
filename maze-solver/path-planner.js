(function (global) {
  const DIRS = global.MAZE_DIRS;  // ["north", "east", "south", "west"]
  const DELTAS = global.MAZE_DELTAS;

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
      // Validate opposite walls: wall[x][y].east === wall[x+1][y].west
      this._enforceWallSymmetry();
      this._buildGraph();
    }

    /**
     * Enforce wall symmetry: if (x,y) has east wall, then (x+1,y) must have west wall
     * This prevents disconnected graph caused by wall inconsistencies
     */
    _enforceWallSymmetry() {
      for (let y = 0; y < this.size; y++) {
        for (let x = 0; x < this.size; x++) {
          // Check east wall
          if (x + 1 < this.size && this.knownWalls[y][x].east === true) {
            this.knownWalls[y][x + 1].west = true;
          }
          // Check west wall
          if (x - 1 >= 0 && this.knownWalls[y][x].west === true) {
            this.knownWalls[y][x - 1].east = true;
          }
          // Check south wall
          if (y + 1 < this.size && this.knownWalls[y][x].south === true) {
            this.knownWalls[y + 1][x].north = true;
          }
          // Check north wall
          if (y - 1 >= 0 && this.knownWalls[y][x].north === true) {
            this.knownWalls[y - 1][x].south = true;
          }
        }
      }
    }

    /**
     * Build 4D graph: nodes are (x, y, heading), edges are straights and turns with time costs
     * CARDINAL ONLY: no diagonals allowed
     */
    _buildGraph() {
      this.graph = new Map(); // key: "x,y,heading", value: [neighbors]

      for (let y = 0; y < this.size; y++) {
        for (let x = 0; x < this.size; x++) {
          for (const heading of DIRS) {
            const nodeKey = `${x},${y},${heading}`;
            const neighbors = [];

            // Forward straights: 1 to 3 cells in current heading
            const dir = DELTAS[heading];
            for (let len = 1; len <= 3; len++) {
              const targetX = x + dir.x * len;
              const targetY = y + dir.y * len;
              
              // Bounds check
              if (targetX < 0 || targetY < 0 || targetX >= this.size || targetY >= this.size) break;

              // Check if this straight is legal: validate walls for each step
              let canMove = true;
              
              for (let step = 0; step < len; step++) {
                const currentX = x + dir.x * step;
                const currentY = y + dir.y * step;
                const nextX = currentX + dir.x;
                const nextY = currentY + dir.y;

                // Wall check: can we exit currentCell in this heading?
                if (this._isBlocked(currentY, currentX, heading)) {
                  canMove = false;
                  break;
                }

                // Wall check: can we enter nextCell from opposite direction?
                const oppositeHeading = this._getOppositeHeading(heading);
                if (this._isBlocked(nextY, nextX, oppositeHeading)) {
                  canMove = false;
                  break;
                }
              }

              if (!canMove) break; // This length blocked, and longer lengths will also be blocked

              // Valid straight move
              const profile = this.motionProfile.profileSegment(len, false);
              const cost = profile.totalTime;
              const toKey = `${targetX},${targetY},${heading}`;
              neighbors.push({ to: toKey, cost, action: `straight ${len} ${heading}` });
            }

            // Turns: left 90° and right 90° (only cardinal, never diagonal)
            // CRITICAL: Only allow turns into positions that don't have a wall directly ahead
            const turnCost = this.motionProfile.getTurnTime(90);
            const headingIdx = DIRS.indexOf(heading);
            
            // Left turn: counter-clockwise
            const leftHeading = DIRS[(headingIdx + 3) % 4];
            // CRITICAL FIX: Unknown walls must be treated as walls
            if (!this._isBlocked(y, x, leftHeading)) {
              neighbors.push({ to: `${x},${y},${leftHeading}`, cost: turnCost, action: `turn left to ${leftHeading}` });
            }

            // Right turn: clockwise
            const rightHeading = DIRS[(headingIdx + 1) % 4];
            // CRITICAL FIX: Unknown walls must be treated as walls
            if (!this._isBlocked(y, x, rightHeading)) {
              neighbors.push({ to: `${x},${y},${rightHeading}`, cost: turnCost, action: `turn right to ${rightHeading}` });
            }

            this.graph.set(nodeKey, neighbors);
          }
        }
      }
    }

    _getOppositeHeading(heading) {
      const opposites = { north: "south", south: "north", east: "west", west: "east" };
      return opposites[heading];
    }

    /**
     * Check if a wall is blocked in a given direction
     * CRITICAL: unknown walls (undefined) are treated as walls (default safe behavior)
     * @param {number} y - cell y coordinate
     * @param {number} x - cell x coordinate
     * @param {string} dir - direction (north, south, east, west)
     * @returns {boolean} true if blocked/wall, false if open
     */
    _isBlocked(y, x, dir) {
      const cell = this.knownWalls[y][x];
      if (!cell) return true;              // unknown cell = wall
      const w = cell[dir];
      return w === true || w === undefined; // unknown wall = wall
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
     * Convert state path into discrete grid cells (one per step)
     * CRITICAL FIX for BUG 3: Handle segment expansion correctly
     * The Dijkstra path records states at ENDPOINTS of moves, not at starts
     * We must reconstruct the full cell sequence by advancing from current position
     */
    statePathToGridPath(statePath) {
      const gridPath = [];
      if (!statePath || statePath.length === 0) return gridPath;

      // Start at origin
      gridPath.push({ x: 0, y: 0 });

      // Expand each move into individual cells
      for (let i = 0; i < statePath.length; i++) {
        const step = statePath[i];
        if (step.action && step.action.startsWith('straight')) {
          // Parse action: "straight N heading"
          const parts = step.action.split(' ');
          const len = parseInt(parts[1]);
          const moveHeading = parts[2];
          
          const dir = DELTAS[moveHeading];
          if (!dir) continue;

          // Get current position from gridPath (the last added cell)
          const lastCell = gridPath[gridPath.length - 1];
          
          // Add each cell in this straight run
          for (let j = 1; j <= len; j++) {
            const cellX = lastCell.x + dir.x * j;
            const cellY = lastCell.y + dir.y * j;
            
            // Avoid duplicates
            if (gridPath.length === 0 || 
                gridPath[gridPath.length - 1].x !== cellX || 
                gridPath[gridPath.length - 1].y !== cellY) {
              gridPath.push({ x: cellX, y: cellY });
            }
          }
        }
        // Turns don't add new cells, just change heading
      }

      return gridPath;
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