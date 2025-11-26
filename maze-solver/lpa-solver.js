(function (global) {
  const DIRS = global.MAZE_DIRS;
  const DELTAS = global.MAZE_DELTAS;

  /**
   * LPA* (Lifelong Planning A*) Solver
   * Incrementally replans when walls are discovered, avoiding redundant searches
   * Reference: Koenig & Likhachev (2002)
   */
  class LPASolver {
    constructor(maze) {
      this.maze = maze;
      this.size = maze.size;
      console.log('[LPA*] Constructing LPASolver for maze size', this.size);
      this.reset();
    }

    reset() {
      console.log('[LPA*] Reset called');
      this.position = { x: 0, y: 0 };
      this.heading = "east";
      this.knownWalls = this._createWallGrid();
      this._applyBoundaryWalls();
      
      // LPA* specific: maintain g-values and rhs-values
      this.gValues = Array.from({ length: this.size }, () => Array(this.size).fill(Infinity));
      this.rhsValues = Array.from({ length: this.size }, () => Array(this.size).fill(Infinity));
      
      // Goal cell rhs is 0
      for (const key of this.maze.goalCells) {
        const [gx, gy] = key.split(",").map(Number);
        this.rhsValues[gy][gx] = 0;
      }
      
      // Priority queue (open list): store [priority, x, y]
      this.openList = new PriorityQueue();
      this.closedList = new Set();
      this.inQueue = new Set();
      this.lastPath = [];
      this.searchCount = 0;
      this.wallUpdates = [];
      
      // Initialize with goal cells
      for (const key of this.maze.goalCells) {
        const [gx, gy] = key.split(",").map(Number);
        const priority = this._calculateKey(gx, gy);
        this.openList.enqueue([priority, gx, gy]);
        this.inQueue.add(`${gx},${gy}`);
      }
      // Also initialize the start vertex
      this._updateVertex(this.position.x, this.position.y);
    }

    _createWallGrid() {
      return Array.from({ length: this.size }, () =>
        Array.from({ length: this.size }, () => ({ north: false, east: false, south: false, west: false }))
      );
    }

    // Applies boundary walls to the maze grid.
    // Coordinate system: knownWalls[y][x], y=row, x=col, (0,0) is top-left.
    _applyBoundaryWalls() {
      for (let x = 0; x < this.size; x++) {
        // Top row (north boundary)
        this.knownWalls[0][x].north = true;
        // Bottom row (south boundary)
        this.knownWalls[this.size - 1][x].south = true;
      }
      for (let y = 0; y < this.size; y++) {
        // Left column (west boundary)
        this.knownWalls[y][0].west = true;
        // Right column (east boundary)
        this.knownWalls[y][this.size - 1].east = true;
      }
    }

    atGoal() {
      // Debug: log position and goal check
      // console.log('[LPA*] atGoal?', this.position, this.maze.goalCells);
      return this.maze.goalCells.has(`${this.position.x},${this.position.y}`);
    }

    _heuristic(x, y) {
      // Manhattan distance to nearest goal cell
      let minDist = Infinity;
      for (const key of this.maze.goalCells) {
        const [gx, gy] = key.split(",").map(Number);
        const dist = Math.abs(x - gx) + Math.abs(y - gy);
        minDist = Math.min(minDist, dist);
      }
      return minDist;
    }

    _calculateKey(x, y) {
      // console.log('[LPA*] _calculateKey', x, y);
      const h = this._heuristic(x, y);
      const gVal = this.gValues[y][x];
      const rhsVal = this.rhsValues[y][x];
      const minGRhs = Math.min(gVal, rhsVal);
      return [minGRhs + h, minGRhs];
    }

    _updateVertex(x, y) {
      const key = `${x},${y}`;
      // If not goal cell, compute rhs value from predecessors (incoming edges)
      if (!this.maze.goalCells.has(key)) {
        let minPred = Infinity;
        for (const dir of DIRS) {
          // Predecessor: move from neighbor to (x, y) via -dir
          const nx = x - DELTAS[dir].x;
          const ny = y - DELTAS[dir].y;
          if (nx < 0 || ny < 0 || nx >= this.size || ny >= this.size) continue;
          // Only consider if there is no wall between (nx, ny) and (x, y) in that direction
          if (this.knownWalls[ny][nx][dir]) continue;
          minPred = Math.min(minPred, this.gValues[ny][nx] + 1);
        }
        this.rhsValues[y][x] = minPred;
      }
      // If in open list, remove it
      if (this.inQueue.has(key)) {
        this.inQueue.delete(key);
      }
      // If g != rhs, add to open list
      if (this.gValues[y][x] !== this.rhsValues[y][x]) {
        const priority = this._calculateKey(x, y);
        this.openList.enqueue([priority, x, y]);
        this.inQueue.add(key);
      }
    }

    senseWalls() {
      const { x, y } = this.position;
      const real = this.maze.cells[y][x];
      const updates = [];
      
      for (const dir of DIRS) {
        const hasWall = real[dir];
        const wasKnown = this.knownWalls[y][x][dir];
        
        if (hasWall !== wasKnown) {
          this.knownWalls[y][x][dir] = hasWall;
          const nx = x + DELTAS[dir].x;
          const ny = y + DELTAS[dir].y;
          if (nx >= 0 && ny >= 0 && nx < this.size && ny < this.size) {
            const opposite = DIRS[(DIRS.indexOf(dir) + 2) % 4];
            this.knownWalls[ny][nx][opposite] = hasWall;
          }
          updates.push({ dir, x, y, hasWall });
        }
      }
      
      // If walls changed, mark neighbors for recomputation
      if (updates.length > 0) {
        this.wallUpdates.push({ pos: { ...this.position }, updates, step: this.searchCount });
        this._updateVertex(x, y);
        for (const dir of DIRS) {
          if (!this.knownWalls[y][x][dir]) {
            const nx = x + DELTAS[dir].x;
            const ny = y + DELTAS[dir].y;
            if (nx >= 0 && ny >= 0 && nx < this.size && ny < this.size) {
              this._updateVertex(nx, ny);
            }
          }
        }
      }
    }

    _computeShortestPath() {
      this.closedList.clear();
      const MAX_ITERS = 10000;
      let iter = 0;
      const startX = this.position.x;
      const startY = this.position.y;
      while (true) {
        // Termination: openList.topKey() >= calculateKey(start) && rhs(start) == g(start)
        if (this.openList.isEmpty()) break;
        const topKey = this.openList.topKey();
        const startKey = this._calculateKey(startX, startY);
        // Compare both components of the key lexicographically
        const keyLessThan = (a, b) => (a[0] < b[0]) || (a[0] === b[0] && a[1] < b[1]);
        if (!keyLessThan(topKey, startKey) && this.rhsValues[startY][startX] === this.gValues[startY][startX]) {
          break;
        }
        if (iter++ > MAX_ITERS) {
          console.error('[LPA*] _computeShortestPath() exceeded max iterations, breaking to prevent freeze. openList length:', this.openList.items.length);
          break;
        }
        const dq = this.openList.dequeue();
        if (!Array.isArray(dq) || dq.length !== 3) {
          console.error('[LPA*] dequeue returned invalid value:', dq, 'iter:', iter);
          break;
        }
        const [priority, x, y] = dq;
        const key = `${x},${y}`;
        this.inQueue.delete(key);
        // Log current node and queue length
        if (iter % 1000 === 0 || iter < 10) {
          console.log(`[LPA*] iter ${iter} processing node (${x},${y}) priority:`, priority, 'openList length:', this.openList.items.length);
        }
        // Check if key is outdated (compare both key components)
        const newKey = this._calculateKey(x, y);
        if (keyLessThan(newKey, priority)) {
          // Key outdated, re-insert
          this.openList.enqueue([newKey, x, y]);
          this.inQueue.add(key);
          continue;
        }
        // Overconsistent: g > rhs
        if (this.gValues[y][x] > this.rhsValues[y][x]) {
          this.gValues[y][x] = this.rhsValues[y][x];
        } else {
          // Underconsistent: g < rhs or overconsistent
          this.gValues[y][x] = Infinity;
          this._updateVertex(x, y);
        }
        this.closedList.add(key);
        this.searchCount++;
        // Expand neighbors
        for (const dir of DIRS) {
          if (this.knownWalls[y][x][dir]) continue;
          const nx = x + DELTAS[dir].x;
          const ny = y + DELTAS[dir].y;
          if (nx < 0 || ny < 0 || nx >= this.size || ny >= this.size) continue;
          this._updateVertex(nx, ny);
        }
      }
    }

    _chooseDirection(x, y) {
      let bestDir = null;
      let bestScore = Infinity;
      
      for (const dir of DIRS) {
        if (this.knownWalls[y][x][dir]) continue;
        const nx = x + DELTAS[dir].x;
        const ny = y + DELTAS[dir].y;
        if (nx < 0 || ny < 0 || nx >= this.size || ny >= this.size) continue;
        
        const score = this.gValues[ny][nx] + 1;
        if (score < bestScore) {
          bestScore = score;
          bestDir = dir;
        }
      }
      
      return bestDir;
    }

    step() {
      console.log('[LPA*] step() called at', this.position.x, this.position.y);
      
      // Check if already at goal BEFORE doing anything
      if (this.atGoal()) {
        console.log('[LPA*] Already at goal, returning done');
        return { done: true, reason: "goal-reached", path: [] };
      }
      
      this.senseWalls();
      this._computeShortestPath();
      if (this.gValues[this.position.y][this.position.x] === Infinity) {
        return { done: true, reason: "no-path", path: [] };
      }
      const dir = this._chooseDirection(this.position.x, this.position.y);
      if (!dir) {
        return { done: this.atGoal(), reason: "stuck", path: [] };
      }
      this.position = {
        x: this.position.x + DELTAS[dir].x,
        y: this.position.y + DELTAS[dir].y
      };
      this.heading = dir;
      
      // Check if reached goal after movement
      const atGoalNow = this.atGoal();
      console.log('[LPA*] After move to', this.position.x, this.position.y, 'atGoal:', atGoalNow);
      
      return { done: atGoalNow, direction: dir, path: this.lastPath.slice() };
    }

    getSearchFrontier() {
      return Array.from(this.closedList).map(key => {
        const [x, y] = key.split(",").map(Number);
        return { x, y, g: this.gValues[y][x], rhs: this.rhsValues[y][x] };
      });
    }

    getWallUpdateHistory() {
      return this.wallUpdates;
    }
  }

  // Simple Priority Queue for LPA*
  class PriorityQueue {
    constructor() {
      this.items = [];
    }

    enqueue(item) {
      // Remove any existing entry for (x, y)
      const x = item[1], y = item[2];
      this.items = this.items.filter(e => !(e[1] === x && e[2] === y));
      this.items.push(item);
      this.items.sort((a, b) => {
        const [pa0, pa1] = a[0];
        const [pb0, pb1] = b[0];
        if (pa0 !== pb0) return pa0 - pb0;
        return pa1 - pb1;
      });
    }

    topKey() {
      if (this.items.length === 0) return [Infinity, Infinity];
      return this.items[0][0];
    }

    dequeue() {
      // console.log('[PQ] dequeue');
      return this.items.shift();
    }

    isEmpty() {
      return this.items.length === 0;
    }
  }

  global.LPASolver = LPASolver;
})(window);
