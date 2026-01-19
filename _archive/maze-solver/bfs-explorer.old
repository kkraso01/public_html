(function (global) {
  const DIRS = global.MAZE_DIRS;
  const DELTAS = global.MAZE_DELTAS;

  /**
   * BFSExplorer: Explores the maze using Breadth-First Search
   * Systematically discovers all cells while respecting walls
   */
  class BFSExplorer {
    constructor(maze) {
      this.maze = maze;
      this.size = maze.size;
      this.knownWalls = this._createWallGrid();
      this.distances = [];
      this.position = { x: 0, y: 0 };
      this.heading = "east";
      this.atGoal = false;
      this.visited = new Set();
      this.visited.add('0,0');
    }

    _createWallGrid() {
      const walls = [];
      for (let y = 0; y < this.size; y++) {
        walls[y] = [];
        for (let x = 0; x < this.size; x++) {
          walls[y][x] = {};
          // Initialize boundary walls
          if (x === 0) walls[y][x].west = true;
          if (y === 0) walls[y][x].north = true;
          if (x === this.size - 1) walls[y][x].east = true;
          if (y === this.size - 1) walls[y][x].south = true;
        }
      }
      return walls;
    }

    /**
     * Compute distances using flood-fill from goal
     */
    _computeDistances() {
      this.distances = [];
      for (let y = 0; y < this.size; y++) {
        this.distances[y] = new Array(this.size).fill(Infinity);
      }

      // Set goal cells to 0
      this.maze.goalCells.forEach(key => {
        const [x, y] = key.split(',').map(Number);
        this.distances[y][x] = 0;
      });

      // Flood fill respecting known walls
      let changed = true;
      while (changed) {
        changed = false;
        for (let y = 0; y < this.size; y++) {
          for (let x = 0; x < this.size; x++) {
            if (this.distances[y][x] === Infinity) continue;
            for (const dir of DIRS) {
              const dx = DELTAS[dir].x;
              const dy = DELTAS[dir].y;
              const nx = x + dx;
              const ny = y + dy;
              if (nx >= 0 && ny >= 0 && nx < this.size && ny < this.size) {
                // Only traverse if no wall blocking
                if (!this.knownWalls[y][x][dir] && this.distances[ny][nx] > this.distances[y][x] + 1) {
                  this.distances[ny][nx] = this.distances[y][x] + 1;
                  changed = true;
                }
              }
            }
          }
        }
      }
    }

    /**
     * Sense walls at current position
     */
    _senseWalls() {
      const { x, y } = this.position;
      for (const dir of DIRS) {
        const dx = DELTAS[dir].x;
        const dy = DELTAS[dir].y;
        const nx = x + dx;
        const ny = y + dy;
        if (nx >= 0 && ny >= 0 && nx < this.size && ny < this.size) {
          const hasWall = this.maze.cells[y][x][dir];
          this.knownWalls[y][x][dir] = hasWall;
        }
      }
    }

    /**
     * Find unvisited cell closest to current position
     */
    _findNearestUnvisited() {
      let best = null;
      let bestDist = Infinity;
      
      for (let y = 0; y < this.size; y++) {
        for (let x = 0; x < this.size; x++) {
          const key = `${x},${y}`;
          if (!this.visited.has(key)) {
            const dist = Math.abs(x - this.position.x) + Math.abs(y - this.position.y);
            if (dist < bestDist) {
              bestDist = dist;
              best = { x, y, dist };
            }
          }
        }
      }
      return best;
    }

    /**
     * BFS to find path to target, respecting known walls
     */
    _findPathBFS(targetX, targetY) {
      const visited = new Set();
      const queue = [{ x: this.position.x, y: this.position.y, path: [] }];
      visited.add(`${this.position.x},${this.position.y}`);
      
      while (queue.length > 0) {
        const current = queue.shift();
        
        if (current.x === targetX && current.y === targetY) {
          return current.path;
        }
        
        for (const dir of DIRS) {
          const dx = DELTAS[dir].x;
          const dy = DELTAS[dir].y;
          const nx = current.x + dx;
          const ny = current.y + dy;
          const key = `${nx},${ny}`;
          
          // Check bounds and wall
          if (nx >= 0 && ny >= 0 && nx < this.size && ny < this.size &&
              !visited.has(key) && !this.knownWalls[current.y][current.x][dir]) {
            visited.add(key);
            queue.push({
              x: nx,
              y: ny,
              path: [...current.path, { x: nx, y: ny, dir }]
            });
          }
        }
      }
      
      return null;
    }

    /**
     * Step: execute one exploration step
     */
    step() {
      // Sense walls at current position
      this._senseWalls();

      // Check if at goal
      if (this.maze.goalCells.has(`${this.position.x},${this.position.y}`)) {
        this.atGoal = true;
        console.log("[BFSExplorer] Reached goal!");
        return { done: true };
      }

      // Recompute distances with latest wall knowledge
      this._computeDistances();

      // Find best neighbor (lowest distance to goal)
      let bestDir = null;
      let bestDist = this.distances[this.position.y][this.position.x];

      for (const dir of DIRS) {
        const dx = DELTAS[dir].x;
        const dy = DELTAS[dir].y;
        const nx = this.position.x + dx;
        const ny = this.position.y + dy;
        
        if (nx >= 0 && ny >= 0 && nx < this.size && ny < this.size) {
          // Only consider if no wall blocks
          if (!this.knownWalls[this.position.y][this.position.x][dir]) {
            const dist = this.distances[ny][nx];
            if (dist < bestDist) {
              bestDist = dist;
              bestDir = dir;
            }
          }
        }
      }

      // If we can reach goal, do it
      if (bestDir) {
        this.position.x += DELTAS[bestDir].x;
        this.position.y += DELTAS[bestDir].y;
        this.heading = bestDir;
        this.visited.add(`${this.position.x},${this.position.y}`);
        return { done: false };
      }

      // Check if all reachable cells visited
      const unreached = this._findNearestUnvisited();
      if (!unreached) {
        console.log("[BFSExplorer] All reachable cells visited. Exploration complete.");
        return { done: true };
      }

      // Stuck - no path to goal and no unvisited cells reachable
      console.log("[BFSExplorer] Stuck at (" + this.position.x + "," + this.position.y + ")");
      return { done: true };
    }

    /**
     * Reset
     */
    reset() {
      this.knownWalls = this._createWallGrid();
      this.distances = [];
      this.position = { x: 0, y: 0 };
      this.heading = "east";
      this.atGoal = false;
      this.visited = new Set();
      this.visited.add('0,0');
    }
  }

  global.BFSExplorer = BFSExplorer;
})(window);
