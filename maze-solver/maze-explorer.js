(function (global) {
  const DIRS = global.MAZE_DIRS;
  const DELTAS = global.MAZE_DELTAS;

  /**
   * MazeExplorer: Explores the maze using flood-fill to reach the goal
   * This is the standard micromouse exploration approach
   */
  class MazeExplorer {
    constructor(maze) {
      this.maze = maze;
      this.size = maze.size;
      this.knownWalls = this._createWallGrid();
      this.distances = [];
      this.position = { x: 0, y: 0 };
      this.heading = "east";
      this.atGoal = false;
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

      // Flood fill
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
     * Step: move to the neighbor with lowest distance
     */
    step() {
      // Sense walls first
      this._senseWalls();

      // Recompute distances with updated walls
      this._computeDistances();

      // Check if at goal
      if (this.maze.goalCells.has(`${this.position.x},${this.position.y}`)) {
        this.atGoal = true;
        return { done: true };
      }

      // Find best neighbor
      let bestDir = null;
      let bestDist = this.distances[this.position.y][this.position.x];

      for (const dir of DIRS) {
        const dx = DELTAS[dir].x;
        const dy = DELTAS[dir].y;
        const nx = this.position.x + dx;
        const ny = this.position.y + dy;
        if (nx >= 0 && ny >= 0 && nx < this.size && ny < this.size) {
          if (!this.knownWalls[this.position.y][this.position.x][dir]) {
            const dist = this.distances[ny][nx];
            if (dist < bestDist) {
              bestDist = dist;
              bestDir = dir;
            }
          }
        }
      }

      if (bestDir) {
        // Move
        this.position.x += DELTAS[bestDir].x;
        this.position.y += DELTAS[bestDir].y;
        this.heading = bestDir;
        return { done: false };
      } else {
        // Stuck
        return { done: true };
      }
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
    }
  }

  global.MazeExplorer = MazeExplorer;
})(window);