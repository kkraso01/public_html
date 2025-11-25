(function (global) {
  const DIRS = global.MAZE_DIRS;
  const DELTAS = global.MAZE_DELTAS;

  const headingPriority = (current) => {
    const order = {
      north: ["north", "east", "west", "south"],
      east: ["east", "south", "north", "west"],
      south: ["south", "west", "east", "north"],
      west: ["west", "north", "south", "east"]
    };
    return order[current] || DIRS;
  };

  class FloodSolver {
    constructor(maze) {
      this.maze = maze;
      this.size = maze.size;
      this.reset();
    }

    reset() {
      this.position = { x: 0, y: 0 };
      this.heading = "east"; // mimic Webots controller default
      this.knownWalls = this._createWallGrid();
      this._applyBoundaryWalls();
      this.latestDistances = null;
    }

    _createWallGrid() {
      return Array.from({ length: this.size }, () =>
        Array.from({ length: this.size }, () => ({ north: false, east: false, south: false, west: false }))
      );
    }

    _applyBoundaryWalls() {
      for (let i = 0; i < this.size; i += 1) {
        this.knownWalls[0][i].north = true;
        this.knownWalls[this.size - 1][i].south = true;
        this.knownWalls[i][0].west = true;
        this.knownWalls[i][this.size - 1].east = true;
      }
    }

    atGoal() {
      return this.maze.goalCells.has(`${this.position.x},${this.position.y}`);
    }

    senseWalls() {
      const { x, y } = this.position;
      const real = this.maze.cells[y][x];
      for (const dir of DIRS) {
        const hasWall = real[dir];
        this.knownWalls[y][x][dir] = hasWall;
        const nx = x + DELTAS[dir].x;
        const ny = y + DELTAS[dir].y;
        if (nx >= 0 && ny >= 0 && nx < this.size && ny < this.size) {
          const opposite = DIRS[(DIRS.indexOf(dir) + 2) % 4];
          this.knownWalls[ny][nx][opposite] = hasWall;
        }
      }
    }

    computeDistances(goalSet = this.maze.goalCells) {
      const dist = Array.from({ length: this.size }, () => Array(this.size).fill(Infinity));
      const queue = [];
      for (const key of goalSet) {
        const [gx, gy] = key.split(",").map(Number);
        dist[gy][gx] = 0;
        queue.push([gx, gy]);
      }

      while (queue.length) {
        const [x, y] = queue.shift();
        for (const dir of DIRS) {
          if (this.knownWalls[y][x][dir]) continue;
          const nx = x + DELTAS[dir].x;
          const ny = y + DELTAS[dir].y;
          if (nx < 0 || ny < 0 || nx >= this.size || ny >= this.size) continue;
          if (dist[ny][nx] > dist[y][x] + 1) {
            dist[ny][nx] = dist[y][x] + 1;
            queue.push([nx, ny]);
          }
        }
      }
      this.latestDistances = dist;
      return dist;
    }

    _moveForward(dir) {
      this.position = {
        x: this.position.x + DELTAS[dir].x,
        y: this.position.y + DELTAS[dir].y
      };
      this.heading = dir;
    }

    _chooseDirection(distances) {
      const { x, y } = this.position;
      let bestDir = null;
      let bestScore = Infinity;
      const priority = headingPriority(this.heading);

      for (const dir of priority) {
        if (this.knownWalls[y][x][dir]) continue;
        const nx = x + DELTAS[dir].x;
        const ny = y + DELTAS[dir].y;
        if (nx < 0 || ny < 0 || nx >= this.size || ny >= this.size) continue;
        const score = distances[ny][nx];
        if (score < bestScore) {
          bestScore = score;
          bestDir = dir;
        }
      }
      return bestDir;
    }

    step() {
      this.senseWalls();
      const distances = this.computeDistances();
      const dir = this._chooseDirection(distances);
      if (!dir || distances[this.position.y][this.position.x] === Infinity) {
        return { done: true, reason: "no-path" };
      }
      this._moveForward(dir);
      return { done: this.atGoal(), direction: dir };
    }
  }

  global.FloodSolver = FloodSolver;
})(window);
