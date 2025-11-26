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
   * MazeExplorer: Chinese-style exploration with dynamic flood fill,
   * dead-end & dead-zone pruning, and optional oblique sprinting.
   */
  class MazeExplorer {
    constructor(maze, options = {}) {
      this.maze = maze;
      this.size = maze.size;
      this.allowOblique = Boolean(options.allowOblique);

      this.knownWalls = this._createWallGrid();
      this.distances = [];
      this.visited = this._createBooleanGrid(false);
      this.deadEnds = this._createBooleanGrid(false);
      this.deadZones = this._createBooleanGrid(false);
      this.position = { x: 0, y: 0 };
      this.heading = "east";
      this.atGoal = false;

      this._dynamicFloodFill();
      this._computeDeadZones();
    }

    _createWallGrid() {
      const walls = [];
      for (let y = 0; y < this.size; y++) {
        walls[y] = [];
        for (let x = 0; x < this.size; x++) {
          walls[y][x] = {};
          if (x === 0) walls[y][x].west = true;
          if (y === 0) walls[y][x].north = true;
          if (x === this.size - 1) walls[y][x].east = true;
          if (y === this.size - 1) walls[y][x].south = true;
        }
      }
      return walls;
    }

    _createBooleanGrid(value) {
      return Array.from({ length: this.size }, () => Array(this.size).fill(value));
    }

    _inBounds(x, y) {
      return x >= 0 && y >= 0 && x < this.size && y < this.size;
    }

    _diagonalMoveAllowed(x, y, dir) {
      const primary = dir.components;
      const nx = x + dir.dx;
      const ny = y + dir.dy;
      if (!this._inBounds(nx, ny)) return false;

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

      return checks.every(([cx, cy, facing]) => this._inBounds(cx, cy) && !this.knownWalls[cy][cx][facing]);
    }

    _dynamicFloodFill() {
      this.distances = this._createBooleanGrid(Infinity);
      const queue = [];

      this.maze.goalCells.forEach(key => {
        const [x, y] = key.split(',').map(Number);
        this.distances[y][x] = 0;
        queue.push({ x, y, cost: 0 });
      });

      while (queue.length) {
        queue.sort((a, b) => a.cost - b.cost);
        const { x, y } = queue.shift();
        const dirs = this.allowOblique
          ? DIAGONAL_DIRS
          : DIRS.map(name => ({ name, dx: DELTAS[name].x, dy: DELTAS[name].y, diagonal: false, components: [name] }));

        for (const dir of dirs) {
          if (dir.diagonal && !this._diagonalMoveAllowed(x, y, dir)) continue;

          const nx = x + dir.dx;
          const ny = y + dir.dy;
          if (!this._inBounds(nx, ny)) continue;

          if (!dir.diagonal && this.knownWalls[y][x][dir.name]) continue;

          const cost = dir.diagonal ? Math.SQRT2 : 1;
          const newDist = this.distances[y][x] + cost;
          if (newDist < this.distances[ny][nx]) {
            this.distances[ny][nx] = newDist;
            queue.push({ x: nx, y: ny, cost: newDist });
          }
        }
      }
    }

    _updateFloodFillAt(cell) {
      const queue = [cell];
      while (queue.length) {
        const { x, y } = queue.shift();
        let best = Infinity;

        for (const dir of DIRS) {
          if (this.knownWalls[y][x][dir]) continue;
          const nx = x + DELTAS[dir].x;
          const ny = y + DELTAS[dir].y;
          if (!this._inBounds(nx, ny)) continue;
          best = Math.min(best, this.distances[ny][nx] + 1);
        }

        if (best < this.distances[y][x]) {
          this.distances[y][x] = best;
          for (const dir of DIRS) {
            if (this.knownWalls[y][x][dir]) continue;
            const nx = x + DELTAS[dir].x;
            const ny = y + DELTAS[dir].y;
            if (this._inBounds(nx, ny)) queue.push({ x: nx, y: ny });
          }
        }
      }
    }

    _senseWalls() {
      const { x, y } = this.position;
      let updated = false;
      for (const dir of DIRS) {
        const nx = x + DELTAS[dir].x;
        const ny = y + DELTAS[dir].y;
        if (!this._inBounds(nx, ny)) continue;

        const hasWall = this.maze.cells[y][x][dir];
        if (this.knownWalls[y][x][dir] !== hasWall) {
          this.knownWalls[y][x][dir] = hasWall;
          const opposite = DIRS[(DIRS.indexOf(dir) + 2) % 4];
          this.knownWalls[ny][nx][opposite] = hasWall;
          updated = true;
          this._updateFloodFillAt({ x, y });
          this._updateFloodFillAt({ x: nx, y: ny });
        }
      }
      return updated;
    }

    _updateDeadEnds() {
      let changed = false;
      do {
        changed = false;
        for (let y = 0; y < this.size; y++) {
          for (let x = 0; x < this.size; x++) {
            if (this.maze.goalCells.has(`${x},${y}`) || this.deadEnds[y][x]) continue;

            let openCount = 0;
            for (const dir of DIRS) {
              if (!this.knownWalls[y][x][dir]) openCount += 1;
            }

            if (openCount === 1) {
              this.deadEnds[y][x] = true;
              changed = true;
            }
          }
        }
      } while (changed);
    }

    _hasMonotonicPathToGoal(x, y) {
      let cx = x;
      let cy = y;
      const guard = this.size * this.size * 2;
      let steps = 0;

      while (this.distances[cy][cx] !== 0 && steps < guard) {
        const currentDist = this.distances[cy][cx];
        const neighbors = [];
        for (const dir of DIRS) {
          if (this.knownWalls[cy][cx][dir]) continue;
          const nx = cx + DELTAS[dir].x;
          const ny = cy + DELTAS[dir].y;
          if (!this._inBounds(nx, ny)) continue;
          neighbors.push({ nx, ny, dist: this.distances[ny][nx] });
        }

        neighbors.sort((a, b) => a.dist - b.dist);
        const lower = neighbors.find(n => n.dist < currentDist);
        if (!lower) return false;

        cx = lower.nx;
        cy = lower.ny;
        steps += 1;
      }

      return this.distances[cy][cx] === 0;
    }

    _computeDeadZones() {
      for (let y = 0; y < this.size; y++) {
        for (let x = 0; x < this.size; x++) {
          if (this.deadEnds[y][x]) {
            this.deadZones[y][x] = false;
            continue;
          }
          const hasPath = this._hasMonotonicPathToGoal(x, y);
          this.deadZones[y][x] = !hasPath;
        }
      }
    }

    _chooseNextMove() {
      const dirs = this.allowOblique
        ? DIAGONAL_DIRS
        : DIRS.map(name => ({ name, dx: DELTAS[name].x, dy: DELTAS[name].y, diagonal: false, components: [name] }));
      let bestDir = null;
      let bestVal = Infinity;

      for (const dir of dirs) {
        if (dir.diagonal && !this._diagonalMoveAllowed(this.position.x, this.position.y, dir)) continue;

        const nx = this.position.x + dir.dx;
        const ny = this.position.y + dir.dy;
        if (!this._inBounds(nx, ny)) continue;
        if (!dir.diagonal && this.knownWalls[this.position.y][this.position.x][dir.name]) continue;
        if (this.deadEnds[ny][nx] || this.deadZones[ny][nx]) continue;

        const candidate = this.distances[ny][nx];
        if (candidate < bestVal) {
          bestVal = candidate;
          bestDir = dir;
        } else if (candidate === bestVal && bestDir && dir.name === this.heading) {
          bestDir = dir;
        }
      }

      return bestDir;
    }

    step() {
      this.visited[this.position.y][this.position.x] = true;
      this._senseWalls();

      this._updateDeadEnds();
      this._dynamicFloodFill();
      this._computeDeadZones();

      if (this.maze.goalCells.has(`${this.position.x},${this.position.y}`)) {
        this.atGoal = true;
        return { done: true };
      }

      const nextDir = this._chooseNextMove();
      if (!nextDir) {
        return { done: true };
      }

      this.position.x += nextDir.dx;
      this.position.y += nextDir.dy;
      this.heading = nextDir.name;
      return { done: false };
    }

    reset() {
      this.knownWalls = this._createWallGrid();
      this.distances = [];
      this.visited = this._createBooleanGrid(false);
      this.deadEnds = this._createBooleanGrid(false);
      this.deadZones = this._createBooleanGrid(false);
      this.position = { x: 0, y: 0 };
      this.heading = "east";
      this.atGoal = false;
      this._dynamicFloodFill();
      this._computeDeadZones();
    }
  }

  global.MazeExplorer = MazeExplorer;
})(window);
