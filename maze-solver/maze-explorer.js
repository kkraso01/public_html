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
      this.phase = "to-goal"; // to-goal -> return -> done
      this.atGoal = false;
      this.currentGoalSet = this.maze.goalCells;
      this.traversalHistory = [];
      this.primaryPath = new Set();

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

      if (primary.some(d => this.knownWalls[y][x][d] !== false)) return false;

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

    _dynamicFloodFill(goalSet = this.currentGoalSet || this.maze.goalCells) {
      this.distances = Array.from({ length: this.size }, () => Array(this.size).fill(Infinity));
      const queue = [];

      goalSet.forEach(key => {
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
            if (this.maze.goalCells.has(`${x},${y}`) || (x === 0 && y === 0) || this.deadEnds[y][x]) continue;

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

    _computeDeadZones() {
      const reachable = new Set();
      const queue = [];

      // Start from goal cells
      for (const key of this.currentGoalSet) {
        const [gx, gy] = key.split(',').map(Number);
        reachable.add(key);
        queue.push({ x: gx, y: gy });
      }

      const dirs = this.allowOblique
        ? DIAGONAL_DIRS
        : DIRS.map(name => ({ name, dx: DELTAS[name].x, dy: DELTAS[name].y, diagonal: false, components: [name] }));

      while (queue.length) {
        const { x, y } = queue.shift();

        for (const dir of dirs) {
          if (dir.diagonal && !this._diagonalMoveAllowed(x, y, dir)) continue;
          if (!dir.diagonal && this.knownWalls[y][x][dir.name]) continue;

          const nx = x + dir.dx;
          const ny = y + dir.dy;
          if (!this._inBounds(nx, ny)) continue;

          const k = `${nx},${ny}`;
          if (!reachable.has(k)) {
            reachable.add(k);
            queue.push({ x: nx, y: ny });
          }
        }
      }

      // Mark dead zones = not reachable from goals
      for (let y = 0; y < this.size; y++) {
        for (let x = 0; x < this.size; x++) {
          const k = `${x},${y}`;
          this.deadZones[y][x] = !this.deadEnds[y][x] && !reachable.has(k);
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

    _chooseReturnMove() {
      const dirs = this.allowOblique
        ? DIAGONAL_DIRS
        : DIRS.map(name => ({
            name,
            dx: DELTAS[name].x,
            dy: DELTAS[name].y,
            diagonal: false,
            components: [name]
          }));

      const currDist = this.distances[this.position.y][this.position.x];
      const currManhattan = Math.abs(this.position.x) + Math.abs(this.position.y);

      let candidates = [];

      for (const dir of dirs) {
        if (dir.diagonal && !this._diagonalMoveAllowed(this.position.x, this.position.y, dir)) continue;

        const nx = this.position.x + dir.dx;
        const ny = this.position.y + dir.dy;
        if (!this._inBounds(nx, ny)) continue;
        if (!dir.diagonal && this.knownWalls[this.position.y][this.position.x][dir.name]) continue;

        const key = `${nx},${ny}`;
        const onPrimary = this.primaryPath.has(key);
        const isVisited = this.visited[ny][nx];
        const isDeadEnd = this.deadEnds[ny][nx];
        const nd = this.distances[ny][nx];
        const manhattan = Math.abs(nx) + Math.abs(ny);

        candidates.push({
          dir,
          onPrimary,
          isVisited,
          isDeadEnd,
          nd,
          manhattan,
          decreasesFlood: nd < currDist,
          decreasesManhattan: manhattan < currManhattan
        });
      }

      if (!candidates.length) return null;

      // 1) Never go deeper into a dead-end unless it still descends flood distance.
      candidates = candidates.filter(c => !c.isDeadEnd || c.decreasesFlood);
      if (!candidates.length) return null;

      // 2) Prefer strict flood-distance descent. Within those, pick unvisited, then primary, then smallest dist.
      const descending = candidates.filter(c => c.decreasesFlood);
      if (descending.length) {
        descending.sort((a, b) => {
          if (a.isVisited !== b.isVisited) return a.isVisited ? 1 : -1;
          if (a.onPrimary !== b.onPrimary) return a.onPrimary ? -1 : 1;
          return a.nd - b.nd;
        });
        return descending[0].dir;
      }

      // 3) Safe exploration: unvisited neighbors that reduce Manhattan distance to start.
      const explore = candidates.filter(c => !c.isVisited && c.decreasesManhattan);
      if (explore.length) {
        explore.sort((a, b) => a.manhattan - b.manhattan);
        return explore[0].dir;
      }

      // 4) Follow the forward primary path back toward start when available.
      const primaryMoves = candidates.filter(c => c.onPrimary);
      if (primaryMoves.length) {
        primaryMoves.sort((a, b) => a.nd - b.nd);
        return primaryMoves[0].dir;
      }

      // 5) Final fallback: any neighbor closest in Manhattan distance.
      candidates.sort((a, b) => a.manhattan - b.manhattan);
      return candidates[0].dir;
    }

    step() {
      this.visited[this.position.y][this.position.x] = true;
      if (this.phase === "to-goal") {
        this.primaryPath.add(`${this.position.x},${this.position.y}`);
      }
      this.traversalHistory.push(`${this.position.x},${this.position.y}`);
      this._senseWalls();

      this._updateDeadEnds();

      if (this.phase === "to-goal") {
        this._computeDeadZones();
      }

      if (this.phase === "to-goal" && this.maze.goalCells.has(`${this.position.x},${this.position.y}`)) {
        this.atGoal = true;
        this.phase = "return";
        this.primaryPath = new Set(this.traversalHistory);
        this.currentGoalSet = new Set(["0,0"]);
        this._dynamicFloodFill();
        // Don't recompute dead zones during return to preserve forward corridor classification
        return { done: false, reachedGoal: true };
      }

      if (this.phase === "return" && this.position.x === 0 && this.position.y === 0) {
        this.phase = "done";
        return { done: true, returned: true };
      }

      const nextDir = this.phase === "return" ? this._chooseReturnMove() : this._chooseNextMove();
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
      this.phase = "to-goal";
      this.atGoal = false;
      this.currentGoalSet = this.maze.goalCells;
      this.traversalHistory = [];
      this.primaryPath = new Set();
      this._dynamicFloodFill();
      this._computeDeadZones();
    }
  }

  global.MazeExplorer = MazeExplorer;
})(window);
