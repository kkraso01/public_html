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
      this.phase = "to-goal"; // to-goal -> return -> optimize -> race -> done
      this.atGoal = false;
      this.currentGoalSet = this.maze.goalCells;
      this.traversalHistory = [];
      this.primaryPath = new Set();
      
      // Path optimization (Stage 1 & 2)
      this.discreteOptimalPath = null;  // Grid cells: (x,y) sequence from Dijkstra
      this.racingLineSegments = null;   // Compressed macro-segments for execution
      this.optimizationStats = null;    // Comparison: exploration vs. racing
      this.racingPathIndex = 0;         // Current position in racing line execution
      this.pathOptimizer = null;        // 4-dir or 8-dir optimizer (initialized based on allowOblique)

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

      // Wall is === true when it exists
      if (primary.some(d => this.knownWalls[y][x][d] === true)) return false;

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

      return checks.every(([cx, cy, facing]) => this._inBounds(cx, cy) && this.knownWalls[cy][cx][facing] !== true);
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
        
        // CRITICAL: Forward and backward phases ALWAYS use 4-direction only.
        // Oblique (diagonal costs) only apply during RACING phase optimization.
        // This ensures exploration distances are comparable (cost = 1 per step).
        const dirs = DIRS.map(name => ({ name, dx: DELTAS[name].x, dy: DELTAS[name].y, diagonal: false, components: [name] }));

        for (const dir of dirs) {
          const nx = x + dir.dx;
          const ny = y + dir.dy;
          if (!this._inBounds(nx, ny)) continue;

          if (this.knownWalls[y][x][dir.name] === true) continue;

          const cost = 1;  // Always cardinal cost during exploration
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
            // Only check 4-direction during exploration
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

      // Only use 4-direction during exploration
      const dirs = DIRS.map(name => ({ name, dx: DELTAS[name].x, dy: DELTAS[name].y, diagonal: false, components: [name] }));

      while (queue.length) {
        const { x, y } = queue.shift();

        for (const dir of dirs) {
          if (this.knownWalls[y][x][dir.name]) continue;

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
      // EXPLORATION PHASE: Always 4-direction cardinal only (no diagonals)
      const dirs = DIRS.map(name => ({ name, dx: DELTAS[name].x, dy: DELTAS[name].y, diagonal: false, components: [name] }));
      let bestDir = null;
      let bestVal = Infinity;

      for (const dir of dirs) {
        const nx = this.position.x + dir.dx;
        const ny = this.position.y + dir.dy;
        if (!this._inBounds(nx, ny)) continue;
        if (this.knownWalls[this.position.y][this.position.x][dir.name]) continue;
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
      // RETURN PHASE: Always 4-direction cardinal only (no diagonals)
      const dirs = DIRS.map(name => ({
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
        const nx = this.position.x + dir.dx;
        const ny = this.position.y + dir.dy;
        if (!this._inBounds(nx, ny)) continue;
        if (this.knownWalls[this.position.y][this.position.x][dir.name]) continue;

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

      // 🔥 CRITICAL: Recompute flood-fill from scratch each step.
      // This ensures distances are correct when walls block previously assumed corridors.
      // _updateFloodFillAt() only decreases distances; it cannot handle increasing them.
      this._dynamicFloodFill(this.currentGoalSet);

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
        this.phase = "optimize";
        // 🔥 STAGE 1 & 2: Compute optimal path using discovered walls
        this._computeOptimalPath();
        return { done: false, optimized: true };
      }

      if (this.phase === "optimize") {
        // Transition to racing phase
        this.phase = "race";
        this.racingPathIndex = 0;
        return { done: false, raceStarting: true };
      }

      if (this.phase === "race") {
        const result = this._executeRacingLine();
        if (result.done) {
          this.phase = "done";
          return { done: true, raceComplete: true };
        }
        return result;
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

    /**
     * STAGE 1: Compute discrete optimal path using appropriate optimizer.
     * Then STAGE 2: Compress into racing line segments.
     */
    _computeOptimalPath() {
      // Initialize the appropriate optimizer if not already done
      if (!this.pathOptimizer) {
        if (this.allowOblique && typeof global.ObliquePathOptimizer !== 'undefined') {
          this.pathOptimizer = new global.ObliquePathOptimizer(this.maze, true);
        } else {
          // Fallback: use internal 4-direction method
          this.pathOptimizer = { use4Dir: true };
        }
      }

      // Use appropriate path planner
      let gridPath;
      if (this.pathOptimizer.use4Dir) {
        gridPath = this._dijkstraGridPath();
      } else {
        this.pathOptimizer.setWallMap(this.knownWalls);
        gridPath = this.pathOptimizer.computeOptimalPathOblique(0, 0);
      }

      if (!gridPath || gridPath.length === 0) {
        console.warn("No optimal path found");
        return;
      }
      this.discreteOptimalPath = gridPath;
      
      // STAGE 2: Compress to racing line (macro-segments)
      if (this.pathOptimizer.use4Dir) {
        this.racingLineSegments = this._compressToRacingLine(gridPath);
      } else {
        this.racingLineSegments = this.pathOptimizer.compressToRacingLineOblique(gridPath);
      }
      
      // Compute stats: exploration cost vs. optimal cost
      this._computeOptimizationStats(gridPath);
    }

    /**
     * Execute the racing line: follow the optimized discrete path.
     * Only move if path is valid (no walls blocking).
     */
    _executeRacingLine() {
      if (!this.discreteOptimalPath || this.discreteOptimalPath.length === 0) {
        return { done: true };
      }

      if (this.racingPathIndex >= this.discreteOptimalPath.length) {
        return { done: true };
      }

      const target = this.discreteOptimalPath[this.racingPathIndex];
      
      // Validate move doesn't cross walls
      const dx = target.x - this.position.x;
      const dy = target.y - this.position.y;
      
      // Get direction of move
      let moveDir = null;
      if (dx === 1 && dy === 0) moveDir = "east";
      else if (dx === -1 && dy === 0) moveDir = "west";
      else if (dx === 0 && dy === 1) moveDir = "south";
      else if (dx === 0 && dy === -1) moveDir = "north";
      else if (dx === 1 && dy === 1) moveDir = "southEast";
      else if (dx === 1 && dy === -1) moveDir = "northEast";
      else if (dx === -1 && dy === 1) moveDir = "southWest";
      else if (dx === -1 && dy === -1) moveDir = "northWest";
      
      // Check for wall using strict equality
      if (moveDir) {
        const isCardinal = ["east", "west", "north", "south"].includes(moveDir);
        if (isCardinal && this.knownWalls[this.position.y][this.position.x][moveDir] === true) {
          // Wall blocks this move, skip to next or end
          return { done: true, error: "Wall blocks racing path" };
        }
        if (!isCardinal) {
          const diagonalDir = DIAGONAL_DIRS.find(d => d.name === moveDir);
          if (!this._diagonalMoveAllowed(this.position.x, this.position.y, diagonalDir)) {
            return { done: true, error: "Diagonal blocked" };
          }
        }
      }
      
      // Move is valid
      this.position.x = target.x;
      this.position.y = target.y;

      // Update heading toward next cell if available
      if (this.racingPathIndex < this.discreteOptimalPath.length - 1) {
        const nextTarget = this.discreteOptimalPath[this.racingPathIndex + 1];
        const dir = this._getDirection(target, nextTarget);
        if (dir) {
          this.heading = dir.name;
        }
      }

      this.racingPathIndex++;

      return {
        done: false,
        racing: true,
        progress: `${this.racingPathIndex}/${this.discreteOptimalPath.length}`
      };
    }

    /**
     * STAGE 1a: Dijkstra for 4-direction shortest cell-path (0,0) → goal
     * Used when allowOblique is FALSE.
     * Returns array of {x, y} in order.
     */
    _dijkstraGridPath() {
      const distMap = [];
      const prev = [];
      for (let y = 0; y < this.size; y++) {
        distMap[y] = [];
        prev[y] = [];
        for (let x = 0; x < this.size; x++) {
          distMap[y][x] = Infinity;
          prev[y][x] = null;
        }
      }
      distMap[0][0] = 0;

      const pq = [];
      pq.push({ cost: 0, x: 0, y: 0 });

      // Always 4-direction for cardinal pathfinding
      const dirs = DIRS.map(name => ({ name, dx: DELTAS[name].x, dy: DELTAS[name].y, diagonal: false, components: [name] }));

      while (pq.length) {
        pq.sort((a, b) => a.cost - b.cost);
        const { cost, x, y } = pq.shift();

        if (cost > distMap[y][x]) continue;
        if (this.maze.goalCells.has(`${x},${y}`)) {
          return this._reconstructGridPath(prev, x, y);
        }

        for (const dir of dirs) {
          if (this.knownWalls[y][x][dir.name]) continue;

          const nx = x + dir.dx;
          const ny = y + dir.dy;
          if (!this._inBounds(nx, ny)) continue;

          const stepCost = 1;
          const newCost = cost + stepCost;
          if (newCost < distMap[ny][nx]) {
            distMap[ny][nx] = newCost;
            prev[ny][nx] = { x, y };
            pq.push({ cost: newCost, x: nx, y: ny });
          }
        }
      }
      return null;
    }

    _reconstructGridPath(prev, goalX, goalY) {
      const path = [];
      let x = goalX;
      let y = goalY;
      while (x !== null && y !== null) {
        path.unshift({ x, y });
        const p = prev[y][x];
        if (!p) break;
        x = p.x;
        y = p.y;
      }
      return path;
    }

    /**
     * STAGE 2: Compress discrete path into racing line: straights + turns.
     */
    _compressToRacingLine(gridPath) {
      if (!gridPath || gridPath.length < 2) return [];

      const segments = [];
      let i = 0;

      while (i < gridPath.length - 1) {
        const current = gridPath[i];
        const next = gridPath[i + 1];
        const dir = this._getDirection(current, next);

        let length = 1;
        let j = i + 1;

        // Extend straight as far as possible
        while (j < gridPath.length - 1) {
          const p1 = gridPath[j];
          const p2 = gridPath[j + 1];
          const nextDir = this._getDirection(p1, p2);
          if (nextDir && dir && nextDir.name === dir.name) {
            length++;
            j++;
          } else {
            break;
          }
        }

        segments.push({
          type: "straight",
          direction: dir.name,
          length: length,
          distance: dir.diagonal ? length * Math.SQRT2 : length
        });

        // If there's a turn coming, note it
        if (j < gridPath.length - 1) {
          const turnToDir = this._getDirection(gridPath[j], gridPath[j + 1]);
          const turnType = this._getTurnType(dir, turnToDir);
          if (turnType) {
            segments.push({
              type: "turn",
              angle: turnType.angle,
              direction: turnType.direction
            });
          }
        }

        i = j;
      }

      return segments;
    }

    _getDirection(p1, p2) {
      const dx = Math.sign(p2.x - p1.x);
      const dy = Math.sign(p2.y - p1.y);

      if (dx === 0 && dy === -1) return { name: "north", dx: 0, dy: -1, angle: 0, diagonal: false };
      if (dx === 1 && dy === 0) return { name: "east", dx: 1, dy: 0, angle: 90, diagonal: false };
      if (dx === 0 && dy === 1) return { name: "south", dx: 0, dy: 1, angle: 180, diagonal: false };
      if (dx === -1 && dy === 0) return { name: "west", dx: -1, dy: 0, angle: 270, diagonal: false };
      if (dx === 1 && dy === -1) return { name: "northEast", dx: 1, dy: -1, angle: 45, diagonal: true };
      if (dx === 1 && dy === 1) return { name: "southEast", dx: 1, dy: 1, angle: 135, diagonal: true };
      if (dx === -1 && dy === 1) return { name: "southWest", dx: -1, dy: 1, angle: 225, diagonal: true };
      if (dx === -1 && dy === -1) return { name: "northWest", dx: -1, dy: -1, angle: 315, diagonal: true };
      return null;
    }

    _getTurnType(fromDir, toDir) {
      if (!fromDir || !toDir) return null;
      const fromAngle = fromDir.angle;
      const toAngle = toDir.angle;
      let diff = (toAngle - fromAngle + 360) % 360;
      if (diff > 180) diff = diff - 360;

      if (diff === 0) return null; // No turn
      if (Math.abs(diff) === 90 || Math.abs(diff) === 45 || Math.abs(diff) === 135) {
        return {
          angle: Math.abs(diff),
          direction: diff > 0 ? "left" : "right"
        };
      }
      return null;
    }

    /**
     * STAGE 2b: Compute stats comparing exploration vs. optimal.
     */
    _computeOptimizationStats(optimalPath) {
      const explorationCost = this.traversalHistory.length;
      let optimalCost = 0;
      for (let i = 0; i < optimalPath.length - 1; i++) {
        const p1 = optimalPath[i];
        const p2 = optimalPath[i + 1];
        const dx = Math.abs(p2.x - p1.x);
        const dy = Math.abs(p2.y - p1.y);
        optimalCost += (dx === 1 && dy === 1) ? Math.SQRT2 : 1;
      }

      this.optimizationStats = {
        explorationSteps: explorationCost,
        optimalSteps: optimalCost.toFixed(2),
        improvementFactor: (explorationCost / optimalCost).toFixed(2),
        timeSavedPercent: ((1 - optimalCost / explorationCost) * 100).toFixed(1)
      };
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
      this.discreteOptimalPath = null;
      this.racingLineSegments = null;
      this.optimizationStats = null;
      this.racingPathIndex = 0;
      this.pathOptimizer = null;
      this._dynamicFloodFill();
      this._computeDeadZones();
    }
  }

  global.MazeExplorer = MazeExplorer;
})(window);
