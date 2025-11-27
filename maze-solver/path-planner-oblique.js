(function (global) {
  /**
   * Eight grid directions for connectivity.
   * dx, dy are in cell units. angle is in radians.
   */
  const DIR8 = [
    { name: "N",  dx:  0, dy: -1, diagonal: false, angle: -Math.PI / 2 },
    { name: "NE", dx:  1, dy: -1, diagonal: true,  angle: -Math.PI / 4 },
    { name: "E",  dx:  1, dy:  0, diagonal: false, angle: 0 },
    { name: "SE", dx:  1, dy:  1, diagonal: true,  angle:  Math.PI / 4 },
    { name: "S",  dx:  0, dy:  1, diagonal: false, angle:  Math.PI / 2 },
    { name: "SW", dx: -1, dy:  1, diagonal: true,  angle:  3 * Math.PI / 4 },
    { name: "W",  dx: -1, dy:  0, diagonal: false, angle:  Math.PI },
    { name: "NW", dx: -1, dy: -1, diagonal: true,  angle: -3 * Math.PI / 4 }
  ];

  const EPS = 1e-6;

  /**
   * RacePlanner
   * -----------
   * Full race plan computation for post-exploration sprint phase.
   * 
   * Pipeline:
   *  1. 8D shortest grid path (Dijkstra with Asian diagonal legality)
   *  2. Compress to line segments (straight runs)
   *  3. Insert smooth circular arcs at corners
   *  4. Time-optimal velocity planning (forward-backward pass)
   *
   * Output: discrete path + geometry + per-segment velocity profiles
   */
  class RacePlanner {
    /**
     * @param {Object} maze - { size, goalCells: Set("x,y") }
     * @param {Array<Array<Object>>} knownWalls - [y][x] = {north,east,south,west}
     * @param {Object} options
     *   - cellSize       : meters per cell (e.g. 0.018)
     *   - vMax           : max straight-line speed (m/s)
     *   - aMax           : max linear acceleration (m/s^2)
     *   - dMax           : max linear deceleration (m/s^2)
     *   - aLatMax        : max lateral (centripetal) acceleration (m/s^2)
     *   - cornerRadius   : default corner arc radius (meters)
     *   - bodyLength     : robot length (meters, e.g. 0.095 = 95mm)
     *   - bodyWidth      : robot width (meters, e.g. 0.075 = 75mm)
     *   - wheelbase      : distance front-to-rear wheel (meters, e.g. 0.055 = 55mm)
     *   - trackWidth     : distance left-to-right wheel (meters, e.g. 0.070 = 70mm)
     */
    constructor(maze, knownWalls, options = {}) {
      this.maze = maze;
      this.size = maze.size;
      
      // CRITICAL FIX: Sanitize knownWalls to make all undefined edges explicitly OPEN
      // This allows diagonals to work correctly during race planning
      this.knownWalls = this._sanitizeWallMap(knownWalls);

      // ===== ROBOT PARAMETERS (typical competitive micromouse) =====
      // Real winners: 90-110mm length, 70-85mm width, wheelbase 50-60mm, track 65-75mm
      this.bodyLength      = options.bodyLength   || 0.095;   // 95mm typical
      this.bodyWidth       = options.bodyWidth    || 0.075;   // 75mm typical
      this.wheelbase       = options.wheelbase    || 0.055;   // 55mm front-to-rear
      this.trackWidth      = options.trackWidth   || 0.070;   // 70mm left-to-right

      // ===== MOTION PARAMETERS =====
      this.cellSize     = options.cellSize     || 0.018;
      this.vMax         = options.vMax         || 3.0;   // 3 m/s typical sprint
      this.aMax         = options.aMax         || 5.0;   // 5 m/s^2
      this.dMax         = options.dMax         || 6.0;   // braking
      this.aLatMax      = options.aLatMax      || 7.0;   // lateral accel
      
      // CORNER RADIUS: Set to realistic micromouse turning radius
      // Real competition robots use 30-70mm radius for smooth, visible curves
      // - 40mm (0.04m) is optimal for 18mm cells + reasonable max speed
      // - Much larger than 3.6mm to ensure curves are visible on canvas
      const geometricLimit = this.cellSize / 4;  // 4.5mm: theoretical minimum
      const physicsLimit = (this.vMax * this.vMax) / this.aLatMax;  // v² / a_lat
      
      if (options.cornerRadius !== undefined) {
        this.cornerRadius = options.cornerRadius;
      } else {
        // DEFAULT: 40mm radius (competitive micromouse standard, visible on screen)
        this.cornerRadius = 0.04;
      }
      console.log(
        `[RacePlanner] Robot: ${(this.bodyLength*1000).toFixed(0)}×${(this.bodyWidth*1000).toFixed(0)}mm ` +
        `wheelbase=${(this.wheelbase*1000).toFixed(0)}mm track=${(this.trackWidth*1000).toFixed(0)}mm | ` +
        `Motion: vMax=${this.vMax}m/s aLat=${this.aLatMax}m/s² | ` +
        `Turn radius: ${(this.cornerRadius * 1000).toFixed(1)}mm (competitive standard)`
      );

      this._enforceCardinalSymmetry();
    }

    // ---------- wall sanitization ----------

    /**
     * Create a safe copy of wall map where undefined edges = WALLS (unknown = closed).
     * Micromouse rule: Unknown edges must be assumed to be walls until confirmed open.
     * This prevents the planner from cutting through unexplored areas.
     */
    _sanitizeWallMap(knownWalls) {
      const n = this.size;
      const sanitized = [];
      
      for (let y = 0; y < n; y++) {
        sanitized[y] = [];
        for (let x = 0; x < n; x++) {
          const cell = knownWalls[y]?.[x] || {};
          
          // Explicitly set all walls: undefined/missing = true (WALL/UNKNOWN)
          // Only explicitly false (discovered open) = false
          sanitized[y][x] = {
            north: cell.north !== false,    // Default to wall unless confirmed open
            south: cell.south !== false,
            east:  cell.east !== false,
            west:  cell.west !== false
          };
        }
      }
      
      console.log("[RacePlanner] Wall map sanitized: undefined edges now treated as WALLS (unknown=closed)");
      return sanitized;
    }

    // ---------- wall helpers ----------

    _inBounds(x, y) {
      return x >= 0 && y >= 0 && x < this.size && y < this.size;
    }

    // Treat only explicit true as wall (undefined is now false after sanitization)
    _isWall(walls, dir) {
      if (!walls) return false; // Safety: if cell doesn't exist, treat as open (not wall)
      return walls[dir] === true;
    }

    _enforceCardinalSymmetry() {
      const n = this.size;
      for (let y = 0; y < n; y++) {
        for (let x = 0; x < n; x++) {
          const cell = this.knownWalls[y][x] || (this.knownWalls[y][x] = {});
          
          // east/west symmetry
          if (cell.east === true && x + 1 < n) {
            this.knownWalls[y][x + 1] ||= {};
            this.knownWalls[y][x + 1].west = true;
          }
          if (cell.west === true && x - 1 >= 0) {
            this.knownWalls[y][x - 1] ||= {};
            this.knownWalls[y][x - 1].east = true;
          }
          
          // north/south symmetry
          if (cell.south === true && y + 1 < n) {
            this.knownWalls[y + 1][x] ||= {};
            this.knownWalls[y + 1][x].north = true;
          }
          if (cell.north === true && y - 1 >= 0) {
            this.knownWalls[y - 1][x] ||= {};
            this.knownWalls[y - 1][x].south = true;
          }
        }
      }
    }

    /**
     * Log the complete maze structure with walls to console
     * Uses ASCII art to visualize walls and open passages
     */
    logMazeStructure() {
      const n = this.size;
      const walls = this.knownWalls;
      
      // Build ASCII maze
      let mazeStr = "\n[RacePlanner] MAZE STRUCTURE:\n\n";
      
      for (let y = 0; y < n; y++) {
        // Top wall line
        let topLine = "  ";
        for (let x = 0; x < n; x++) {
          topLine += walls[y][x].north ? "┌─" : "┌ ";
        }
        topLine += "┐";
        mazeStr += topLine + "\n";
        
        // Cell line with side walls
        let cellLine = "  ";
        for (let x = 0; x < n; x++) {
          cellLine += walls[y][x].west ? "│" : " ";
          cellLine += " ";
        }
        cellLine += walls[y][n-1].east ? "│" : " ";
        mazeStr += cellLine + "\n";
      }
      
      // Bottom wall line
      let bottomLine = "  ";
      for (let x = 0; x < n; x++) {
        bottomLine += walls[n-1][x].south ? "└─" : "└ ";
      }
      bottomLine += "┘";
      mazeStr += bottomLine + "\n";
      
      // Build detailed wall map
      const wallDetails = {};
      for (let y = 0; y < n; y++) {
        for (let x = 0; x < n; x++) {
          const key = `(${x},${y})`;
          const cell = walls[y][x];
          const wallList = [];
          if (cell.north) wallList.push("N");
          if (cell.east) wallList.push("E");
          if (cell.south) wallList.push("S");
          if (cell.west) wallList.push("W");
          
          if (wallList.length > 0) {
            wallDetails[key] = wallList.join(" ");
          }
        }
      }
      
      mazeStr += "\nCell Walls (N/E/S/W):\n";
      Object.entries(wallDetails).forEach(([cell, walls]) => {
        mazeStr += `  ${cell}: ${walls}\n`;
      });
      
      console.log(mazeStr);
      console.log("[RacePlanner] Maze size:", n, "×", n);
      console.log("[RacePlanner] Goal cells:", Array.from(this.maze.goalCells));
    }

    /**
     * FULL Japanese diagonal legality with multi-cell extension
     * Uses grid ray stepping to check diagonal corridors of any length.
     * Supports arbitrarily long diagonals (5, 10, 20+ cells).
     */
    _canMoveDiagonalLong(x0, y0, dx, dy) {
      // dx, dy ∈ { -1, +1 }
      const x1 = x0 + dx;
      const y1 = y0 + dy;

      // 1-step must be legal for multi-step to start
      if (!this._canMoveDiagonalOne(x0, y0, dx, dy)) {
        return false;
      }

      let x = x1;
      let y = y1;

      // Walk the diagonal ray until blocked
      while (true) {
        const nx = x + dx;
        const ny = y + dy;

        if (!this._inBounds(nx, ny)) break;

        if (!this._canMoveDiagonalOne(x, y, dx, dy)) {
          break;
        }

        x = nx;
        y = ny;
      }

      return { endX: x, endY: y };
    }

    /**
     * Japanese 1-step diagonal legality check
     * Applies all micromouse competition rules:
     *  - Primary cardinals open from source
     *  - Opposite cardinals open into target
     *  - L-block test (both corner walls block movement)
     */
    _canMoveDiagonalOne(x, y, dx, dy) {
      const nx = x + dx;
      const ny = y + dy;
      if (!this._inBounds(nx, ny)) return false;

      const cell   = this.knownWalls[y][x];
      const target = this.knownWalls[ny][nx];
      if (!cell || !target) return false;

      // Cardinal directions
      const dirX = dx > 0 ? "east" : "west";
      const dirY = dy > 0 ? "south" : "north";
      const oppX = dx > 0 ? "west" : "east";
      const oppY = dy > 0 ? "north" : "south";

      // 1. Primary cardinals must be open
      if (cell[dirX]) return false;
      if (cell[dirY]) return false;

      // 2. Opposite cardinals must be open into target
      if (target[oppX]) return false;
      if (target[oppY]) return false;

      // 3. L-block test (both corner walls block movement)
      const cx1 = x + dx;
      const cy1 = y;
      const cx2 = x;
      const cy2 = y + dy;

      if (!this._inBounds(cx1, cy1) || !this._inBounds(cx2, cy2)) {
        return false;
      }

      const corner1 = this.knownWalls[cy1][cx1];
      const corner2 = this.knownWalls[cy2][cx2];

      const edge1 = dy > 0 ? "south" : "north";
      const edge2 = dx > 0 ? "east"  : "west";

      // Both corners must NOT block (true L-block blocks)
      if (corner1[edge1] && corner2[edge2]) {
        return false;
      }

      return true;
    }

    /**
     * For Dijkstra: adapter to work with DIR8 format
     * Single-step diagonal check for path finding
     */
    _canMoveDiagonal(x, y, dir) {
      return this._canMoveDiagonalOne(x, y, dir.dx, dir.dy);
    }

    _canMoveCardinal(x, y, dir) {
      const nx = x + dir.dx;
      const ny = y + dir.dy;
      if (!this._inBounds(nx, ny)) return false;

      const cell   = this.knownWalls[y][x];
      const target = this.knownWalls[ny][nx];
      if (!cell || !target) return false;

      const dirName =
        dir.dx === 1 ? "east" :
        dir.dx === -1 ? "west" :
        dir.dy === 1 ? "south" : "north";

      const oppName =
        dir.dx === 1 ? "west" :
        dir.dx === -1 ? "east" :
        dir.dy === 1 ? "north" : "south";

      if (cell[dirName]) return false;
      if (target[oppName]) return false;

      return true;
    }

    // ---------- Stage 1: 8D shortest path on grid ----------

    /**
     * Dijkstra on cells only (no heading), 8-connected with Asian diagonals.
     * Returns array of {x,y} from start to some goal cell.
     */
    computeGridPath(startX = 0, startY = 0) {
      const key = (x, y) => `${x},${y}`;
      const dist = new Map();
      const prev = new Map();
      const heap = new MinHeap();

      const startKey = key(startX, startY);
      dist.set(startKey, 0);
      heap.push([0, startX, startY]);

      const goals = this.maze.goalCells;

      while (!heap.isEmpty()) {
        const [cost, x, y] = heap.pop();
        const k = key(x, y);
        const best = dist.get(k);
        if (best === undefined || cost > best + EPS) continue;

        // Goal check
        if (goals.has(k)) {
          return this._reconstructCellPath(prev, x, y);
        }

        // Explore neighbors
        for (const dir of DIR8) {
          const nx = x + dir.dx;
          const ny = y + dir.dy;
          if (!this._inBounds(nx, ny)) continue;

          let ok = false;
          if (dir.diagonal) ok = this._canMoveDiagonal(x, y, dir);
          else              ok = this._canMoveCardinal(x, y, dir);
          if (!ok) continue;

          // CRITICAL: Diagonal moves cost √2 ≈ 1.414 cells (Euclidean distance)
          // Cardinal moves cost exactly 1.0 cell
          // This ensures the path planner prefers efficient diagonal shortcuts
          // while maintaining realistic physical costs for time-optimal racing
          const stepLengthCells = dir.diagonal ? Math.SQRT2 : 1.0;
          const stepCost = stepLengthCells;

          const nk = key(nx, ny);
          const newCost = cost + stepCost;
          const oldCost = dist.has(nk) ? dist.get(nk) : Infinity;

          if (newCost + EPS < oldCost) {
            dist.set(nk, newCost);
            prev.set(nk, k);
            heap.push([newCost, nx, ny]);
          }
        }
      }

      // Log the distance matrix for debugging
      console.log("[RacePlanner] Dijkstra distance matrix:", {
        size: this.maze.size,
        distMapSize: dist.size,
        matrix: this._buildMatrixFromDist(dist)
      });

      return null;
    }

    _buildMatrixFromDist(dist) {
      // Convert distance map to 2D matrix for visualization
      const size = this.maze.size;
      const matrix = Array(size).fill(null).map(() => Array(size).fill(Infinity));
      
      for (const [key, value] of dist.entries()) {
        const [xStr, yStr] = key.split(",");
        const x = Number(xStr);
        const y = Number(yStr);
        if (x >= 0 && x < size && y >= 0 && y < size) {
          matrix[y][x] = Number(value.toFixed(3));
        }
      }
      
      return matrix;
    }

    _reconstructCellPath(prev, gx, gy) {
      const key = (x, y) => `${x},${y}`;
      const path = [];
      let k = key(gx, gy);

      while (prev.has(k)) {
        const [xStr, yStr] = k.split(",");
        path.push({ x: Number(xStr), y: Number(yStr) });
        k = prev.get(k);
      }
      const [sx, sy] = k.split(",");
      path.push({ x: Number(sx), y: Number(sy) });
      path.reverse();
      return path;
    }

    // ---------- Stage 1b: path smoothing ----------

    /**
     * Remove back-forward reversals (zigzags).
     * Pattern: A -> B -> A is removed.
     */
    _removeUselessReversals(path) {
      return path.filter((cell, i) => {
        if (i === 0 || i === path.length - 1) return true;
        const prev = path[i - 1];
        const next = path[i + 1];
        return !(prev.x === next.x && prev.y === next.y);
      });
    }

    /**
     * Remove collinear middle points.
     * If A -> B -> C are in a line, remove B.
     */
    _removeCollinear(path) {
      if (path.length < 3) return path;
      const result = [path[0]];

      for (let i = 1; i < path.length - 1; i++) {
        const a = path[i - 1];
        const b = path[i];
        const c = path[i + 1];

        const dx1 = b.x - a.x;
        const dy1 = b.y - a.y;
        const dx2 = c.x - b.x;
        const dy2 = c.y - b.y;

        // Collinear if cross product is 0 and same direction
        const collinear = (dx1 * dy2 === dy1 * dx2) &&
                         (Math.sign(dx1) === Math.sign(dx2) || dx1 === 0 || dx2 === 0) &&
                         (Math.sign(dy1) === Math.sign(dy2) || dy1 === 0 || dy2 === 0);

        if (!collinear) {
          result.push(b);
        }
      }

      result.push(path[path.length - 1]);
      return result;
    }

    /**
     * Replace L-shaped corners with diagonal shortcuts.
     * Pattern: A -> B (cardinal) -> C (perpendicular cardinal)
     * Replace with A -> C if diagonal is legal.
     */
    _cornerShortcuts(path) {
      if (path.length < 3) return path;
      const result = [path[0]];

      for (let i = 1; i < path.length - 1; i++) {
        const a = path[i - 1];
        const b = path[i];
        const c = path[i + 1];

        const dx1 = b.x - a.x;
        const dy1 = b.y - a.y;
        const dx2 = c.x - b.x;
        const dy2 = c.y - b.y;

        // Check if perpendicular cardinals forming L-shape
        const isL = (Math.abs(dx1) === 1 && dy1 === 0) && (dx2 === 0 && Math.abs(dy2) === 1);
        const isLrev = (dx1 === 0 && Math.abs(dy1) === 1) && (Math.abs(dx2) === 1 && dy2 === 0);

        if (isL || isLrev) {
          // Try diagonal shortcut from a to c
          const ddx = c.x - a.x;
          const ddy = c.y - a.y;

          if (Math.abs(ddx) === 1 && Math.abs(ddy) === 1) {
            // Check if diagonal is legal
            if (this._canMoveDiagonalOne(a.x, a.y, ddx, ddy)) {
              // Skip b, go direct a -> c
              result.push(c);
              i++; // Skip the intermediate cardinal step
              continue;
            }
          }
        }

        result.push(b);
      }

      result.push(path[path.length - 1]);
      return result;
    }

    /**
     * Extend diagonals across entire corridor.
     * CRITICAL FIX: Try ALL 4 diagonal directions at EVERY cell,
     * even if the raw path is rectilinear. Detects diagonal corridors
     * that aren't already diagonal in the input path.
     */
    _extendDiagonals(path) {
      if (path.length < 2) return path;

      const out = [path[0]];
      let i = 1;

      while (i < path.length) {
        const prev = out[out.length - 1];
        const curr = path[i];

        // Try all 4 diagonal directions from prev
        // This is the KEY FIX: check diagonals even if path isn't diagonal
        const diagDirs = [
          [1, 1],   // SE
          [1, -1],  // NE
          [-1, 1],  // SW
          [-1, -1]  // NW
        ];

        let extended = false;

        for (const [dx, dy] of diagDirs) {
          // 1-step diagonal must be legal first
          const oneStep = this._canMoveDiagonalOne(prev.x, prev.y, dx, dy);
          if (!oneStep) continue;

          // Full multi-cell diagonal legality check
          const ext = this._canMoveDiagonalLong(prev.x, prev.y, dx, dy);
          const end = { x: ext.endX, y: ext.endY };

          // Check if curr lies ALONG that diagonal corridor region
          const inCorridor =
            Math.sign(curr.x - prev.x) === dx &&
            Math.sign(curr.y - prev.y) === dy &&
            Math.abs(curr.x - prev.x) <= Math.abs(end.x - prev.x) &&
            Math.abs(curr.y - prev.y) <= Math.abs(end.y - prev.y);

          if (!inCorridor) continue;

          // Accept full diagonal extension across entire corridor
          out.push(end);
          extended = true;

          // Skip all path[i] that lie inside the diagonal corridor region
          while (i < path.length) {
            const pt = path[i];
            const ptInCorridor =
              Math.sign(pt.x - prev.x) === dx &&
              Math.sign(pt.y - prev.y) === dy &&
              Math.abs(pt.x - prev.x) <= Math.abs(end.x - prev.x) &&
              Math.abs(pt.y - prev.y) <= Math.abs(end.y - prev.y);

            if (!ptInCorridor) break;
            i++;
          }

          break; // Done trying diagonals at this node
        }

        if (!extended) {
          // No diagonal corridor found → use original step
          out.push(curr);
          i++;
        }
      }

      return out;
    }

    /**
     * Master smoothing pipeline for optimal path compression.
     * Produces Japanese-style mega-segments:
     *  1. Remove zigzags
     *  2. Remove collinear points
     *  3. Replace corners with diagonals
     *  4. Extend diagonals to full corridor length
     */
    _smoothGridPath(rawCells) {
      if (!rawCells || rawCells.length < 3) return rawCells;

      const smoothingLog = {
        stage: "path-smoothing",
        rawPath: {
          length: rawCells.length,
          cells: rawCells.map(c => `(${c.x},${c.y})`).join(' → ')
        },
        iterations: []
      };

      let path = [...rawCells];

      // Iteration loop for convergence
      let changed = true;
      let iterations = 0;
      const maxIterations = 10;

      while (changed && iterations < maxIterations) {
        iterations++;
        const before = path.length;
        const iterLog = { iteration: iterations, before };

        // Apply smoothing pipeline
        path = this._removeUselessReversals(path);
        iterLog.afterRemoveReversals = path.length;

        path = this._removeCollinear(path);
        iterLog.afterRemoveCollinear = path.length;

        path = this._cornerShortcuts(path);
        iterLog.afterCornerShortcuts = path.length;

        path = this._extendDiagonals(path);
        iterLog.afterExtendDiagonals = path.length;

        changed = path.length < before;
        iterLog.changed = changed;
        iterLog.cellsRemoved = before - path.length;

        smoothingLog.iterations.push(iterLog);
      }

      smoothingLog.finalPath = {
        length: path.length,
        cells: path.map(c => `(${c.x},${c.y})`).join(' → '),
        compressionRatio: ((1 - path.length / rawCells.length) * 100).toFixed(1) + '%',
        cellsRemoved: rawCells.length - path.length,
        totalIterations: iterations
      };

      console.log("[RacePlanner] Path Smoothing Result:", JSON.stringify(smoothingLog, null, 2));

      return path;
    }

    // ---------- Stage 2: compress to line segments ----------

    /**
     * Convert cell path [{x,y},...] to straight line segments.
     * Each segment: { type:"line", length (meters), angle (radians) }
     */
    _cellsToLineSegments(cells) {
      if (!cells || cells.length < 2) return [];

      const segs = [];
      const toAngle = (dx, dy) => Math.atan2(dy, dx);

      let i = 0;
      while (i < cells.length - 1) {
        const p0 = cells[i];
        const p1 = cells[i + 1];
        const dx1 = p1.x - p0.x;
        const dy1 = p1.y - p0.y;
        const angle = toAngle(dx1, dy1);
        const diagonal1 = Math.abs(dx1) === 1 && Math.abs(dy1) === 1;

        let lengthCells = diagonal1 ? Math.SQRT2 : 1.0;
        let j = i + 1;

        // Extend same direction
        while (j < cells.length - 1) {
          const a = cells[j];
          const b = cells[j + 1];
          const dx = b.x - a.x;
          const dy = b.y - a.y;
          const diag = Math.abs(dx) === 1 && Math.abs(dy) === 1;
          const ang = toAngle(dx, dy);
          if (Math.abs(ang - angle) > 1e-3) break;
          lengthCells += diag ? Math.SQRT2 : 1.0;
          j++;
        }

        const lengthMeters = lengthCells * this.cellSize;
        segs.push({
          type: "line",
          length: lengthMeters,
          angle: angle
        });

        i = j;
      }

      return segs;
    }

    /**
     * Classify turn type based on angle delta.
     * Returns: { mode: string, radius: number, description: string }
     */
    _classifyTurn(dTheta) {
      const QUARTER_PI = Math.PI / 4;
      const normTheta = this._normalizeAngle(dTheta);
      const absDTheta = Math.abs(normTheta);
      
      // Map angle deltas to turn types
      if (absDTheta < 0.1) return null; // Straight (skip)
      
      if (Math.abs(absDTheta - QUARTER_PI) < 0.1) {
        // 45° turn
        return {
          mode: dTheta > 0 ? "corner45Left" : "corner45Right",
          radius: this.cornerRadius,
          description: `45° ${dTheta > 0 ? "left" : "right"}`,
          angle: normTheta
        };
      }
      
      if (Math.abs(absDTheta - Math.PI / 2) < 0.1) {
        // 90° turn
        return {
          mode: dTheta > 0 ? "corner90Left" : "corner90Right",
          radius: this.cornerRadius * 0.8,  // Tighter for 90°
          description: `90° ${dTheta > 0 ? "left" : "right"}`,
          angle: normTheta
        };
      }
      
      if (Math.abs(absDTheta - 3 * QUARTER_PI) < 0.1) {
        // 135° turn (sharp)
        return {
          mode: dTheta > 0 ? "corner135Left" : "corner135Right",
          radius: this.cornerRadius * 0.6,  // Even tighter for 135°
          description: `135° ${dTheta > 0 ? "left" : "right"}`,
          angle: normTheta
        };
      }
      
      // Generic arc for any other angle
      return {
        mode: "arcR",
        radius: Math.max(this.cornerRadius * 0.4, Math.abs(this.vMax * this.vMax / (this.aLatMax || 7.0))),
        description: `Arc ${(absDTheta * 180 / Math.PI).toFixed(0)}°`,
        angle: normTheta
      };
    }

    /**
     * Normalize angle to [-π, π]
     */
    _normalizeAngle(a) {
      while (a <= -Math.PI) a += 2 * Math.PI;
      while (a >  Math.PI) a -= 2 * Math.PI;
      return a;
    }

    /**
     * Insert circular arcs at corners between line segments.
     * Enhanced version that generates Asian-style turn primitives.
     * 
     * Output: array of { type:"line"|"arc", length, angle, radius?, mode?, ccw?, 
     *                     centerX?, centerY?, startAngle?, endAngle? }
     * 
     * Turn modes: corner45Left/Right, corner90Left/Right, corner135Left/Right, arcR
     * 
     * Arc geometry includes full circle info for renderer to draw with ctx.arc()
     */
    _insertCornerArcs(lineSegments) {
      if (!lineSegments || lineSegments.length === 0) return [];

      const result = [];
      const r = this.cornerRadius;

      result.push({ type: "line", ...lineSegments[0] });

      // Track current position in 2D space for arc center calculation
      let currentX = 0;
      let currentY = 0;
      let currentAngle = lineSegments[0].angle;

      for (let i = 1; i < lineSegments.length; i++) {
        const prev = result[result.length - 1];
        const next = lineSegments[i];

        const a0 = prev.angle;
        const a1 = next.angle;
        let dTheta = this._normalizeAngle(a1 - a0);

        if (Math.abs(dTheta) < 1e-3) {
          // Same direction; extend previous
          prev.length += next.length;
          // Update position
          currentX += Math.cos(currentAngle) * next.length;
          currentY += Math.sin(currentAngle) * next.length;
          continue;
        }

        // Classify turn type
        const turnInfo = this._classifyTurn(dTheta);
        if (!turnInfo) {
          result.push({ type: "line", ...next });
          currentX += Math.cos(currentAngle) * next.length;
          currentY += Math.sin(currentAngle) * next.length;
          continue;
        }

        const arcRadius = turnInfo.radius;
        const absTheta = Math.abs(dTheta);
        const t = arcRadius * Math.tan(absTheta / 2);

        // If lines too short for arc, skip it (too tight corner)
        if (prev.length < 2 * t || next.length < 2 * t) {
          result.push({ type: "line", ...next });
          currentX += Math.cos(currentAngle) * next.length;
          currentY += Math.sin(currentAngle) * next.length;
          continue;
        }

        // === TRIM AND INSERT TURN ===

        // Trim previous line
        prev.length -= t;

        // Move to start of arc (end of trimmed previous line)
        currentX += Math.cos(currentAngle) * t;
        currentY += Math.sin(currentAngle) * t;

        // Compute arc center
        // Arc is perpendicular to current direction, offset by radius
        const perpAngle = currentAngle + (dTheta > 0 ? Math.PI / 2 : -Math.PI / 2);
        const centerX = currentX + arcRadius * Math.cos(perpAngle);
        const centerY = currentY + arcRadius * Math.sin(perpAngle);

        // Start angle: direction from center to current position
        const startAngle = Math.atan2(currentY - centerY, currentX - centerX);
        // End angle: after turning by dTheta
        const endAngle = startAngle + dTheta;

        // Insert arc with turn primitive + geometry
        const arc = {
          type: "arc",
          mode: turnInfo.mode,
          radius: arcRadius,
          angle: dTheta,
          length: Math.abs(dTheta) * arcRadius,
          ccw: dTheta > 0,
          description: turnInfo.description,
          // NEW: Full geometry for renderer
          centerX: centerX,
          centerY: centerY,
          startAngle: startAngle,
          endAngle: endAngle
        };
        console.log(`[RacePlanner._insertCornerArcs] Arc inserted: ${turnInfo.description}, center=(${centerX.toFixed(3)}, ${centerY.toFixed(3)}), radius=${arcRadius.toFixed(4)}, angles=[${startAngle.toFixed(3)}, ${endAngle.toFixed(3)}]`);
        result.push(arc);

        // Move to end of arc
        currentX = centerX + arcRadius * Math.cos(endAngle);
        currentY = centerY + arcRadius * Math.sin(endAngle);
        currentAngle = a1;

        // Add trimmed next line
        const trimmedNext = {
          type: "line",
          angle: next.angle,
          length: next.length - t
        };
        result.push(trimmedNext);
      }

      return result;
    }

    // ---------- Stage 3: time-optimal velocity planning ----------

    /**
     * Forward-backward pass for time-optimal velocity profile.
     * Returns { segmentProfiles: [...], vPoints: [...], totalTime }
     */
    _velocityPlan(segments) {
      const n = segments.length;
      if (!n) return { segmentProfiles: [], vPoints: [], totalTime: 0 };

      const L = segments.map(s => s.length);

      // Local velocity limits from curvature
      const vLimit = segments.map(seg => {
        if (seg.type === "line") return this.vMax;
        // Arc: v <= sqrt(aLatMax * r)
        const r = seg.radius;
        const vLat = Math.sqrt(Math.max(this.aLatMax, EPS) * Math.max(r, EPS));
        return Math.min(this.vMax, vLat);
      });

      const aMax = this.aMax;
      const dMax = this.dMax;

      const v = new Array(n + 1).fill(0);

      // Forward pass (accel-limited)
      v[0] = 0;
      for (let i = 0; i < n; i++) {
        const vIn = v[i];
        const vAcc = Math.sqrt(vIn * vIn + 2 * aMax * L[i]);
        v[i + 1] = Math.min(vAcc, vLimit[i]);
      }

      // Backward pass (decel-limited; end at 0)
      v[n] = 0;
      for (let i = n - 1; i >= 0; i--) {
        const vOut = v[i + 1];
        const vDec = Math.sqrt(vOut * vOut + 2 * dMax * L[i]);
        v[i] = Math.min(v[i], vDec);
      }

      // Compute times
      let totalTime = 0;
      const segmentProfiles = [];

      for (let i = 0; i < n; i++) {
        const vIn = v[i];
        const vOut = v[i + 1];
        const length = L[i];

        let dt;
        if (length < EPS) {
          dt = 0;
        } else if (Math.abs(vIn - vOut) < 1e-6) {
          // Approx constant speed
          const vAvg = Math.max((vIn + vOut) * 0.5, EPS);
          dt = length / vAvg;
        } else {
          // Constant accel: a = (vOut^2 - vIn^2) / (2 * L)
          const a = (vOut * vOut - vIn * vIn) / (2 * length);
          const aSafe = Math.sign(a) * Math.min(Math.abs(a), a > 0 ? aMax : dMax);
          dt = Math.abs((vOut - vIn) / (aSafe || EPS));
        }

        totalTime += dt;
        segmentProfiles.push({
          segment: segments[i],
          vIn,
          vOut,
          time: dt
        });
      }

      return { segmentProfiles, vPoints: v, totalTime };
    }

    // ---------- Public API ----------

    /**
     * Full race plan computation.
     * 
     * Returns:
     * {
     *   gridPath: [{x,y}, ...],
     *   segments: [{type:"line"/"arc", length, angle, radius?, ccw?}, ...],
     *   profiles: [{segment, vIn, vOut, time}, ...],
     *   totalTime: Number
     * }
     */
    computeRacePlan(startX = 0, startY = 0) {
      // Log maze structure at start of planning
      this.logMazeStructure();
      
      const rawCells = this.computeGridPath(startX, startY);
      if (!rawCells || rawCells.length < 2) {
        console.warn("RacePlanner: no path found");
        return null;
      }

      // Stage 1b: Smooth the path (remove zigzags, extend diagonals)
      const cells = this._smoothGridPath(rawCells);

      const lineSegments = this._cellsToLineSegments(cells);
      const raceSegments = this._insertCornerArcs(lineSegments);
      const { segmentProfiles, totalTime } = this._velocityPlan(raceSegments);

      const result = {
        gridPath: cells,
        segments: raceSegments,
        profiles: segmentProfiles,
        totalTime
      };

      console.log("[RacePlanner] Race plan output:", {
        gridPathLength: result.gridPath.length,
        segmentsLength: result.segments.length,
        profilesLength: result.profiles.length,
        totalTime: result.totalTime.toFixed(3),
        gridPath: result.gridPath,
        segments: result.segments,
        profiles: result.profiles
      });

      return result;
    }

    // ===== Backward compatibility adapters for maze-explorer.js =====

    /**
     * Set the wall map (maze configuration).
     * Adapter for maze-explorer.js compatibility.
     * Sanitizes the wall map to enable diagonal movement during race planning.
     */
    setWallMap(knownWalls) {
      this.knownWalls = this._sanitizeWallMap(knownWalls);
    }

    /**
     * Compute optimal path using oblique (8-direction) movement.
     * Adapter for maze-explorer.js: returns grid cell path only.
     */
    computeOptimalPathOblique(startX = 0, startY = 0) {
      return this.computeGridPath(startX, startY);
    }

    /**
     * Compress grid path to racing line segments.
     * Adapter for maze-explorer.js: takes cell array, returns segment array.
     */
    compressToRacingLine(gridPath) {
      if (!gridPath || gridPath.length < 2) return [];
      return this._cellsToLineSegments(gridPath);
    }

    /**
     * Compute velocity profiles for segments.
     * Adapter for maze-explorer.js: takes segments, returns profiles with timing.
     */
    computeProfilesForSegments(segments) {
      if (!segments || segments.length === 0) {
        return { segmentProfiles: [], totalTime: 0 };
      }
      return this._velocityPlan(segments);
    }
  }

  // --------- Minimal binary heap ---------

  class MinHeap {
    constructor() {
      this.heap = [];
    }

    isEmpty() {
      return this.heap.length === 0;
    }

    push(item) {
      this.heap.push(item);
      this._bubbleUp(this.heap.length - 1);
    }

    pop() {
      if (this.heap.length === 0) return null;
      if (this.heap.length === 1) return this.heap.pop();

      const root = this.heap[0];
      this.heap[0] = this.heap.pop();
      this._sinkDown(0);
      return root;
    }

    _bubbleUp(i) {
      while (i > 0) {
        const p = (i - 1) >> 1;
        if (this.heap[i][0] >= this.heap[p][0]) break;
        [this.heap[i], this.heap[p]] = [this.heap[p], this.heap[i]];
        i = p;
      }
    }

    _sinkDown(i) {
      const n = this.heap.length;
      while (true) {
        const l = 2 * i + 1;
        const r = 2 * i + 2;
        let s = i;
        if (l < n && this.heap[l][0] < this.heap[s][0]) s = l;
        if (r < n && this.heap[r][0] < this.heap[s][0]) s = r;
        if (s === i) break;
        [this.heap[i], this.heap[s]] = [this.heap[s], this.heap[i]];
        i = s;
      }
    }
  }

  global.RacePlanner = RacePlanner;
})(window);
