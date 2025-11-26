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
   * ObliquePathOptimizer: 8-direction path optimization for post-exploration racing
   * Used ONLY after maze is fully mapped (exploration complete)
   * 
   * This is the "second file" for oblique motion:
   * - Dijkstra with 8-direction support
   * - Diagonal lane validation
   * - Racing line compression with diagonal segments
   * - Time-optimal trajectory calculation
   */
  class ObliquePathOptimizer {
    constructor(maze, allowOblique = true) {
      this.maze = maze;
      this.size = maze.size;
      this.allowOblique = allowOblique;
      this.knownWalls = null;
    }

    /**
     * Set the wall map from exploration
     */
    setWallMap(wallMap) {
      this.knownWalls = wallMap;
    }

    _inBounds(x, y) {
      return x >= 0 && y >= 0 && x < this.size && y < this.size;
    }

    _diagonalMoveAllowed(x, y, dir) {
      const primary = dir.components;
      const nx = x + dir.dx;
      const ny = y + dir.dy;
      if (!this._inBounds(nx, ny)) return false;

      // Check primary directions (cardinal components) - wall is === true when it exists
      if (primary.some(d => this.knownWalls[y][x][d] === true)) return false;

      // Check corner cells for blocking walls
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

      return checks.every(([cx, cy, facing]) => 
        this._inBounds(cx, cy) && this.knownWalls[cy][cx][facing] !== true
      );
    }

    /**
     * Compute shortest path using 8-direction Dijkstra
     * Returns array of {x, y} cells
     */
    computeOptimalPathOblique(startX = 0, startY = 0) {
      if (!this.knownWalls) {
        console.warn("Wall map not set");
        return null;
      }

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
      distMap[startY][startX] = 0;

      const pq = [];
      pq.push({ cost: 0, x: startX, y: startY });

      while (pq.length) {
        pq.sort((a, b) => a.cost - b.cost);
        const { cost, x, y } = pq.shift();

        if (cost > distMap[y][x]) continue;
        
        // Check if at goal
        if (this.maze.goalCells.has(`${x},${y}`)) {
          return this._reconstructPath(prev, x, y);
        }

        // Explore all 8 directions
        for (const dir of DIAGONAL_DIRS) {
          if (dir.diagonal && !this._diagonalMoveAllowed(x, y, dir)) continue;
          if (!dir.diagonal && this.knownWalls[y][x][dir.name] === true) continue;

          const nx = x + dir.dx;
          const ny = y + dir.dy;
          if (!this._inBounds(nx, ny)) continue;

          const stepCost = dir.diagonal ? Math.SQRT2 : 1;
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

    _reconstructPath(prev, goalX, goalY) {
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
     * Compress path into racing segments (straights + turns with diagonal support)
     */
    compressToRacingLineOblique(gridPath) {
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
          distance: dir.diagonal ? length * Math.SQRT2 : length,
          isDiagonal: dir.diagonal
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
      if (Math.abs(diff) === 90 || Math.abs(diff) === 45 || Math.abs(diff) === 135 || Math.abs(diff) === 180) {
        return {
          angle: Math.abs(diff),
          direction: diff > 0 ? "left" : "right"
        };
      }
      return null;
    }

    /**
     * Calculate time-optimal motion profile for oblique segments
     */
    computeSegmentProfile(segment, vMax = 1.0, aMax = 0.5, dMax = 0.5) {
      const distance = segment.distance;
      
      // Simple trapezoidal profile
      const accelDist = Math.min(distance / 2, vMax * vMax / (2 * aMax));
      const cruiseDist = Math.max(0, distance - 2 * accelDist);
      const accelTime = vMax / aMax;
      const cruiseTime = cruiseDist / vMax;
      const decelTime = vMax / dMax;
      
      return {
        totalTime: accelTime + cruiseTime + decelTime,
        accelDist,
        cruiseDist,
        decelDist: accelDist,
        peakVelocity: vMax
      };
    }
  }

  global.ObliquePathOptimizer = ObliquePathOptimizer;
})(window);
