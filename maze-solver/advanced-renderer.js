(function (global) {
  const DIRS = global.MAZE_DIRS;
  const DELTAS = global.MAZE_DELTAS;

  /**
   * Enhanced Maze Renderer with advanced visualization
   * Shows: search frontier, wall discoveries, velocity profiles, trajectory overlays, racing paths
   * 
   * State Management:
   * - Maintains separate visualization modes and path tracking per phase
   * - Clears exploration data when entering race phase
   * - Properly resets all state on maze reset or generation
   */
  class AdvancedMazeRenderer {
    constructor(canvas, maze, solver) {
      this.canvas = canvas;
      this.ctx = canvas.getContext("2d");
      this.maze = maze;
      this.solver = solver;
      this.deviceRatio = window.devicePixelRatio || 1;
      
      // ==================== VISUALIZATION STATE ====================
      this.visualizationMode = "heatmap"; // heatmap, frontier, walls, velocity
      this.showVelocityProfile = false;
      this.showTrajectory = false;
      
      // ==================== EXPLORATION STATE ====================
      this.searchFrontier = [];              // LPA* frontier for heatmap
      this.wallUpdates = [];                 // Recent wall discovery events
      this.forwardPath = [];                 // TO-GOAL phase path (exploration to goal)
      this.backwardPath = [];                // RETURN phase path (return to start)
      this.racingPath = [];                  // RACE phase path (optimized racing line)
      
      // ==================== SOLVER STATE ====================
      this.currentSolver = null;             // Track active solver (FloodSolver or LPASolver)
      this.currentPhase = "to-goal";         // Current phase: to-goal, return, race, or optimize
      
      // ==================== PLANNING STATE ====================
      this.plannedPath = null;               // Planned optimal path (before racing)
      
      // ==================== RACING STATE ====================
      this.discreteOptimalPath = null;       // Waypoints from race planner
      this.racingSegments = null;            // Compressed segments for motion profile
      this.racingPathIndex = 0;              // Current position index in racing path
      this.totalRaceTime = 0;                // Total estimated race time
      this.raceWalls = null;                 // Discovered walls only (for renderer synchronization)
      this.racingPlannerMode = "4D";         // Track which planner was used ('4D' or '8D')
      
      this._onResize = () => this._resize();
      this._resize();
      window.addEventListener("resize", this._onResize);
    }

    dispose() {
      window.removeEventListener("resize", this._onResize);
    }

    _resize() {
      const { canvas } = this;
      const rect = canvas.getBoundingClientRect();
      this.scale = this.deviceRatio;
      canvas.width = rect.width * this.scale;
      canvas.height = rect.height * this.scale;
      this.ctx.setTransform(this.scale, 0, 0, this.scale, 0, 0);
      
      // Redraw after resize
      this.draw();
    }

    setVisualizationMode(mode) {
      this.visualizationMode = mode;
      this.draw();
    }

    _drawHeatmap() {
      const { ctx } = this;
      const cellSize = Math.min(this.canvas.clientWidth, this.canvas.clientHeight) / this.maze.size;
      
      // During racing phase, ONLY show racing path cells, ignore all old exploration data
      const isRacing = this.currentPhase === "race" && this.discreteOptimalPath && this.discreteOptimalPath.length > 0;
      
      if (isRacing) {
        // Racing phase: highlight ONLY the cells in the racing path with heat gradient
        const pathSet = new Set();
        for (let i = 0; i < this.discreteOptimalPath.length; i++) {
          const cell = this.discreteOptimalPath[i];
          pathSet.add(`${cell.x},${cell.y}`);
        }

        // Draw only cells that are in the racing path
        for (let i = 0; i < this.discreteOptimalPath.length; i++) {
          const cell = this.discreteOptimalPath[i];
          const progress = i / (this.discreteOptimalPath.length - 1 || 1); // 0 at start, 1 at end
          
          // Heat gradient: cold (blue) -> hot (red)
          const hue = 240 * (1 - progress); // 240=blue, 0=red
          const brightness = 45 + 25 * (1 - progress);
          const saturation = 80 + 20 * progress;
          ctx.fillStyle = `hsla(${hue}, ${saturation}%, ${brightness}%, 0.6)`;
          ctx.fillRect(cell.x * cellSize, cell.y * cellSize, cellSize, cellSize);
        }
        return; // Exit early - don't draw old exploration data during racing
      }
      
      // Exploration phase: show distance/frontier heatmap
      if (!this.solver.latestDistances && !this.searchFrontier.length) return;
      
      // Use search frontier if available (LPA*)
      if (this.searchFrontier.length > 0) {
        const frontier = new Map();
        for (const cell of this.searchFrontier) {
          frontier.set(`${cell.x},${cell.y}`, cell.g);
        }
        
        let max = 0;
        for (const val of frontier.values()) {
          if (Number.isFinite(val)) max = Math.max(max, val);
        }
        
        for (let y = 0; y < this.maze.size; y++) {
          for (let x = 0; x < this.maze.size; x++) {
            const val = frontier.get(`${x},${y}`);
            if (val === undefined) continue;
            if (!Number.isFinite(val)) continue;
            
            const t = max === 0 ? 0 : val / max;
            
            // Heat gradient: cold (blue) -> hot (red)
            const hue = 240 * (1 - t); // 240=blue, 0=red
            const brightness = 45 + 25 * (1 - t);
            const saturation = 80 + 20 * t;
            ctx.fillStyle = `hsla(${hue}, ${saturation}%, ${brightness}%, 0.6)`;
            ctx.fillRect(x * cellSize, y * cellSize, cellSize, cellSize);
            
            // Draw distance value
            if (Number.isFinite(val) && val < 30) {
              ctx.fillStyle = `rgba(226, 232, 240, ${0.3 + 0.2 * (1 - t)})`;
              ctx.font = `${Math.max(8, cellSize * 0.25)}px monospace`;
              ctx.textAlign = "center";
              ctx.textBaseline = "middle";
              ctx.fillText(Math.floor(val).toString(), x * cellSize + cellSize / 2, y * cellSize + cellSize / 2);
            }
          }
        }
      } else if (this.solver.latestDistances) {
        // Fallback: use distance heatmap
        let max = 0;
        this.solver.latestDistances.forEach(row => {
          row.forEach(v => {
            if (Number.isFinite(v)) max = Math.max(max, v);
          });
        });
        
        for (let y = 0; y < this.maze.size; y++) {
          for (let x = 0; x < this.maze.size; x++) {
            const d = this.solver.latestDistances[y][x];
            if (!Number.isFinite(d)) continue;
            
            const t = max === 0 ? 0 : d / max;
            
            const hue = 240 * (1 - t);
            const brightness = 45 + 25 * (1 - t);
            ctx.fillStyle = `hsla(${hue}, 80%, ${brightness}%, 0.5)`;
            ctx.fillRect(x * cellSize, y * cellSize, cellSize, cellSize);
            
            // Draw distance numbers
            if (d < 30 && Number.isFinite(d)) {
              ctx.fillStyle = `rgba(226, 232, 240, ${0.3 + 0.2 * (1 - t)})`;
              ctx.font = `${Math.max(8, cellSize * 0.25)}px monospace`;
              ctx.textAlign = "center";
              ctx.textBaseline = "middle";
              ctx.fillText(d.toString(), x * cellSize + cellSize / 2, y * cellSize + cellSize / 2);
            }
          }
        }
      }
    }

    _drawWallUpdates() {
      if (!this.wallUpdates || this.wallUpdates.length === 0) return;
      
      const { ctx } = this;
      const cellSize = Math.min(this.canvas.clientWidth, this.canvas.clientHeight) / this.maze.size;
      
      // Draw wall discovery events with fading effect
      for (let i = 0; i < this.wallUpdates.length; i++) {
        const update = this.wallUpdates[i];
        const age = 1 - (i / Math.max(1, this.wallUpdates.length)); // Recent = bright
        const opacity = age * 0.8;
        
        // Highlight the cell where wall was discovered
        ctx.fillStyle = `rgba(239, 68, 68, ${opacity * 0.4})`;
        const x = update.pos.x * cellSize;
        const y = update.pos.y * cellSize;
        ctx.fillRect(x, y, cellSize, cellSize);
        
        // Draw border
        ctx.strokeStyle = `rgba(239, 68, 68, ${opacity})`;
        ctx.lineWidth = 2;
        ctx.strokeRect(x, y, cellSize, cellSize);
        
        // Draw pulsing circle at center
        ctx.fillStyle = `rgba(239, 68, 68, ${opacity * 0.6})`;
        const pulse = Math.sin(Date.now() / 200) * 0.5 + 0.5;
        const radius = cellSize * 0.15 * (0.5 + pulse * 0.5);
        ctx.beginPath();
        ctx.arc(x + cellSize / 2, y + cellSize / 2, radius, 0, Math.PI * 2);
        ctx.fill();
      }
    }

    _drawGrid() {
      const { ctx } = this;
      const cellSize = Math.min(this.canvas.clientWidth, this.canvas.clientHeight) / this.maze.size;
      ctx.strokeStyle = "#0f172a";
      ctx.lineWidth = 1;
      for (let i = 0; i <= this.maze.size; i += 1) {
        ctx.beginPath();
        ctx.moveTo(0, i * cellSize);
        ctx.lineTo(this.maze.size * cellSize, i * cellSize);
        ctx.stroke();
        ctx.beginPath();
        ctx.moveTo(i * cellSize, 0);
        ctx.lineTo(i * cellSize, this.maze.size * cellSize);
        ctx.stroke();
      }
    }

    _drawWalls() {
      const { ctx } = this;
      const cellSize = Math.min(this.canvas.clientWidth, this.canvas.clientHeight) / this.maze.size;
      ctx.strokeStyle = "#94a3b8";
      ctx.lineWidth = 3;
      ctx.lineCap = "round";
      
      for (let y = 0; y < this.maze.size; y += 1) {
        for (let x = 0; x < this.maze.size; x += 1) {
          const cell = this.maze.cells[y][x];
          const baseX = x * cellSize;
          const baseY = y * cellSize;
          
          if (cell.north) {
            ctx.beginPath();
            ctx.moveTo(baseX, baseY);
            ctx.lineTo(baseX + cellSize, baseY);
            ctx.stroke();
          }
          if (cell.west) {
            ctx.beginPath();
            ctx.moveTo(baseX, baseY);
            ctx.lineTo(baseX, baseY + cellSize);
            ctx.stroke();
          }
          if (x === this.maze.size - 1 && cell.east) {
            ctx.beginPath();
            ctx.moveTo(baseX + cellSize, baseY);
            ctx.lineTo(baseX + cellSize, baseY + cellSize);
            ctx.stroke();
          }
          if (y === this.maze.size - 1 && cell.south) {
            ctx.beginPath();
            ctx.moveTo(baseX, baseY + cellSize);
            ctx.lineTo(baseX + cellSize, baseY + cellSize);
            ctx.stroke();
          }
        }
      }
    }

    _drawGoals() {
      const { ctx } = this;
      const cellSize = Math.min(this.canvas.clientWidth, this.canvas.clientHeight) / this.maze.size;
      ctx.fillStyle = "rgba(34,197,94,0.18)";
      this.maze.goalCells.forEach((key) => {
        const [x, y] = key.split(",").map(Number);
        ctx.fillRect(x * cellSize, y * cellSize, cellSize, cellSize);
      });
    }

    _drawPlannedPath() {
      if (!this.plannedPath || this.plannedPath.length === 0) return;
      // Skip drawing planned path during racing phase - use racing path instead
      if (this.currentPhase === "race") return;

      const { ctx } = this;
      const cellSize = Math.min(this.canvas.clientWidth, this.canvas.clientHeight) / this.maze.size;

      ctx.strokeStyle = "rgba(168, 85, 247, 0.8)"; // Purple for planned path
      ctx.lineWidth = cellSize * 0.4;
      ctx.lineCap = "round";
      ctx.lineJoin = "round";
      ctx.setLineDash([8, 4]);
      ctx.beginPath();

      for (let i = 0; i < this.plannedPath.length; i++) {
        const [x, y] = this.plannedPath[i].state.split(',').map(Number);
        const centerX = x * cellSize + cellSize / 2;
        const centerY = y * cellSize + cellSize / 2;
        if (i === 0) {
          ctx.moveTo(centerX, centerY);
        } else {
          ctx.lineTo(centerX, centerY);
        }
      }
      ctx.stroke();
      ctx.setLineDash([]);
    }

    _drawRacingPath() {
      // Draw optimized racing path with visual feedback
      if (!this.discreteOptimalPath || this.discreteOptimalPath.length === 0) return;
      if (this.currentPhase !== "race") return;

      const { ctx } = this;
      const cellSize = Math.min(this.canvas.clientWidth, this.canvas.clientHeight) / this.maze.size;

      // CRITICAL FIX: Use only discovered walls (raceWalls or knownWalls), NEVER the perfect maze
      // The renderer must visualize only what the robot has actually discovered
      const walls = this.raceWalls || (this.currentSolver && this.currentSolver.knownWalls) || this.maze.cells;
      const size = this.maze.size;

      // Helper: Check if wall exists in given direction
      const hasWall = (x, y, direction) => {
        if (x < 0 || y < 0 || x >= size || y >= size) return true;
        const cell = walls[y]?.[x];
        if (!cell) return false;
        return cell[direction] === true;
      };

      // Helper: Validate Asian diagonal legality
      // Diagonal movement requires: both primary cardinals free + opposite cardinals into target free
      const canMoveDiagonal = (x0, y0, x1, y1) => {
        if (x1 < 0 || y1 < 0 || x1 >= size || y1 >= size) return false;

        const dx = x1 - x0;
        const dy = y1 - y0;

        // Determine which cardinal directions are involved
        const dirX = dx > 0 ? "east" : "west";
        const dirY = dy > 0 ? "south" : "north";
        const oppX = dx > 0 ? "west" : "east";
        const oppY = dy > 0 ? "north" : "south";

        // Primary cardinals from source must be open
        if (hasWall(x0, y0, dirX)) return false;
        if (hasWall(x0, y0, dirY)) return false;

        // Opposite cardinals into target must be open
        if (hasWall(x1, y1, oppX)) return false;
        if (hasWall(x1, y1, oppY)) return false;

        // L-block check: neither corner cell can block both edges
        const cx1 = x0 + dx;
        const cy1 = y0;
        const cx2 = x0;
        const cy2 = y0 + dy;

        const cell1 = walls[cy1]?.[cx1];
        const cell2 = walls[cy2]?.[cx2];

        if (cell1 && cell1[oppY] === true && cell2 && cell2[oppX] === true) {
          return false; // L-block blocks diagonal
        }

        return true;
      };

      // Helper: Check cardinal movement (only one direction)
      const canMoveCardinal = (x0, y0, x1, y1) => {
        if (x1 < 0 || y1 < 0 || x1 >= size || y1 >= size) return false;

        const dx = x1 - x0;
        const dy = y1 - y0;

        if (dx > 0 && hasWall(x0, y0, "east")) return false;
        if (dx < 0 && hasWall(x0, y0, "west")) return false;
        if (dy > 0 && hasWall(x0, y0, "south")) return false;
        if (dy < 0 && hasWall(x0, y0, "north")) return false;

        return true;
      };

      // Helper: Check if movement is valid (respects wall legality)
      const canMoveTo = (x0, y0, x1, y1) => {
        const dx = Math.abs(x1 - x0);
        const dy = Math.abs(y1 - y0);

        if (dx === 0 && dy === 0) return true;
        if (dx <= 1 && dy <= 1) {
          // Adjacent cell (cardinal or diagonal)
          if (dx === 1 && dy === 1) {
            // Diagonal
            return canMoveDiagonal(x0, y0, x1, y1);
          } else {
            // Cardinal
            return canMoveCardinal(x0, y0, x1, y1);
          }
        }
        return false;
      };

      // Helper: Get all cells between two points, respecting walls
      const getCellsBetween = (p1, p2) => {
        const cells = [];
        let x0 = p1.x, y0 = p1.y;
        const x1 = p2.x, y1 = p2.y;

        const dx = x1 - x0;
        const dy = y1 - y0;
        const stepX = dx === 0 ? 0 : (dx > 0 ? 1 : -1);
        const stepY = dy === 0 ? 0 : (dy > 0 ? 1 : -1);

        cells.push({ x: x0, y: y0 });

        // Walk step-by-step from source to target, validating each move
        while (x0 !== x1 || y0 !== y1) {
          // Try to move closer: prioritize diagonal when possible
          let moved = false;

          const canDiag = stepX !== 0 && stepY !== 0 && canMoveTo(x0, y0, x0 + stepX, y0 + stepY);
          const canX = stepX !== 0 && canMoveTo(x0, y0, x0 + stepX, y0);
          const canY = stepY !== 0 && canMoveTo(x0, y0, x0, y0 + stepY);

          if (canDiag) {
            x0 += stepX;
            y0 += stepY;
            moved = true;
          } else if (canX) {
            x0 += stepX;
            moved = true;
          } else if (canY) {
            y0 += stepY;
            moved = true;
          }

          if (moved) {
            cells.push({ x: x0, y: y0 });
          } else {
            // Stuck - can't reach target due to walls
            break;
          }
        }

        return cells;
      };

      // Build complete interpolated path
      const fullPath = [];
      const waypoints = this.discreteOptimalPath;

      for (let i = 0; i < waypoints.length; i++) {
        if (i === 0) {
          fullPath.push(waypoints[i]);
        } else {
          const between = getCellsBetween(waypoints[i - 1], waypoints[i]);
          // Add all except first (already added)
          for (let j = 1; j < between.length; j++) {
            fullPath.push(between[j]);
          }
        }
      }

      // Draw background: YELLOW for skipped cells (interpolated), RED for planned waypoints
      // First pass: draw all cells yellow (skipped interpolated cells)
      ctx.fillStyle = "rgba(234, 179, 8, 0.35)"; // Yellow for skipped cells
      for (const cell of fullPath) {
        // Draw with slight overlap to eliminate seams
        ctx.fillRect(cell.x * cellSize - 0.5, cell.y * cellSize - 0.5, cellSize + 1, cellSize + 1);
      }

      // Second pass: overlay RED for planned waypoints
      ctx.fillStyle = "rgba(239, 68, 68, 0.45)"; // Red for planned cells
      for (const cell of waypoints) {
        ctx.fillRect(cell.x * cellSize - 0.5, cell.y * cellSize - 0.5, cellSize + 1, cellSize + 1);
      }

      // Draw the optimized racing path (waypoints only) with connecting lines
      ctx.strokeStyle = "rgba(239, 68, 68, 0.95)"; // Red line for racing path (planned waypoints)
      ctx.lineWidth = cellSize * 0.35;
      ctx.lineCap = "round";
      ctx.lineJoin = "round";
      ctx.beginPath();

      const startCell = waypoints[0];
      ctx.moveTo(startCell.x * cellSize + cellSize / 2, startCell.y * cellSize + cellSize / 2);

      for (let i = 1; i < waypoints.length; i++) {
        const cell = waypoints[i];
        ctx.lineTo(cell.x * cellSize + cellSize / 2, cell.y * cellSize + cellSize / 2);
      }
      ctx.stroke();

      // Draw waypoint markers (compressed path decision points) - RED for planned
      ctx.fillStyle = "rgba(239, 68, 68, 1.0)";
      for (let i = 0; i < waypoints.length; i++) {
        const cell = waypoints[i];
        const radius = cellSize * 0.15;
        ctx.beginPath();
        ctx.arc(cell.x * cellSize + cellSize / 2, cell.y * cellSize + cellSize / 2, radius, 0, Math.PI * 2);
        ctx.fill();

        // Outline for waypoints
        ctx.strokeStyle = "rgba(220, 38, 38, 0.9)";
        ctx.lineWidth = 2;
        ctx.stroke();
      }

      // Draw progress marker at current racing position (on interpolated path)
      if (this.racingPathIndex < fullPath.length) {
        const currentCell = fullPath[this.racingPathIndex];
        const progress = this.racingPathIndex / fullPath.length;

        // Highlight current cell with extended borders (no gaps)
        ctx.fillStyle = `rgba(239, 68, 68, ${0.55 + progress * 0.45})`;
        ctx.fillRect(currentCell.x * cellSize - 0.5, currentCell.y * cellSize - 0.5, cellSize + 1, cellSize + 1);

        // Robot position marker - bright and visible (red)
        ctx.fillStyle = "#ef4444";
        ctx.beginPath();
        ctx.arc(currentCell.x * cellSize + cellSize / 2, currentCell.y * cellSize + cellSize / 2, cellSize * 0.38, 0, Math.PI * 2);
        ctx.fill();

        ctx.strokeStyle = "#dc2626";
        ctx.lineWidth = 3;
        ctx.stroke();

        // Inner highlight circle
        ctx.fillStyle = "#fca5a5";
        ctx.beginPath();
        ctx.arc(currentCell.x * cellSize + cellSize / 2, currentCell.y * cellSize + cellSize / 2, cellSize * 0.2, 0, Math.PI * 2);
        ctx.fill();

        // Progress text badge with dark background
        ctx.fillStyle = "rgba(0, 0, 0, 0.6)";
        const progressText = `${this.racingPathIndex}/${fullPath.length}`;
        ctx.font = `bold ${Math.max(12, cellSize * 0.38)}px Arial`;
        const textMetrics = ctx.measureText(progressText);
        const textWidth = textMetrics.width;
        const textHeight = cellSize * 0.45;

        ctx.fillRect(
          currentCell.x * cellSize + cellSize / 2 - textWidth / 2 - 4,
          currentCell.y * cellSize + cellSize / 2 - textHeight / 2 - 3,
          textWidth + 8,
          textHeight + 6
        );

        ctx.fillStyle = "#ef4444";
        ctx.textAlign = "center";
        ctx.textBaseline = "middle";
        ctx.fillText(progressText, currentCell.x * cellSize + cellSize / 2, currentCell.y * cellSize + cellSize / 2);
      }

      // Draw direction arrows along waypoints (fewer for cleaner look)
      const step = Math.max(1, Math.floor(waypoints.length / 7));
      for (let i = 0; i < waypoints.length - 1; i += step) {
        const cell = waypoints[i];
        const nextCell = waypoints[i + 1];
        const dx = nextCell.x - cell.x;
        const dy = nextCell.y - cell.y;

        // Normalize direction for arrow angle
        const len = Math.sqrt(dx * dx + dy * dy);
        if (len === 0) continue;

        const ndx = dx / len;
        const ndy = dy / len;
        const angle = Math.atan2(ndy, ndx);

        const centerX = cell.x * cellSize + cellSize / 2;
        const centerY = cell.y * cellSize + cellSize / 2;
        const arrowSize = cellSize * 0.22;

        ctx.fillStyle = "rgba(34, 197, 94, 1.0)";
        ctx.beginPath();
        ctx.moveTo(centerX + Math.cos(angle) * arrowSize, centerY + Math.sin(angle) * arrowSize);
        ctx.lineTo(
          centerX + Math.cos(angle - Math.PI * 0.85) * arrowSize * 0.85,
          centerY + Math.sin(angle - Math.PI * 0.85) * arrowSize * 0.85
        );
        ctx.lineTo(
          centerX + Math.cos(angle + Math.PI * 0.85) * arrowSize * 0.85,
          centerY + Math.sin(angle + Math.PI * 0.85) * arrowSize * 0.85
        );
        ctx.fill();
      }

      // Draw mode indicator badge
      if (this.racingPlannerMode) {
        ctx.save();
        const modeText = `${this.racingPlannerMode} MODE`;
        const fontSize = cellSize * 0.75;
        ctx.font = `bold ${fontSize}px Arial`;
        const metrics = ctx.measureText(modeText);

        // Rounded background for mode indicator
        const padding = cellSize * 0.35;
        const bgX = this.canvas.width - metrics.width - padding - cellSize * 0.4;
        const bgY = cellSize * 0.25;
        const bgWidth = metrics.width + padding;
        const bgHeight = fontSize + cellSize * 0.35;

        // Draw background with slight rounding effect
        ctx.fillStyle = this.racingPlannerMode === '8D' ? "rgba(34, 197, 94, 0.98)" : "rgba(59, 130, 246, 0.98)";
        ctx.fillRect(bgX - 5, bgY, bgWidth + 10, bgHeight);

        // Text
        ctx.fillStyle = "rgba(255, 255, 255, 1.0)";
        ctx.textAlign = "right";
        ctx.textBaseline = "top";
        ctx.fillText(modeText, this.canvas.width - cellSize * 0.35, cellSize * 0.4);
        ctx.restore();
      }
    }

    _drawRobot() {
      const { ctx } = this;
      // Use the current solver (could be FloodSolver or LPASolver)
      const solver = this.currentSolver || this.solver;
      const { x, y } = solver.position;
      const cellSize = Math.min(this.canvas.clientWidth, this.canvas.clientHeight) / this.maze.size;
      const centerX = x * cellSize + cellSize / 2;
      const centerY = y * cellSize + cellSize / 2;
      const r = cellSize * 0.25 * 0.75;  // Reduced to 75% of original size
      
      // Draw path trails with different colors for phases
      // Forward path (TO-GOAL) - Blue
      if (this.forwardPath.length > 1) {
        ctx.strokeStyle = "rgba(59, 90, 246, 0.5)";  // Blue
        ctx.lineWidth = cellSize * 0.15;
        ctx.lineCap = "round";
        ctx.lineJoin = "round";
        ctx.beginPath();
        const start = this.forwardPath[0];
        ctx.moveTo(start.x * cellSize + cellSize / 2, start.y * cellSize + cellSize / 2);
        for (let i = 1; i < this.forwardPath.length; i++) {
          const pos = this.forwardPath[i];
          ctx.lineTo(pos.x * cellSize + cellSize / 2, pos.y * cellSize + cellSize / 2);
        }
        ctx.stroke();
      }

      // Backward path (RETURN) - Red
      if (this.backwardPath.length > 1) {
        ctx.strokeStyle = "rgba(239, 68, 68, 0.5)";  // Red
        ctx.lineWidth = cellSize * 0.15;
        ctx.lineCap = "round";
        ctx.lineJoin = "round";
        ctx.beginPath();
        const start = this.backwardPath[0];
        ctx.moveTo(start.x * cellSize + cellSize / 2, start.y * cellSize + cellSize / 2);
        for (let i = 1; i < this.backwardPath.length; i++) {
          const pos = this.backwardPath[i];
          ctx.lineTo(pos.x * cellSize + cellSize / 2, pos.y * cellSize + cellSize / 2);
        }
        ctx.stroke();
      }

      // Racing path (RACE) - Green
      if (this.racingPath.length > 1) {
        ctx.strokeStyle = "rgba(16, 185, 129, 0.6)";  // Green
        ctx.lineWidth = cellSize * 0.2;
        ctx.lineCap = "round";
        ctx.lineJoin = "round";
        ctx.beginPath();
        const start = this.racingPath[0];
        ctx.moveTo(start.x * cellSize + cellSize / 2, start.y * cellSize + cellSize / 2);
        for (let i = 1; i < this.racingPath.length; i++) {
          const pos = this.racingPath[i];
          ctx.lineTo(pos.x * cellSize + cellSize / 2, pos.y * cellSize + cellSize / 2);
        }
        ctx.stroke();
      }
      
      // Draw start position marker
      if (this.forwardPath.length > 0) {
        const startPos = this.forwardPath[0];
        const startCenterX = startPos.x * cellSize + cellSize / 2;
        const startCenterY = startPos.y * cellSize + cellSize / 2;
        
        // Draw start marker circle with star
        ctx.fillStyle = "rgba(59, 130, 246, 0.3)";
        ctx.beginPath();
        ctx.arc(startCenterX, startCenterY, cellSize * 0.35, 0, Math.PI * 2);
        ctx.fill();
        
        // Draw star symbol at start
        ctx.fillStyle = "#3b82f6";
        ctx.font = `bold ${Math.max(12, cellSize * 0.4)}px Arial`;
        ctx.textAlign = "center";
        ctx.textBaseline = "middle";
        ctx.fillText("⭐", startCenterX, startCenterY);
        
        // Draw direct line from start to current position
        ctx.strokeStyle = "rgba(59, 130, 246, 0.5)";
        ctx.lineWidth = 2;
        ctx.setLineDash([6, 4]);
        ctx.beginPath();
        ctx.moveTo(startCenterX, startCenterY);
        ctx.lineTo(centerX, centerY);
        ctx.stroke();
        ctx.setLineDash([]);
      }
      
      // Draw robot body
      ctx.fillStyle = "#fbbf24";
      ctx.strokeStyle = "#f59e0b";
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.arc(centerX, centerY, r, 0, Math.PI * 2);
      ctx.fill();
      ctx.stroke();

      // Heading arrow
      const dir = solver.heading;
      const angle = {
        north: -Math.PI / 2,
        east: 0,
        south: Math.PI / 2,
        west: Math.PI
      }[dir] ?? 0;
      
      ctx.strokeStyle = "#1e293b";
      ctx.lineWidth = 3;
      ctx.beginPath();
      ctx.moveTo(centerX, centerY);
      ctx.lineTo(centerX + Math.cos(angle) * r * 1.2, centerY + Math.sin(angle) * r * 1.2);
      ctx.stroke();
    }

    draw() {
      const { ctx } = this;
      
      // ==================== STEP 1: CLEAR CANVAS ====================
      // Full canvas clear in physical coordinates to ensure no artifacts
      ctx.save();
      ctx.setTransform(1, 0, 0, 1, 0, 0);
      ctx.fillStyle = "rgba(15, 23, 42, 1.0)"; // Dark background (#0f172a)
      ctx.fillRect(0, 0, this.canvas.width, this.canvas.height);
      ctx.restore();
      
      // ==================== STEP 2: DRAW BASE LAYERS ====================
      // Always draw maze structure
      this._drawGoals();
      this._drawGrid();
      this._drawWalls();
      
      // ==================== STEP 3: DRAW VISUALIZATION MODE ====================
      // Only show exploration heatmap/frontier during exploration phases
      if (this.currentPhase !== "race") {
        if (this.visualizationMode === "heatmap") {
          this._drawHeatmap();
        } else if (this.visualizationMode === "walls") {
          this._drawWallUpdates();
        }
      }
      
      // ==================== STEP 4: DRAW PATH PLANNING ====================
      // Show planned path only during planning/optimization phase
      if (this.currentPhase === "optimize") {
        this._drawPlannedPath();
      }
      
      // ==================== STEP 5: DRAW RACING PATH ====================
      // Show optimized racing path only during race phase
      if (this.currentPhase === "race") {
        this._drawRacingPath();
      }
      
      // ==================== STEP 6: DRAW ROBOT (ALWAYS ON TOP) ====================
      this._drawRobot();
    }

    recordRobotPosition(x, y, phase = "to-goal") {
      // Update phase and handle state transitions
      const phaseChanged = this.currentPhase !== phase;
      this.currentPhase = phase;
      
      // Record position based on phase
      if (phase === "to-goal") {
        // Exploration phase - build forward path
        if (this.forwardPath.length === 0 || 
            this.forwardPath[this.forwardPath.length - 1].x !== x || 
            this.forwardPath[this.forwardPath.length - 1].y !== y) {
          this.forwardPath.push({ x, y });
        }
      } else if (phase === "return") {
        // Return phase - build backward path
        // Clear forward path visualization on return (keep the data but switch focus)
        if (this.backwardPath.length === 0 || 
            this.backwardPath[this.backwardPath.length - 1].x !== x || 
            this.backwardPath[this.backwardPath.length - 1].y !== y) {
          this.backwardPath.push({ x, y });
        }
      } else if (phase === "optimize") {
        // Optimization phase - preparing race plan
        this.visualizationMode = "heatmap"; // Keep exploration viz during planning
      } else if (phase === "race") {
        // Racing phase - follow optimized path
        if (this.racingPath.length === 0 || 
            this.racingPath[this.racingPath.length - 1].x !== x || 
            this.racingPath[this.racingPath.length - 1].y !== y) {
          this.racingPath.push({ x, y });
        }
      }
    }

    clearPath() {
      // Reset all path tracking
      this.forwardPath = [];
      this.backwardPath = [];
      this.racingPath = [];
      
      // Reset phase and planning state
      this.currentPhase = "to-goal";
      this.plannedPath = null;
      
      // Reset racing state
      this.discreteOptimalPath = null;
      this.racingPathIndex = 0;
      this.racingSegments = null;
      this.totalRaceTime = 0;
      this.raceWalls = null;
      this.racingPlannerMode = "4D";
      
      // Reset exploration tracking
      this.searchFrontier = [];
      this.wallUpdates = [];
      
      // Set initial visualization to heatmap (exploration mode)
      this.visualizationMode = "heatmap";
      
      // Initialize start position if solver has position
      if (this.solver && this.solver.position) {
        this.forwardPath = [{ x: this.solver.position.x, y: this.solver.position.y }];
      }
      
      // Trigger redraw
      this.draw();
    }

    clearBackwardPath() {
      this.backwardPath = [];
      this.draw();
    }

    setRacingPathInfo(discreteOptimalPath, racingPathIndex, phase, plannerMode = '4D') {
      this.discreteOptimalPath = discreteOptimalPath;
      this.racingPathIndex = racingPathIndex;
      this.currentPhase = phase;
      this.racingPlannerMode = plannerMode;
      
      // Log racing progress
      if (phase === "race" && discreteOptimalPath && discreteOptimalPath.length > 0) {
        const progress = ((racingPathIndex / discreteOptimalPath.length) * 100).toFixed(1);
        console.log(`[Renderer] ${plannerMode} Racing - ${racingPathIndex}/${discreteOptimalPath.length} (${progress}%)`);
      }
      
      this.draw();
    }

    /**
     * Set sanitized walls from RacePlanner for accurate visualization.
     * This ensures the renderer uses the same wall map as the planner.
     */
    setRaceWalls(sanitizedWalls) {
      this.raceWalls = sanitizedWalls;
    }

    setPlannedPath(path) {
      this.plannedPath = path;
      this.currentPhase = "optimize";
      this.draw();
    }

    clearPlannedPath() {
      this.plannedPath = null;
      this.draw();
    }

    updateSearchFrontier(frontier) {
      this.searchFrontier = frontier || [];
    }

    updateWallUpdates(updates) {
      this.wallUpdates = updates ? updates.slice(-5) : [];
    }

    setCurrentSolver(solver) {
      this.currentSolver = solver;
    }

    /**
     * Initialize renderer for a new maze.
     * Call this after maze generation or reset.
     */
    initializeForMaze() {
      // Clear all exploration state
      this.forwardPath = [];
      this.backwardPath = [];
      this.racingPath = [];
      this.searchFrontier = [];
      this.wallUpdates = [];
      this.plannedPath = null;
      
      // Clear all racing state
      this.discreteOptimalPath = null;
      this.racingPathIndex = 0;
      this.racingSegments = null;
      this.totalRaceTime = 0;
      this.raceWalls = null;
      this.racingPlannerMode = "4D";
      
      // Reset phase and visualization
      this.currentPhase = "to-goal";
      this.visualizationMode = "heatmap";
      
      // Trigger redraw
      this.draw();
    }
  }

  global.AdvancedMazeRenderer = AdvancedMazeRenderer;
})(window);
