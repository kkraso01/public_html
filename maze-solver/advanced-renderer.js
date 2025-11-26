(function (global) {
  const DIRS = global.MAZE_DIRS;
  const DELTAS = global.MAZE_DELTAS;

  /**
   * Enhanced Maze Renderer with advanced visualization
   * Shows: search frontier, wall discoveries, velocity profiles, trajectory overlays
   */
  class AdvancedMazeRenderer {
    constructor(canvas, maze, solver) {
      this.canvas = canvas;
      this.ctx = canvas.getContext("2d");
      this.maze = maze;
      this.solver = solver;
      this.deviceRatio = window.devicePixelRatio || 1;
      
      this.visualizationMode = "heatmap"; // heatmap, frontier, walls, velocity
      this.showVelocityProfile = false;
      this.showTrajectory = false;
      this.searchFrontier = [];
      this.wallUpdates = [];
      this.forwardPath = [{ x: 0, y: 0 }];  // Forward exploration path (TO-GOAL phase)
      this.backwardPath = [];                 // Return path (RETURN phase)
      this.racingPath = [];                   // Racing line path (RACE phase)
      this.currentSolver = null; // Track the active solver (FloodSolver or LPASolver)
      this.plannedPath = null; // Planned optimal path
      this.currentPhase = "to-goal";
      this.discreteOptimalPath = null;       // Racing path from optimizer
      this.racingPathIndex = 0;               // Current position in racing path
      
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
      this.draw();
    }

    setVisualizationMode(mode) {
      this.visualizationMode = mode;
      this.draw();
    }

    _drawHeatmap() {
      const { ctx } = this;
      const cellSize = Math.min(this.canvas.clientWidth, this.canvas.clientHeight) / this.maze.size;
      
      // During racing phase, ONLY show racing path progress, ignore all old exploration data
      const isRacing = this.currentPhase === "race" && this.discreteOptimalPath && this.discreteOptimalPath.length > 0;
      
      if (isRacing) {
        // Racing phase: show only the racing path colored by progress (blue → red)
        for (let i = 0; i < this.discreteOptimalPath.length; i++) {
          const cell = this.discreteOptimalPath[i];
          const progress = i / (this.discreteOptimalPath.length - 1 || 1); // 0 at start, 1 at end
          
          // Heat gradient: cold (blue) -> hot (red)
          const t = progress;
          const hue = 240 * (1 - t); // 240=blue, 0=red
          const brightness = 45 + 25 * (1 - t);
          const saturation = 80 + 20 * t;
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

    recordRobotPosition(x, y, phase = "to-goal") {
      this.currentPhase = phase;
      
      if (phase === "to-goal") {
        if (this.forwardPath.length === 0 || 
            this.forwardPath[this.forwardPath.length - 1].x !== x || 
            this.forwardPath[this.forwardPath.length - 1].y !== y) {
          this.forwardPath.push({ x, y });
        }
      } else if (phase === "return") {
        if (this.backwardPath.length === 0 || 
            this.backwardPath[this.backwardPath.length - 1].x !== x || 
            this.backwardPath[this.backwardPath.length - 1].y !== y) {
          this.backwardPath.push({ x, y });
        }
      } else if (phase === "race") {
        if (this.racingPath.length === 0 || 
            this.racingPath[this.racingPath.length - 1].x !== x || 
            this.racingPath[this.racingPath.length - 1].y !== y) {
          this.racingPath.push({ x, y });
        }
      }
    }

    clearPath() {
      this.forwardPath = [{ x: 0, y: 0 }];
      this.backwardPath = [];
      this.racingPath = [];
      
      if (this.solver && this.solver.position) {
        this.forwardPath.push({ x: this.solver.position.x, y: this.solver.position.y });
      }
    }

    clearBackwardPath() {
      this.backwardPath = [];
    }

    setRacingPathInfo(discreteOptimalPath, racingPathIndex, phase) {
      this.discreteOptimalPath = discreteOptimalPath;
      this.racingPathIndex = racingPathIndex;
      this.currentPhase = phase;
      this.draw();
    }

    draw() {
      const { ctx } = this;
      ctx.save();
      ctx.setTransform(1, 0, 0, 1, 0, 0);
      ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);
      ctx.restore();
      
      // Draw based on mode
      if (this.visualizationMode === "heatmap") {
        this._drawHeatmap();
      } else if (this.visualizationMode === "walls") {
        this._drawWallUpdates();
      }
      
      this._drawGoals();
      this._drawGrid();
      this._drawWalls();
      this._drawPlannedPath();
      this._drawRobot();
    }

    updateSearchFrontier(frontier) {
      this.searchFrontier = frontier;
    }

    updateWallUpdates(updates) {
      this.wallUpdates = updates.slice(-5); // Keep last 5 updates
    }

    setCurrentSolver(solver) {
      this.currentSolver = solver;
    }

    setPlannedPath(path) {
      this.plannedPath = path;
      this.draw();
    }
  }

  global.AdvancedMazeRenderer = AdvancedMazeRenderer;
})(window);
