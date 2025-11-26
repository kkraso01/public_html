(function (global) {
  const DIRS = global.MAZE_DIRS;
  const DELTAS = global.MAZE_DELTAS;

  class MazeRenderer {
    constructor(canvas, maze, solver) {
      this.canvas = canvas;
      this.ctx = canvas.getContext("2d");
      this.maze = maze;
      this.solver = solver;
      this.deviceRatio = window.devicePixelRatio || 1;
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

    _drawDistanceMap() {
      if (!this.solver.latestDistances) return;
      const { ctx } = this;
      const cellSize = Math.min(this.canvas.clientWidth, this.canvas.clientHeight) / this.maze.size;
      let max = 0;
      this.solver.latestDistances.forEach((row) => row.forEach((v) => {
        if (Number.isFinite(v) && v > max) max = v;
      }));
      for (let y = 0; y < this.maze.size; y += 1) {
        for (let x = 0; x < this.maze.size; x += 1) {
          const d = this.solver.latestDistances[y][x];
          if (!Number.isFinite(d)) continue;
          const t = max === 0 ? 0 : d / max;
          ctx.fillStyle = `rgba(99, 102, 241, ${0.08 + 0.35 * (1 - t)})`;
          ctx.fillRect(x * cellSize, y * cellSize, cellSize, cellSize);
          ctx.fillStyle = "#dbeafe";
          ctx.font = `${Math.max(10, cellSize * 0.35)}px Inter, system-ui`;
          ctx.textAlign = "center";
          ctx.textBaseline = "middle";
          ctx.fillText(d.toString(), x * cellSize + cellSize / 2, y * cellSize + cellSize / 2);
        }
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
          // Draw east and south walls only on boundary to avoid duplicates
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

    _drawRobot() {
      const { ctx } = this;
      const { x, y } = this.solver.position;
      const cellSize = Math.min(this.canvas.clientWidth, this.canvas.clientHeight) / this.maze.size;
      const centerX = x * cellSize + cellSize / 2;
      const centerY = y * cellSize + cellSize / 2;
      const r = cellSize * 0.25;
      ctx.fillStyle = "#fbbf24";
      ctx.strokeStyle = "#f59e0b";
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.arc(centerX, centerY, r, 0, Math.PI * 2);
      ctx.fill();
      ctx.stroke();

      // Heading arrow
      const dir = this.solver.heading;
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
      ctx.save();
      ctx.setTransform(1, 0, 0, 1, 0, 0);
      ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);
      ctx.restore();
      this._drawDistanceMap();
      this._drawGoals();
      this._drawGrid();
      this._drawWalls();
      this._drawRobot();
    }
  }

  class MazeUI {
    
    constructor(canvasId) {
      const canvas = document.getElementById(canvasId);
      if (!canvas) return;
      this.canvas = canvas;
      this.maze = new global.Maze();
      this.solver = new global.FloodSolver(this.maze);
      this.lpaSolver = typeof global.LPASolver !== 'undefined' ? new global.LPASolver(this.maze) : null;
      this.motionProfile = typeof global.MotionProfile !== 'undefined' ? new global.MotionProfile() : null;
      this.trajectoryOptimizer = typeof global.TrajectoryOptimizer !== 'undefined' && this.motionProfile 
        ? new global.TrajectoryOptimizer(this.motionProfile) : null;
      this.pathPlanner = typeof global.PathPlanner !== 'undefined' && this.motionProfile 
        ? new global.PathPlanner(this.maze, this.motionProfile) : null;
      
      // Use advanced renderer
      this.renderer = typeof global.AdvancedMazeRenderer !== 'undefined' 
        ? new global.AdvancedMazeRenderer(canvas, this.maze, this.solver)
        : new MazeRenderer(canvas, this.maze, this.solver);
      
      this.interval = null;
      this.useLPA = false;
      this.currentAlgorithm = "dynamic-explorer";
      this.explorerType = "dynamic"; // dynamic (Chinese FF) or bfs
      this.allowObliqueSprint = false;
      this.stepDelay = 200; // milliseconds between steps
      this.mode = "explore"; // explore, plan, speedrun
      this.plannedPath = null;
      this.originalPlannedPath = null;
      this.speedRunCount = 0;
      this.stats = {
        nodesExplored: 0,
        timeSeconds: 0,
        wallDiscoveries: 0,
        timeSaved: 0
      };
      
      // Initialize explorer with BFS by default
      this._initializeExplorer();
      
      this._bindControls();
      this._createAlgorithmSelect();
      // Remove custom visualization checkboxes: do not call _createVizModeSelect()
      this.solver.computeDistances();
      this._createSpeedControls();
      this.renderer.draw();
    }

    _bindControls() {
      const generateBtn = document.getElementById("maze-generate");
      const runBtn = document.getElementById("maze-run");
      const stepBtn = document.getElementById("maze-step");
      const resetBtn = document.getElementById("maze-reset");
      const exploreBtn = document.getElementById("maze-explore");
      const planBtn = document.getElementById("maze-plan");
      const speedRunBtn = document.getElementById("maze-speedrun");

      generateBtn?.addEventListener("click", () => {
        this.stop();
        this.maze.generate();
        this.solver = new global.FloodSolver(this.maze);
        if (this.lpaSolver && typeof global.LPASolver !== 'undefined') {
          this.lpaSolver = new global.LPASolver(this.maze);
        }
        this.renderer.dispose();
        this.renderer = typeof global.AdvancedMazeRenderer !== 'undefined' 
          ? new global.AdvancedMazeRenderer(this.canvas, this.maze, this.solver)
          : new MazeRenderer(this.canvas, this.maze, this.solver);
        // Set the current solver in renderer
        if (this.renderer.setCurrentSolver) {
          this.renderer.setCurrentSolver(this.useLPA && this.lpaSolver ? this.lpaSolver : this.solver);
        }
        if (this.renderer.clearPath) {
          this.renderer.clearPath();
        }
        // Set initial visualization mode based on current algorithm
        if (this.renderer.setVisualizationMode) {
          const mode = this.useLPA ? "walls" : "heatmap";
          this.renderer.setVisualizationMode(mode);
        }
        this.solver.computeDistances();
        this._resetStats();
        this.renderer.draw();
      });

      resetBtn?.addEventListener("click", () => {
        this.stop();
        if (this.useLPA && this.lpaSolver) {
          this.lpaSolver.reset();
        } else {
          this.solver.reset();
          this.solver.computeDistances();
        }
        // Reset explorer
        if (this.explorer) {
          this.explorer.reset();
        }
        // Clear all paths
        this.plannedPath = [];
        this.originalPlannedPath = [];
        if (this.renderer.clearPath) {
          this.renderer.clearPath();
        }
        // Set the current solver in renderer
        if (this.renderer.setCurrentSolver) {
          this.renderer.setCurrentSolver(this.useLPA && this.lpaSolver ? this.lpaSolver : this.solver);
        }
        // Ensure correct visualization mode is set
        if (this.renderer.setVisualizationMode) {
          const mode = this.useLPA ? "walls" : "heatmap";
          this.renderer.setVisualizationMode(mode);
        }
        this._resetStats();
        this.renderer.draw();
      });

      stepBtn?.addEventListener("click", () => this._stepOnce());

      runBtn?.addEventListener("click", () => {
        if (this.interval) {
          this.stop();
          return;
        }
        runBtn.textContent = "Pause";
        if (this.mode === "explore") {
          this.interval = setInterval(() => {
            this._runExploreStep();
          }, this.stepDelay);
        } else if (this.mode === "plan") {
          this._planOptimalPath();
        } else if (this.mode === "speedrun") {
          this._speedRun();
        }
      });

      // Mode radio buttons removed - always explore mode now
    }

    _runExploreStep() {
      // Sync solver position with explorer for rendering
      this.solver.position = { ...this.explorer.position };
      this.solver.heading = this.explorer.heading;
      this.solver.latestDistances = this.explorer?.distances || this.solver.latestDistances;
      if (this.renderer.recordRobotPosition) {
        this.renderer.recordRobotPosition(this.explorer.position.x, this.explorer.position.y);
      }
      this.renderer.draw();
      this._updateStatsDisplay();

      const result = this.explorer.step();
      if (result.done) {
        // Exploration complete
        this.stop();
        
        // After exploration, set wall map for planner
        if (this.pathPlanner) {
          this.pathPlanner.setWallMap(this.explorer.knownWalls);
        }
        
        // Automatically compute and execute the optimal path
        this._executeOptimalPath();
      }
    }

    _executeOptimalPath() {
      if (!this.pathPlanner || !this.pathPlanner.graph) {
        console.warn("Path planner not available");
        return;
      }
      
      this.plannedPath = this.pathPlanner.computeOptimalPath();
      if (this.plannedPath) {
        this.originalPlannedPath = this.plannedPath.slice();
        console.log("Exploration complete. Returning to start for optimal run...");
        
        // Return robot to start position
        this.solver.position = { x: 0, y: 0 };
        this.solver.heading = "east";
        this.renderer.draw();
        
        // Wait a moment, then start the optimal run
        setTimeout(() => {
          this._executeSpeedRun();
        }, 500);
      } else {
        console.warn("No path found to goal");
      }
    }

    _executeSpeedRun() {
      this.mode = "speedrun";
      this.speedRunCount = 0;
      this.plannedPath = this.originalPlannedPath.slice();
      
      // Run at high speed
      this.stepDelay = 50;
      this._updateSpeedButtonStates();
      this.interval = setInterval(() => {
        this._followPlannedPath();
      }, this.stepDelay);
    }

    stop() {
      if (this.interval) {
        clearInterval(this.interval);
        this.interval = null;
        const runBtn = document.getElementById("maze-run");
        if (runBtn) runBtn.textContent = "Run";
      }
    }

    _createAlgorithmSelect() {
      const container = document.getElementById("maze-controls");
      if (!container) return null;
      
      const section = document.createElement("div");
      section.style.marginBottom = "12px";
      
      const label = document.createElement("label");
      label.style.display = "block";
      label.style.fontWeight = "600";
      label.style.marginBottom = "8px";
      label.style.color = "#cbd5e1";
      label.style.fontSize = "0.9rem";
      label.style.textTransform = "uppercase";
      label.style.letterSpacing = "0.5px";
      label.textContent = "Algorithm";
      section.appendChild(label);
      
      const radioGroup = document.createElement("div");
      radioGroup.className = "radio-group";

      // Dynamic flood-fill explorer (Chinese enhancements)
      const dynamicOption = document.createElement("label");
      dynamicOption.className = "radio-option";
      const dynamicRadio = document.createElement("input");
      dynamicRadio.type = "radio";
      dynamicRadio.name = "maze-algorithm";
      dynamicRadio.value = "dynamic-explorer";
      dynamicRadio.checked = true;
      dynamicRadio.addEventListener("change", () => {
        this.useLPA = false;
        this.currentAlgorithm = "dynamic-explorer";
        this.explorerType = "dynamic";
        this._resetStats();
        this.stop();
        this._initializeExplorer();
        if (this.renderer.setVisualizationMode) {
          this.renderer.setVisualizationMode("heatmap");
        }
        if (this.renderer.clearPath) {
          this.renderer.clearPath();
        }
        this.renderer.draw();
      });
      dynamicOption.appendChild(dynamicRadio);
      dynamicOption.appendChild(document.createTextNode("Dynamic Flood-Fill Explorer"));
      radioGroup.appendChild(dynamicOption);

      // Flood Fill (baseline)
      const floodOption = document.createElement("label");
      floodOption.className = "radio-option";
      const floodRadio = document.createElement("input");
      floodRadio.type = "radio";
      floodRadio.name = "maze-algorithm";
      floodRadio.value = "flood-fill";
      floodRadio.addEventListener("change", () => {
        this.useLPA = false;
        this.currentAlgorithm = "flood-fill";
        this.explorerType = "bfs";
        this._resetStats();
        this.stop();
        this._initializeExplorer();
        if (this.renderer.setVisualizationMode) {
          this.renderer.setVisualizationMode("heatmap");
        }
        if (this.renderer.clearPath) {
          this.renderer.clearPath();
        }
        this.renderer.draw();
      });
      floodOption.appendChild(floodRadio);
      floodOption.appendChild(document.createTextNode("Flood Fill"));
      radioGroup.appendChild(floodOption);

      // LPA*
      const lpaOption = document.createElement("label");
      lpaOption.className = "radio-option";
      const lpaRadio = document.createElement("input");
      lpaRadio.type = "radio";
      lpaRadio.name = "maze-algorithm";
      lpaRadio.value = "lpa";
      lpaRadio.addEventListener("change", () => {
        this.useLPA = true;
        this.currentAlgorithm = "lpa";
        this.explorerType = "bfs";
        this._resetStats();
        this.stop();
        this._initializeExplorer();
        if (this.renderer.setVisualizationMode) {
          this.renderer.setVisualizationMode("walls");
        }
        if (this.renderer.clearPath) {
          this.renderer.clearPath();
        }
        this.renderer.draw();
      });
      lpaOption.appendChild(lpaRadio);
      lpaOption.appendChild(document.createTextNode("LPA*"));
      radioGroup.appendChild(lpaOption);

      /*
      // BFS Explorer
      const bfsOption = document.createElement("label");
      bfsOption.className = "radio-option";
      const bfsRadio = document.createElement("input");
      bfsRadio.type = "radio";
      bfsRadio.name = "maze-algorithm";
      bfsRadio.value = "bfs-explorer";
      bfsRadio.addEventListener("change", () => {
        this.useLPA = false;
        this.currentAlgorithm = "explore";
        this.explorerType = "bfs";
        this._resetStats();
        this.stop();
        this._initializeExplorer();
        if (this.renderer.setVisualizationMode) {
          this.renderer.setVisualizationMode("heatmap");
        }
        if (this.renderer.clearPath) {
          this.renderer.clearPath();
        }
        this.renderer.draw();
      });
      bfsOption.appendChild(bfsRadio);
      bfsOption.appendChild(document.createTextNode("BFS Explorer"));
      radioGroup.appendChild(bfsOption);
      */

      section.appendChild(radioGroup);

      const obliqueToggle = document.createElement("label");
      obliqueToggle.className = "radio-option";
      obliqueToggle.style.marginTop = "8px";
      const obliqueCheckbox = document.createElement("input");
      obliqueCheckbox.type = "checkbox";
      obliqueCheckbox.checked = this.allowObliqueSprint;
      obliqueCheckbox.addEventListener("change", (e) => {
        this.allowObliqueSprint = e.target.checked;
        if (this.explorer && this.explorer.allowOblique !== undefined) {
          this.explorer.allowOblique = this.allowObliqueSprint;
          this.explorer._dynamicFloodFill();
          this.explorer._computeDeadZones();
        }
      });
      obliqueToggle.appendChild(obliqueCheckbox);
      obliqueToggle.appendChild(document.createTextNode("Enable Oblique Sprint (8-dir)"));
      section.appendChild(obliqueToggle);
      container.appendChild(section);
      return null;
    }

    _initializeExplorer() {
      if (this.explorerType === "bfs" && typeof global.BFSExplorer !== 'undefined') {
        this.explorer = new global.BFSExplorer(this.maze);
      } else {
        this.explorer = typeof global.MazeExplorer !== 'undefined'
          ? new global.MazeExplorer(this.maze, { allowOblique: this.allowObliqueSprint })
          : null;
      }
    }

    // _createVizModeSelect removed: always use the best visualization for each algorithm

    _resetStats() {
      this.stats = {
        nodesExplored: 0,
        timeSeconds: 0,
        wallDiscoveries: 0,
        timeSaved: 0,
        startTime: Date.now()
      };
    }

    _createSpeedControls() {
      const container = document.getElementById("maze-controls");
      if (!container) return;

      const speedSection = document.createElement("div");
      speedSection.style.marginTop = "12px";
      speedSection.style.marginBottom = "12px";
      
      const label = document.createElement("label");
      label.style.display = "block";
      label.style.fontWeight = "600";
      label.style.marginBottom = "8px";
      label.style.color = "#cbd5e1";
      label.style.fontSize = "0.9rem";
      label.style.textTransform = "uppercase";
      label.style.letterSpacing = "0.5px";
      label.textContent = "Execution Speed";
      speedSection.appendChild(label);

      const buttonGrid = document.createElement("div");
      buttonGrid.style.display = "grid";
      buttonGrid.style.gridTemplateColumns = "1fr 1fr 1fr 1fr";
      buttonGrid.style.gap = "8px";

      const speeds = [
        { id: "speed-fast", label: "Max (50ms)", delay: 50 },
        { id: "speed-normal", label: "Normal (200ms)", delay: 200 },
        { id: "speed-slow", label: "Slow (500ms)", delay: 500 },
        { id: "speed-slowest", label: "Slowest (1s)", delay: 1000 }
      ];

      speeds.forEach(speed => {
        const button = document.createElement("button");
        button.id = speed.id;
        button.textContent = speed.label;
        button.style.padding = "8px 12px";
        button.style.fontSize = "0.8rem";
        button.style.fontWeight = "600";
        button.style.border = "1px solid rgba(99, 102, 241, 0.6)";
        button.style.borderRadius = "6px";
        button.style.color = "#e2e8f0";
        button.style.cursor = "pointer";
        button.style.transition = "all 0.2s ease";
        button.style.background = "linear-gradient(135deg, #4f46e5 0%, #6366f1 100%)";
        
        if (speed.delay === 200) {
          button.style.background = "linear-gradient(135deg, #10b981 0%, #059669 100%)";
          button.style.boxShadow = "0 4px 8px rgba(16, 185, 129, 0.4)";
        }

        button.addEventListener("click", () => {
          this.stepDelay = speed.delay;
          this._updateSpeedButtonStates();
        });

        button.addEventListener("mouseover", () => {
          button.style.transform = "translateY(-2px)";
          button.style.boxShadow = "0 4px 12px rgba(99, 102, 241, 0.4)";
        });

        button.addEventListener("mouseout", () => {
          button.style.transform = "translateY(0)";
          if (speed.delay !== this.stepDelay) {
            button.style.boxShadow = "none";
          }
        });

        buttonGrid.appendChild(button);
      });

      speedSection.appendChild(buttonGrid);
      container.appendChild(speedSection);
    }

    _updateSpeedButtonStates() {
      const buttons = [
        { id: "speed-fast", delay: 50 },
        { id: "speed-normal", delay: 200 },
        { id: "speed-slow", delay: 500 },
        { id: "speed-slowest", delay: 1000 }
      ];

      buttons.forEach(btn => {
        const el = document.getElementById(btn.id);
        if (el) {
          if (btn.delay === this.stepDelay) {
            el.style.background = "linear-gradient(135deg, #10b981 0%, #059669 100%)";
            el.style.boxShadow = "0 4px 8px rgba(16, 185, 129, 0.4)";
          } else {
            el.style.background = "linear-gradient(135deg, #4f46e5 0%, #6366f1 100%)";
            el.style.boxShadow = "none";
          }
        }
      });

      // Restart interval if running
      if (this.interval) {
        clearInterval(this.interval);
        const runBtn = document.getElementById("maze-run");
        this.interval = setInterval(() => {
          const result = this._stepOnce();
          if (result?.done) this.stop();
        }, this.stepDelay);
      }
    }

    _stepOnce() {
      let result;
      const prevPos = { ...this.solver.position };
      
      if (this.useLPA && this.lpaSolver) {
        result = this.lpaSolver.step();
        this.stats.wallDiscoveries = this.lpaSolver.getWallUpdateHistory().length;
        
        // Set the active solver in renderer
        if (this.renderer.setCurrentSolver) {
          this.renderer.setCurrentSolver(this.lpaSolver);
        }
        
        // Update renderer with frontier
        if (this.renderer.updateSearchFrontier) {
          this.renderer.updateSearchFrontier(this.lpaSolver.getSearchFrontier());
        }
        if (this.renderer.updateWallUpdates) {
          this.renderer.updateWallUpdates(this.lpaSolver.getWallUpdateHistory());
        }
        
        this.stats.nodesExplored = this.lpaSolver.searchCount;
        
        // Track position change
        if (this.renderer.recordRobotPosition) {
          this.renderer.recordRobotPosition(this.lpaSolver.position.x, this.lpaSolver.position.y);
        }
      } else {
        result = this.solver.step();
        
        // Set the active solver in renderer
        if (this.renderer.setCurrentSolver) {
          this.renderer.setCurrentSolver(this.solver);
        }
        
        // Update solver's latestDistances for renderer
        if (!this.solver.latestDistances) {
          this.solver.computeDistances();
        }
        
        // Track position change
        if (this.renderer.recordRobotPosition) {
          this.renderer.recordRobotPosition(this.solver.position.x, this.solver.position.y);
        }
      }
      
      this.stats.timeSeconds = (Date.now() - this.stats.startTime) / 1000;
      this.renderer.draw();
      this._updateStatsDisplay();
      return result;
    }

    _updateStatsDisplay() {
      const statsEl = document.getElementById("maze-stats");
      if (!statsEl) return;
      
      const timeSaved = this.useLPA 
        ? Math.max(0, (this.stats.nodesExplored * 0.05 - this.stats.timeSeconds)).toFixed(2)
        : 0;
      
      statsEl.innerHTML = `
        <div style="font-size: 12px; color: #64748b;">
          <p>Algorithm: ${this.currentAlgorithm.toUpperCase()}</p>
          <p>Nodes Explored: ${this.stats.nodesExplored}</p>
          <p>Wall Discoveries: ${this.stats.wallDiscoveries}</p>
          <p>Elapsed: ${this.stats.timeSeconds.toFixed(2)}s</p>
          <p>Estimated Time Saved: ${timeSaved}s (LPA*)</p>
        </div>
      `;
    }

    _planOptimalPath() {
      if (!this.pathPlanner || !this.pathPlanner.graph) {
        alert("Please explore the maze first to build the wall map.");
        return;
      }
      this.mode = "plan";
      this.plannedPath = this.pathPlanner.computeOptimalPath();
      if (this.plannedPath) {
        this.originalPlannedPath = this.plannedPath.slice();
        const compressed = this.pathPlanner.compressPath(this.plannedPath);
        console.log("Planned path:", compressed);
        // Visualize the path
        if (this.renderer.setPlannedPath) {
          this.renderer.setPlannedPath(this.plannedPath);
        }
        this.renderer.draw();
      } else {
        alert("No path found.");
      }
    }

    _speedRun() {
      if (!this.originalPlannedPath) {
        alert("Please explore the maze first to auto-generate the optimal path.");
        return;
      }
      this.stop();
      this._executeSpeedRun();
    }

    _followPlannedPath() {
      if (!this.plannedPath || this.plannedPath.length === 0) {
        // Check if at goal
        if (this.solver.atGoal()) {
          this.speedRunCount++;
          console.log(`Speed run ${this.speedRunCount} completed!`);
          if (this.speedRunCount >= 3) {
            this.stop();
            return;
          }
          // Reset for next run
          this.solver.reset();
          this.plannedPath = this.originalPlannedPath.slice();
          if (this.renderer.clearPath) {
            this.renderer.clearPath();
          }
        }
        return;
      }

      // Get next state
      const nextStep = this.plannedPath.shift();
      const [x, y, heading] = nextStep.state.split(',').map((v, i) => i < 2 ? parseInt(v) : v);

      // Move solver to that position
      this.solver.position = { x, y };
      this.solver.heading = heading;

      // Record position
      if (this.renderer.recordRobotPosition) {
        this.renderer.recordRobotPosition(x, y);
      }

      this.renderer.draw();
    }

    draw() {
      const { ctx } = this;
      ctx.save();
      ctx.setTransform(1, 0, 0, 1, 0, 0);
      ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);
      ctx.restore();
      // Choose visualization based on algorithm
      const isLPA = this.solver && this.solver.constructor && this.solver.constructor.name === 'LPASolver';
      if (isLPA) {
        this._drawLPAOverlay();
      } else {
        this._drawDistanceMap();
      }
      this._drawGoals();
      this._drawGrid();
      this._drawWalls();
      this._drawRobot();
    }

    // Overlay for LPA*: show g/rhs values, open list, inconsistent cells
    _drawLPAOverlay() {
      const { ctx } = this;
      const cellSize = Math.min(this.canvas.clientWidth, this.canvas.clientHeight) / this.maze.size;
      const g = this.solver.gValues;
      const rhs = this.solver.rhsValues;
      // Highlight open list (search frontier)
      if (this.solver.openList && this.solver.openList.items) {
        ctx.save();
        ctx.globalAlpha = 0.18;
        ctx.fillStyle = '#38bdf8';
        for (const item of this.solver.openList.items) {
          const x = item[1], y = item[2];
          ctx.fillRect(x * cellSize, y * cellSize, cellSize, cellSize);
        }
        ctx.restore();
      }
      // Draw g/rhs values and inconsistent cells
      ctx.font = `${Math.max(10, cellSize * 0.28)}px Inter, system-ui`;
      ctx.textAlign = 'center';
      ctx.textBaseline = 'middle';
      for (let y = 0; y < this.maze.size; y++) {
        for (let x = 0; x < this.maze.size; x++) {
          let gx = g[y][x], rx = rhs[y][x];
          let txt = `g:${gx === Infinity ? '∞' : gx}\nr:${rx === Infinity ? '∞' : rx}`;
          // Inconsistent cell highlight
          if (gx !== rx) {
            ctx.save();
            ctx.globalAlpha = 0.18;
            ctx.fillStyle = '#fbbf24';
            ctx.fillRect(x * cellSize, y * cellSize, cellSize, cellSize);
            ctx.restore();
          }
          ctx.fillStyle = '#dbeafe';
          ctx.fillText(txt, x * cellSize + cellSize / 2, y * cellSize + cellSize / 2);
        }
      }
    }
  }
  // Attach MazeUI to window for global access
  global.MazeUI = MazeUI;
})(window);
