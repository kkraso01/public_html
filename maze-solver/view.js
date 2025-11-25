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
      this.renderer = new MazeRenderer(canvas, this.maze, this.solver);
      this.interval = null;
      this._bindControls();
      this.solver.computeDistances();
      this.renderer.draw();
    }

    _bindControls() {
      const generateBtn = document.getElementById("maze-generate");
      const runBtn = document.getElementById("maze-run");
      const stepBtn = document.getElementById("maze-step");
      const resetBtn = document.getElementById("maze-reset");

      generateBtn?.addEventListener("click", () => {
        this.stop();
        this.maze.generate();
        this.solver = new global.FloodSolver(this.maze);
        this.renderer.dispose();
        this.renderer = new MazeRenderer(this.canvas, this.maze, this.solver);
        this.solver.computeDistances();
        this.renderer.draw();
      });

      resetBtn?.addEventListener("click", () => {
        this.stop();
        this.solver.reset();
        this.solver.computeDistances();
        this.renderer.draw();
      });

      stepBtn?.addEventListener("click", () => this._stepOnce());

      runBtn?.addEventListener("click", () => {
        if (this.interval) {
          this.stop();
          return;
        }
        runBtn.textContent = "Pause";
        this.interval = setInterval(() => {
          const result = this._stepOnce();
          if (result?.done) this.stop();
        }, 350);
      });
    }

    _stepOnce() {
      const result = this.solver.step();
      this.renderer.draw();
      return result;
    }

    stop() {
      const runBtn = document.getElementById("maze-run");
      if (this.interval) {
        clearInterval(this.interval);
        this.interval = null;
      }
      if (runBtn) runBtn.textContent = "Run";
    }
  }

  document.addEventListener("DOMContentLoaded", () => {
    new MazeUI("maze-canvas");
  });
})(window);
