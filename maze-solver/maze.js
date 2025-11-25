(function (global) {
  const DIRS = ["north", "east", "south", "west"];
  const DELTAS = {
    north: { x: 0, y: -1 },
    east: { x: 1, y: 0 },
    south: { x: 0, y: 1 },
    west: { x: -1, y: 0 }
  };

  class Maze {
    constructor(size = 16) {
      this.size = size;
      this.goalCells = this._buildGoalCells();
      this.generate();
    }

    _buildGoalCells() {
      const c = Math.floor(this.size / 2);
      return new Set([
        `${c - 1},${c - 1}`,
        `${c},${c - 1}`,
        `${c - 1},${c}`,
        `${c},${c}`
      ]);
    }

    _emptyGrid() {
      const grid = [];
      for (let y = 0; y < this.size; y += 1) {
        const row = [];
        for (let x = 0; x < this.size; x += 1) {
          row.push({ north: true, east: true, south: true, west: true });
        }
        grid.push(row);
      }
      return grid;
    }

    _applyBoundaries(grid) {
      for (let i = 0; i < this.size; i += 1) {
        grid[0][i].north = true;
        grid[this.size - 1][i].south = true;
        grid[i][0].west = true;
        grid[i][this.size - 1].east = true;
      }
    }

    _carve(grid, x, y, dir) {
      const nx = x + DELTAS[dir].x;
      const ny = y + DELTAS[dir].y;
      if (nx < 0 || ny < 0 || nx >= this.size || ny >= this.size) return;
      grid[y][x][dir] = false;
      const opposite = DIRS[(DIRS.indexOf(dir) + 2) % 4];
      grid[ny][nx][opposite] = false;
    }

    _shuffle(arr) {
      for (let i = arr.length - 1; i > 0; i -= 1) {
        const j = Math.floor(Math.random() * (i + 1));
        [arr[i], arr[j]] = [arr[j], arr[i]];
      }
      return arr;
    }

    _ensureReachable(grid) {
      // BFS to confirm any goal cell reachable from start (0,0)
      const q = [[0, 0]];
      const seen = new Set(["0,0"]);
      while (q.length) {
        const [x, y] = q.shift();
        if (this.goalCells.has(`${x},${y}`)) return true;
        for (const dir of DIRS) {
          if (grid[y][x][dir]) continue;
          const nx = x + DELTAS[dir].x;
          const ny = y + DELTAS[dir].y;
          if (nx < 0 || ny < 0 || nx >= this.size || ny >= this.size) continue;
          const key = `${nx},${ny}`;
          if (!seen.has(key)) {
            seen.add(key);
            q.push([nx, ny]);
          }
        }
      }
      return false;
    }

    generate() {
      let attempts = 0;
      while (true) {
        attempts += 1;
        const grid = this._emptyGrid();
        this._applyBoundaries(grid);

        // Depth-first backtracking to create a spanning tree (guarantees connectivity)
        const visited = Array.from({ length: this.size }, () => Array(this.size).fill(false));
        const stack = [[0, 0]];
        visited[0][0] = true;

        while (stack.length) {
          const [cx, cy] = stack[stack.length - 1];
          const neighbors = [];
          for (const dir of DIRS) {
            const nx = cx + DELTAS[dir].x;
            const ny = cy + DELTAS[dir].y;
            if (nx < 0 || ny < 0 || nx >= this.size || ny >= this.size) continue;
            if (!visited[ny][nx]) neighbors.push({ dir, nx, ny });
          }

          if (neighbors.length === 0) {
            stack.pop();
            continue;
          }

          const choice = this._shuffle(neighbors)[0];
          this._carve(grid, cx, cy, choice.dir);
          visited[choice.ny][choice.nx] = true;
          stack.push([choice.nx, choice.ny]);
        }

        // Add extra passages to create loops & reduce long corridors
        for (let y = 0; y < this.size; y += 1) {
          for (let x = 0; x < this.size; x += 1) {
            if (Math.random() < 0.2) {
              const options = [];
              if (x < this.size - 1) options.push("east");
              if (y < this.size - 1) options.push("south");
              if (options.length) {
                const dir = options[Math.floor(Math.random() * options.length)];
                this._carve(grid, x, y, dir);
              }
            }
          }
        }

        // Validate solvability (defensive, spanning tree should guarantee)
        if (this._ensureReachable(grid)) {
          this.cells = grid;
          break;
        }

        if (attempts > 5) {
          // Give up on retries; accept the current grid
          this.cells = grid;
          break;
        }
      }
    }

    hasWall(x, y, dir) {
      return this.cells?.[y]?.[x]?.[dir] ?? true;
    }
  }

  global.Maze = Maze;
  global.MAZE_DIRS = DIRS;
  global.MAZE_DELTAS = DELTAS;
})(window);
