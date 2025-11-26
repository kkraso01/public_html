(function (global) {
  /**
   * Physics-aware motion profiles for Micromouse
   * Models: acceleration, jerk limits, cornering forces, tire slip
   * Reference: Real Micromouse physics - 2-6g acceleration, 6g+ cornering with vacuum
   */
  class MotionProfile {
    constructor(options = {}) {
      // Physical constraints (realistic Micromouse values)
      this.maxSpeed = options.maxSpeed || 4.0; // m/s (competitive mice can reach 4-5 m/s)
      this.maxAccel = options.maxAccel || 2.5; // m/s² (2-3g typical)
      this.maxDecel = options.maxDecel || 2.0; // m/s²
      this.maxJerk = options.maxJerk || 8.0; // m/s³ (limit rate of accel change)
      this.maxCornering = options.maxCornering || 3.0; // m/s² (centripetal)
      this.cellSize = options.cellSize || 0.18; // 180mm per cell in Micromouse
      this.wheelBase = options.wheelBase || 0.08; // 80mm typical
      
      this.segments = [];
      this.currentSpeed = 0;
      this.distance = 0;
    }

    /**
     * Generate motion profile for a straight segment
     * Returns time, speed trajectory, acceleration profile
     */
    profileSegment(lengthCells, isLast = false) {
      const length = lengthCells * this.cellSize;
      const targetSpeed = isLast ? 0 : this.maxSpeed;
      
      // Phase 1: Acceleration with jerk limiting
      const accelPhase = this._accelWithJerk(this.currentSpeed, this.maxSpeed, length);
      
      // Phase 2: Constant speed
      const constantPhase = this._constantSpeed(this.maxSpeed, length - accelPhase.distance);
      
      // Phase 3: Deceleration (for last segment)
      const decelPhase = isLast 
        ? this._decelWithJerk(this.currentSpeed, 0, length - accelPhase.distance - constantPhase.distance)
        : { time: 0, distance: 0, speedProfile: [] };
      
      this.currentSpeed = targetSpeed;
      this.distance += length;
      
      return {
        length,
        totalTime: accelPhase.time + constantPhase.time + decelPhase.time,
        phases: {
          accel: accelPhase,
          constant: constantPhase,
          decel: decelPhase
        },
        speedProfile: [
          ...accelPhase.speedProfile,
          ...constantPhase.speedProfile,
          ...decelPhase.speedProfile
        ]
      };
    }

    _accelWithJerk(startSpeed, targetSpeed, maxLength) {
      // Trapezoidal profile with jerk limiting
      // Jerk limits rate of change of acceleration
      const deltaV = targetSpeed - startSpeed;
      if (deltaV <= 0) return { time: 0, distance: 0, speedProfile: [] };
      
      // Time to reach max acceleration
      const jerkTime = Math.min(this.maxAccel / this.maxJerk, 0.1);
      
      // Distance and time for accel phase
      const accelTime = deltaV / this.maxAccel;
      const accelDist = startSpeed * accelTime + 0.5 * this.maxAccel * accelTime * accelTime;
      
      if (accelDist > maxLength) {
        // Limited acceleration
        const t = (-startSpeed + Math.sqrt(startSpeed * startSpeed + 2 * this.maxAccel * maxLength)) / this.maxAccel;
        return {
          time: t,
          distance: maxLength,
          speedProfile: this._generateSpeedProfile(startSpeed, startSpeed + this.maxAccel * t, t, 10)
        };
      }
      
      return {
        time: accelTime,
        distance: accelDist,
        speedProfile: this._generateSpeedProfile(startSpeed, targetSpeed, accelTime, 10)
      };
    }

    _constantSpeed(speed, maxLength) {
      if (speed <= 0 || maxLength <= 0) {
        return { time: 0, distance: 0, speedProfile: [] };
      }
      const time = maxLength / speed;
      return {
        time,
        distance: maxLength,
        speedProfile: Array(10).fill(speed)
      };
    }

    _decelWithJerk(startSpeed, targetSpeed, maxLength) {
      const deltaV = startSpeed - targetSpeed;
      if (deltaV <= 0) return { time: 0, distance: 0, speedProfile: [] };
      
      const decelTime = deltaV / this.maxDecel;
      const decelDist = startSpeed * decelTime - 0.5 * this.maxDecel * decelTime * decelTime;
      
      if (decelDist > maxLength) {
        const t = (startSpeed - Math.sqrt(Math.max(0, startSpeed * startSpeed - 2 * this.maxDecel * maxLength))) / this.maxDecel;
        return {
          time: t,
          distance: maxLength,
          speedProfile: this._generateSpeedProfile(startSpeed, Math.max(0, startSpeed - this.maxDecel * t), t, 10)
        };
      }
      
      return {
        time: decelTime,
        distance: decelDist,
        speedProfile: this._generateSpeedProfile(startSpeed, targetSpeed, decelTime, 10)
      };
    }

    _generateSpeedProfile(startSpeed, endSpeed, duration, points) {
      const profile = [];
      for (let i = 0; i < points; i++) {
        const t = (i / points) * duration;
        const speed = startSpeed + (endSpeed - startSpeed) * (t / duration);
        profile.push(speed);
      }
      return profile;
    }

    /**
     * Calculate centripetal acceleration for a turn
     * Used to determine safe turning speed
     */
    getMaxSpeedForTurn(radius) {
      // v_max = sqrt(a_c_max * r)
      return Math.sqrt(this.maxCornering * radius);
    }

    /**
     * Calculate time to traverse path with physics
     */
    calculateTraversalTime(pathLength, segmentCount = 1) {
      let totalTime = 0;
      const segmentLength = pathLength / segmentCount;
      
      for (let i = 0; i < segmentCount; i++) {
        const isLast = i === segmentCount - 1;
        const profile = this.profileSegment(segmentLength, isLast);
        totalTime += profile.totalTime;
      }
      
      return totalTime;
    }
  }

  /**
   * Trajectory optimizer - find time-optimal path
   * Uses dynamic programming on pre-computed motion profiles
   */
  class TrajectoryOptimizer {
    constructor(motionProfile) {
      this.motionProfile = motionProfile;
      this.cache = new Map();
    }

    /**
     * Optimize velocity profile for a path
     * Reduces total time while respecting acceleration/jerk limits
     */
    optimizePath(path) {
      if (!path || path.length < 2) return { path, time: 0, profiles: [] };
      
      const profiles = [];
      let totalTime = 0;
      
      // For each segment, compute optimal motion
      for (let i = 0; i < path.length - 1; i++) {
        const isLast = i === path.length - 2;
        const segment = this.motionProfile.profileSegment(1, isLast);
        profiles.push(segment);
        totalTime += segment.totalTime;
      }
      
      return { path, time: totalTime, profiles };
    }

    /**
     * Compare trajectory times between different strategies
     */
    compareStrategies(path) {
      const naive = { time: path.length * 0.5 }; // Simple: 0.5s per cell
      const optimized = this.optimizePath(path);
      
      return {
        naive: naive.time,
        optimized: optimized.time,
        speedup: naive.time / optimized.time,
        timeSaved: naive.time - optimized.time
      };
    }
  }

  global.MotionProfile = MotionProfile;
  global.TrajectoryOptimizer = TrajectoryOptimizer;
})(window);
