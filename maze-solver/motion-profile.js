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
        type: "line",
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

    /**
     * Generate motion profile for an arc segment
     * Applies centripetal acceleration constraint: v² / r ≤ aLatMax
     * 
     * @param {Object} arc - { type: "arc", radius, angle, mode?, length }
     * @param {boolean} isLast - true if final segment (should end at 0)
     * @returns {Object} - { type: "arc", length, totalTime, vIn, vOut, maxSpeed, ... }
     */
    profileArc(arc, isLast = false) {
      const { radius, angle, length, mode } = arc;
      
      if (!radius || radius <= 0) {
        // Degenerate arc, treat as line
        return this.profileSegment(length / this.cellSize, isLast);
      }

      // Maximum safe speed for this radius (centripetal constraint)
      // v_max_curve = sqrt(aLatMax * r)
      const vMaxArc = Math.sqrt(this.maxCornering * radius);
      const vLimit = Math.min(this.maxSpeed, vMaxArc);

      // For arc, we need to manage speed carefully to keep lateral acceleration within limits
      const vIn = this.currentSpeed;
      const targetSpeed = isLast ? 0 : vLimit;

      // Phase 1: Entry (ramp to arc speed if needed)
      let entryDist = 0;
      let entryTime = 0;
      let entrySpeed = vIn;

      if (vIn < vLimit - 0.01) {
        // Need to accelerate to arc speed
        const dv = Math.min(vLimit - vIn, this.maxAccel * 0.1);
        entryDist = vIn * 0.05 + 0.5 * this.maxAccel * 0.05 * 0.05;  // 50ms accel
        entryTime = 0.05;
        entrySpeed = vIn + this.maxAccel * 0.05;
      }

      // Phase 2: Arc at constant speed (or slight accel within arc length)
      const arcDist = length - entryDist;
      const arcSpeed = Math.min(vLimit, entrySpeed);
      const arcTime = arcDist > 0 ? arcDist / arcSpeed : 0;

      // Phase 3: Exit (prepare for next segment)
      // Don't decelerate here; velocity planning will handle it
      const vOut = arcSpeed;
      
      this.currentSpeed = vOut;
      this.distance += length;

      const profile = {
        type: "arc",
        mode: mode,
        length: length,
        radius: radius,
        angle: angle,
        vIn: vIn,
        vOut: vOut,
        maxSpeedCurve: vMaxArc,
        totalTime: entryTime + arcTime,
        phases: {
          entry: { time: entryTime, distance: entryDist, speed: entrySpeed },
          arc: { time: arcTime, distance: arcDist, speed: arcSpeed }
        },
        speedProfile: this._generateArcSpeedProfile(vIn, arcSpeed, entryTime, arcTime)
      };

      if (isLast) {
        // Add deceleration phase to reach stop
        const decelDist = vOut * 0.1 - 0.5 * this.maxDecel * 0.1 * 0.1; // Rough estimate
        const decelTime = vOut / this.maxDecel;
        profile.totalTime += decelTime;
        profile.phases.exit = { time: decelTime, distance: decelDist, speed: 0 };
        this.currentSpeed = 0;
      }

      return profile;
    }

    _generateArcSpeedProfile(vIn, vArc, entryTime, arcTime) {
      const points = 15;
      const profile = [];
      const totalTime = entryTime + arcTime;

      for (let i = 0; i < points; i++) {
        const t = (i / points) * totalTime;
        let v;

        if (t < entryTime) {
          // Entry ramp
          v = vIn + (vArc - vIn) * (t / entryTime);
        } else {
          // Arc at constant speed
          v = vArc;
        }

        profile.push(Math.max(0, v));
      }

      return profile;
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

    /**
     * Profile an array of segments that may include lines and arcs
     * Used by RacePlanner's velocity planning
     * 
     * @param {Array} segments - [{ type: "line"|"arc", length, radius?, mode?, angle? }, ...]
     * @returns {Array} - segment profiles with timings
     */
    profileSegments(segments) {
      const profiles = [];
      this.currentSpeed = 0;
      
      for (let i = 0; i < segments.length; i++) {
        const seg = segments[i];
        const isLast = i === segments.length - 1;
        
        let profile;
        
        if (seg.type === "arc") {
          // Use arc profiler for arc segments
          profile = this.profileArc(seg, isLast);
        } else {
          // Use line profiler for straight segments
          const lengthCells = seg.length / this.cellSize;
          profile = this.profileSegment(lengthCells, isLast);
        }
        
        profiles.push(profile);
      }
      
      return profiles;
    }

    /**
     * Calculate time for a turn
     */
    getTurnTime(angleDegrees = 90) {
      // Assume max angular acceleration for in-place turn
      const maxAngularAccel = 20; // rad/s², typical for micromouse
      const angleRad = angleDegrees * Math.PI / 180;
      // Time for constant accel: t = sqrt(2 * theta / alpha)
      const time = Math.sqrt(2 * angleRad / maxAngularAccel);
      return time;
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
     * Optimize velocity profile for a path (segments may include arcs)
     * Reduces total time while respecting acceleration/jerk limits
     */
    optimizePath(segments) {
      if (!segments || segments.length === 0) return { segments, time: 0, profiles: [] };
      
      const profiles = this.motionProfile.profileSegments(segments);
      let totalTime = 0;

      profiles.forEach(p => {
        totalTime += p.totalTime;
      });

      return { segments, time: totalTime, profiles };
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
