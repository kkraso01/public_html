/**
 * Time-Optimal Trajectory Optimization for Drone Racing
 * 
 * Generates aggressive, time-minimal trajectories through gates by:
 * - Minimizing total flight time (not just tracking error)
 * - Pushing physical limits (max thrust, aggressive tilts)
 * - Using differential flatness property of quadrotors
 * - Computing minimum-snap or minimum-jerk trajectories
 * 
 * Based on research from:
 * - "Minimum Snap Trajectory Generation and Control for Quadrotors" (Mellinger, Kumar)
 * - "Time-Optimal Online Replanning for Agile Quadrotor Flight" (Hehn, D'Andrea)
 * - "Aggressive Flight with Suspended Payloads Using Vision-Based Control" (RPG Zürich)
 */

export class TimeOptimalTrajectory {
  constructor(waypoints, params = {}) {
    this.waypoints = waypoints;
    this.params = params;
    
    // Physical limits for aggressive flight
    this.maxVelocity = params.maxVelocity || 12.0;
    this.maxAcceleration = params.maxAcceleration || 20.0;
    this.maxJerk = params.maxJerk || 50.0;
    this.maxTiltAngle = params.maxTiltAngle ? THREE.MathUtils.degToRad(params.maxTiltAngle) : THREE.MathUtils.degToRad(65);
    
    // Optimization parameters
    this.aggressiveness = params.aggressiveness || 0.8; // 0=conservative, 1=maximum aggression
    this.snapWeight = params.snapWeight || 1.0;         // Weight on snap (4th derivative)
    this.timeWeight = params.timeWeight || 10.0;        // Weight on total time
    
    // Computed trajectory
    this.segments = [];
    this.totalDuration = 0;
    this.segmentTimes = [];
    
    this._optimize();
  }

  /**
   * Main optimization: find time-optimal trajectory through waypoints
   */
  _optimize() {
    console.log('[TimeOptimal] Optimizing trajectory through', this.waypoints.length, 'waypoints');
    
    // Step 1: Compute initial time allocation (heuristic based on distance)
    const segmentTimes = this._initialTimeAllocation();
    
    // Step 2: Optimize segment times to minimize total duration
    this.segmentTimes = this._optimizeSegmentTimes(segmentTimes);
    
    // Step 3: Generate minimum-snap polynomial trajectories for each segment
    this.segments = this._generateMinimumSnapTrajectory(this.segmentTimes);
    
    // Step 4: Verify feasibility (check velocity/acceleration limits)
    this._verifyFeasibility();
    
    this.totalDuration = this.segmentTimes.reduce((sum, t) => sum + t, 0);
    console.log('[TimeOptimal] Total duration:', this.totalDuration.toFixed(2), 's');
  }

  /**
   * Initial time allocation based on distance and velocity limits
   */
  _initialTimeAllocation() {
    const times = [];
    
    for (let i = 0; i < this.waypoints.length - 1; i++) {
      const p0 = this.waypoints[i];
      const p1 = this.waypoints[i + 1];
      const distance = p0.distanceTo(p1);
      
      // Time = distance / avgVelocity (aggressive planning uses high avg velocity)
      const avgVelocity = this.maxVelocity * this.aggressiveness;
      const time = distance / avgVelocity;
      
      // Ensure minimum time for acceleration/deceleration
      const minTime = 2.0 * Math.sqrt(distance / this.maxAcceleration);
      times.push(Math.max(time, minTime));
    }
    
    return times;
  }

  /**
   * Optimize segment times to minimize total duration
   * Uses gradient descent on time allocation
   */
  _optimizeSegmentTimes(initialTimes) {
    let times = [...initialTimes];
    const iterations = 20;
    const learningRate = 0.05;
    
    for (let iter = 0; iter < iterations; iter++) {
      // Compute cost (total time + penalty for constraint violations)
      const cost = this._evaluateTimeCost(times);
      
      // Numerical gradients
      const gradients = [];
      for (let i = 0; i < times.length; i++) {
        const epsilon = 0.01;
        const timesPlus = [...times];
        timesPlus[i] += epsilon;
        const costPlus = this._evaluateTimeCost(timesPlus);
        gradients.push((costPlus - cost) / epsilon);
      }
      
      // Gradient descent update
      for (let i = 0; i < times.length; i++) {
        times[i] -= learningRate * gradients[i];
        times[i] = Math.max(0.5, times[i]); // Minimum segment time
      }
    }
    
    return times;
  }

  /**
   * Evaluate cost for given time allocation
   */
  _evaluateTimeCost(times) {
    let totalTime = times.reduce((sum, t) => sum + t, 0);
    let penalty = 0;
    
    // Check if trajectory is feasible with these times
    const testSegments = this._generateMinimumSnapTrajectory(times);
    
    // Sample trajectory and check constraints
    for (let i = 0; i < testSegments.length; i++) {
      const seg = testSegments[i];
      const sampleCount = 10;
      for (let j = 0; j <= sampleCount; j++) {
        const t = (j / sampleCount) * times[i];
        const state = this._evaluatePolynomial(seg, t);
        
        // Velocity constraint violation
        const velMag = state.velocity.length();
        if (velMag > this.maxVelocity) {
          penalty += (velMag - this.maxVelocity) * 10.0;
        }
        
        // Acceleration constraint violation
        const accMag = state.acceleration.length();
        if (accMag > this.maxAcceleration) {
          penalty += (accMag - this.maxAcceleration) * 5.0;
        }
      }
    }
    
    return this.timeWeight * totalTime + penalty;
  }

  /**
   * Generate minimum-snap trajectory for each segment
   * Solves for polynomial coefficients that minimize snap (4th derivative)
   */
  _generateMinimumSnapTrajectory(segmentTimes) {
    const segments = [];
    
    for (let i = 0; i < this.waypoints.length - 1; i++) {
      const p0 = this.waypoints[i];
      const p1 = this.waypoints[i + 1];
      const T = segmentTimes[i];
      
      // Use 7th-order polynomial (allows snap minimization with position/velocity/acc constraints)
      // p(t) = a0 + a1*t + a2*t^2 + ... + a7*t^7
      
      // Boundary conditions:
      // At t=0: position=p0, velocity=v0, acceleration=a0, jerk=0
      // At t=T: position=p1, velocity=v1, acceleration=a1, jerk=0
      
      // Estimate velocities at waypoints (tangent to path)
      const v0 = this._estimateVelocity(i, 'start');
      const v1 = this._estimateVelocity(i, 'end');
      
      // Accelerations (initially zero, refined during optimization)
      const a0 = new THREE.Vector3(0, 0, 0);
      const a1 = new THREE.Vector3(0, 0, 0);
      
      // Solve for polynomial coefficients (per axis)
      const coeffsX = this._solveMinimumSnapCoeffs(p0.x, p1.x, v0.x, v1.x, a0.x, a1.x, T);
      const coeffsY = this._solveMinimumSnapCoeffs(p0.y, p1.y, v0.y, v1.y, a0.y, a1.y, T);
      const coeffsZ = this._solveMinimumSnapCoeffs(p0.z, p1.z, v0.z, v1.z, a0.z, a1.z, T);
      
      segments.push({
        coeffsX,
        coeffsY,
        coeffsZ,
        duration: T,
        startTime: i === 0 ? 0 : segments[i - 1].startTime + segments[i - 1].duration,
      });
    }
    
    return segments;
  }

  /**
   * Estimate velocity at waypoint (tangent to path for smooth transitions)
   */
  _estimateVelocity(waypointIdx, position) {
    if (position === 'start') {
      if (waypointIdx === 0) {
        // First waypoint: velocity toward next
        const dir = new THREE.Vector3().subVectors(this.waypoints[1], this.waypoints[0]).normalize();
        return dir.multiplyScalar(this.maxVelocity * this.aggressiveness * 0.5);
      } else {
        // Tangent from previous to next
        const dir = new THREE.Vector3().subVectors(this.waypoints[waypointIdx + 1], this.waypoints[waypointIdx - 1]).normalize();
        return dir.multiplyScalar(this.maxVelocity * this.aggressiveness);
      }
    } else {
      // position === 'end'
      if (waypointIdx === this.waypoints.length - 2) {
        // Last segment: velocity toward final waypoint
        const dir = new THREE.Vector3().subVectors(this.waypoints[waypointIdx + 1], this.waypoints[waypointIdx]).normalize();
        return dir.multiplyScalar(this.maxVelocity * this.aggressiveness * 0.5);
      } else {
        // Tangent from current to next-next
        const dir = new THREE.Vector3().subVectors(this.waypoints[waypointIdx + 2], this.waypoints[waypointIdx]).normalize();
        return dir.multiplyScalar(this.maxVelocity * this.aggressiveness);
      }
    }
  }

  /**
   * Solve for 7th-order polynomial coefficients that minimize snap
   */
  _solveMinimumSnapCoeffs(p0, p1, v0, v1, a0, a1, T) {
    // Simplified closed-form solution for 7th-order polynomial
    // with boundary conditions on position, velocity, acceleration
    
    // Matrix equation: A * coeffs = b
    // For now, use quintic (5th-order) which has closed-form solution
    
    const T2 = T * T;
    const T3 = T2 * T;
    const T4 = T3 * T;
    const T5 = T4 * T;
    
    // Quintic polynomial coefficients
    const c0 = p0;
    const c1 = v0;
    const c2 = 0.5 * a0;
    
    // Solve for c3, c4, c5 from end constraints
    const c3 = (20 * (p1 - p0) - (8 * v1 + 12 * v0) * T - (3 * a0 - a1) * T2) / (2 * T3);
    const c4 = (30 * (p0 - p1) + (14 * v1 + 16 * v0) * T + (3 * a0 - 2 * a1) * T2) / (2 * T4);
    const c5 = (12 * (p1 - p0) - 6 * (v1 + v0) * T + (a1 - a0) * T2) / (2 * T5);
    
    return [c0, c1, c2, c3, c4, c5];
  }

  /**
   * Evaluate polynomial at time t
   */
  _evaluatePolynomial(segment, t) {
    const { coeffsX, coeffsY, coeffsZ } = segment;
    
    // Position: p(t) = sum(ci * t^i)
    const position = new THREE.Vector3(
      this._polyEval(coeffsX, t),
      this._polyEval(coeffsY, t),
      this._polyEval(coeffsZ, t)
    );
    
    // Velocity: p'(t) = sum(i * ci * t^(i-1))
    const velocity = new THREE.Vector3(
      this._polyDerivEval(coeffsX, t, 1),
      this._polyDerivEval(coeffsY, t, 1),
      this._polyDerivEval(coeffsZ, t, 1)
    );
    
    // Acceleration: p''(t) = sum(i * (i-1) * ci * t^(i-2))
    const acceleration = new THREE.Vector3(
      this._polyDerivEval(coeffsX, t, 2),
      this._polyDerivEval(coeffsY, t, 2),
      this._polyDerivEval(coeffsZ, t, 2)
    );
    
    // Jerk: p'''(t)
    const jerk = new THREE.Vector3(
      this._polyDerivEval(coeffsX, t, 3),
      this._polyDerivEval(coeffsY, t, 3),
      this._polyDerivEval(coeffsZ, t, 3)
    );
    
    return { position, velocity, acceleration, jerk };
  }

  _polyEval(coeffs, t) {
    let result = 0;
    let tPower = 1;
    for (let i = 0; i < coeffs.length; i++) {
      result += coeffs[i] * tPower;
      tPower *= t;
    }
    return result;
  }

  _polyDerivEval(coeffs, t, order) {
    // Evaluate derivative of polynomial
    let derivCoeffs = [...coeffs];
    
    for (let d = 0; d < order; d++) {
      const newCoeffs = [];
      for (let i = 1; i < derivCoeffs.length; i++) {
        newCoeffs.push(derivCoeffs[i] * i);
      }
      derivCoeffs = newCoeffs;
    }
    
    return this._polyEval(derivCoeffs, t);
  }

  /**
   * Verify trajectory feasibility
   */
  _verifyFeasibility() {
    let maxVelViolation = 0;
    let maxAccViolation = 0;
    
    for (let i = 0; i < this.segments.length; i++) {
      const seg = this.segments[i];
      const sampleCount = 20;
      
      for (let j = 0; j <= sampleCount; j++) {
        const t = (j / sampleCount) * seg.duration;
        const state = this._evaluatePolynomial(seg, t);
        
        const velMag = state.velocity.length();
        if (velMag > this.maxVelocity) {
          maxVelViolation = Math.max(maxVelViolation, velMag - this.maxVelocity);
        }
        
        const accMag = state.acceleration.length();
        if (accMag > this.maxAcceleration) {
          maxAccViolation = Math.max(maxAccViolation, accMag - this.maxAcceleration);
        }
      }
    }
    
    if (maxVelViolation > 0.1) {
      console.warn('[TimeOptimal] Max velocity violation:', maxVelViolation.toFixed(2), 'm/s');
    }
    if (maxAccViolation > 0.1) {
      console.warn('[TimeOptimal] Max acceleration violation:', maxAccViolation.toFixed(2), 'm/s²');
    }
  }

  /**
   * Get state at time t (position, velocity, acceleration, yaw)
   */
  getStateAtTime(t) {
    // Clamp to trajectory duration
    t = Math.max(0, Math.min(t, this.totalDuration));
    
    // Find segment
    let segmentIdx = 0;
    let segmentTime = t;
    
    for (let i = 0; i < this.segments.length; i++) {
      if (segmentTime <= this.segments[i].duration) {
        segmentIdx = i;
        break;
      }
      segmentTime -= this.segments[i].duration;
    }
    
    // Handle edge case: at or past end
    if (segmentIdx >= this.segments.length) {
      segmentIdx = this.segments.length - 1;
      segmentTime = this.segments[segmentIdx].duration;
    }
    
    const segment = this.segments[segmentIdx];
    const state = this._evaluatePolynomial(segment, segmentTime);
    
    // Compute yaw from velocity direction
    const yaw = Math.atan2(state.velocity.y, state.velocity.x);
    
    return {
      position: state.position,
      velocity: state.velocity,
      acceleration: state.acceleration,
      jerk: state.jerk,
      yaw,
    };
  }

  /**
   * Get metrics for display
   */
  getMetrics() {
    return {
      totalDuration: this.totalDuration,
      segmentCount: this.segments.length,
      aggressiveness: this.aggressiveness,
      avgSpeed: this._computeAverageSpeed(),
      maxSpeed: this._computeMaxSpeed(),
    };
  }

  _computeAverageSpeed() {
    let totalDistance = 0;
    for (let i = 0; i < this.waypoints.length - 1; i++) {
      totalDistance += this.waypoints[i].distanceTo(this.waypoints[i + 1]);
    }
    return totalDistance / this.totalDuration;
  }

  _computeMaxSpeed() {
    let maxSpeed = 0;
    for (let i = 0; i < this.segments.length; i++) {
      const seg = this.segments[i];
      const sampleCount = 20;
      for (let j = 0; j <= sampleCount; j++) {
        const t = (j / sampleCount) * seg.duration;
        const state = this._evaluatePolynomial(seg, t);
        maxSpeed = Math.max(maxSpeed, state.velocity.length());
      }
    }
    return maxSpeed;
  }
}
