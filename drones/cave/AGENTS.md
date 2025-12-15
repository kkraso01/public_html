# AI Agent Instructions — Autonomous Cave Exploration

## Overview
Fully autonomous 3D cave exploration demo featuring **SLAM**, **frontier-based exploration**, **next-best-view planning**, and **real-time 3D reconstruction**. The drone navigates unknown cave environments using only lidar and IMU.

## Purpose
- **Autonomous Exploration**: No human input after launch
- **SLAM Pipeline**: ICP-based odometry + particle filter localization
- **Mapping**: Occupancy grid (frontier detection) + TSDF volume (dense reconstruction)
- **Planning**: NBV (Next-Best-View) selection + A* path planning

## Architecture

### System Pipeline
```
┌─────────────┐
│   Sensors   │ → Lidar scans (360° × 180°, 3m range)
└──────┬──────┘   IMU (quaternion, angular velocity)
       │
       ▼
┌─────────────┐
│    SLAM     │ → ICP Odometry (scan-to-scan matching)
│             │   Particle Filter (pose estimation)
└──────┬──────┘
       │
       ▼
┌─────────────┐
│   Mapping   │ → OccupancyGrid3D (voxels: free/occupied/unknown)
│             │   TSDF Volume (signed distance field)
└──────┬──────┘   Frontier extraction (unknown/free boundaries)
       │
       ▼
┌─────────────┐
│  Planning   │ → NBV: select best exploration target
│             │   A* pathfinding (3D grid-based)
└──────┬──────┘   Path smoothing & yaw assignment
       │
       ▼
┌─────────────┐
│  Control    │ → ETH SE(3) controller (nose-first mode)
│             │   Altitude safety constraints
└─────────────┘   Motor allocation
```

## Files and Components

### Main Demo
**`drone_cave_demo.js`** (1168 lines)
- **Lines 1-100**: Initialization, scene setup, performance flags
- **Lines 100-300**: Physics integration, collision handling
- **Lines 300-500**: SLAM update (ICP, particle filter)
- **Lines 500-700**: Frontier extraction, NBV planning
- **Lines 700-900**: Path following, control loop
- **Lines 900-1100**: Rendering (cave, drone, lidar, map)
- **Lines 1100-1168**: Utilities, cleanup

**Key Features**:
- Chase camera (follows drone through cave)
- Real-time lidar visualization (green rays)
- Occupancy grid overlay (transparent voxels)
- Trajectory trail (shows exploration path)
- Performance mode (toggle shadows, particle filter, TSDF)

### Cave Environment
**`cave_sim.js`**
- Procedural cave generation (Perlin noise-based)
- Stalactites, stalagmites, rock formations
- Adjustable complexity (tunnel width, curvature)
- Collision mesh for physics

### Sensors
**`lidar.js`**
- **Range**: 3.0 meters (Crazyflie-scale)
- **Resolution**: 16 beams horizontal × 8 vertical (128 rays total)
- **Update Rate**: 10 Hz (realistic for lightweight lidar)
- **Noise Model**: Gaussian (σ = 0.02m) + occasional misses
- **Output**: `{points: Vector3[], ranges: float[]}`

**Performance Mode**: Reduce to 8×4 beams (32 rays)

### SLAM Components

#### `icp_odometry.js` — Scan Matching
**Algorithm**: Iterative Closest Point (ICP)
```
Input:  previous_scan, current_scan
Output: relative_transform (Δx, Δy, Δz, Δyaw)

1. Match each point in current to nearest in previous
2. Compute centroid-based rotation (SVD)
3. Refine translation
4. Iterate until convergence (10 iterations max)
```

**Limitations**:
- Only works in planar environments (caves are mostly horizontal)
- Drifts over time (corrected by particle filter)
- Sensitive to scan density (need ≥50 points per scan)

#### `mapping.js` — Multi-Purpose Mapper

**OccupancyGrid3D**:
- **Resolution**: 0.25m voxels (adjustable)
- **Size**: 40×40×15 meters (16,000 voxels)
- **States**: FREE (0), OCCUPIED (1), UNKNOWN (2)
- **Update**: Bresenham ray tracing (mark free along ray, occupied at endpoint)

**Frontier Detection**:
```javascript
// Frontiers = voxels where:
isFrontier(voxel) = (voxel == FREE) && 
                    anyNeighbor(voxel) == UNKNOWN
```
Used to find unexplored regions.

**Particle Filter Localization**:
- **Particles**: 50-100 pose hypotheses
- **Prediction**: Motion model (velocity + noise)
- **Update**: Likelihood based on scan match to map
- **Resampling**: Low-variance resampler every 10 steps

**Path Planning (A\*)**:
- **Graph**: OccupancyGrid3D (FREE voxels only)
- **Heuristic**: Euclidean distance
- **Neighbors**: 26-connected (3D)
- **Path Smoothing**: Waypoint reduction (skip intermediate points if line-of-sight clear)

#### `tsdf_volume.js` — Dense Reconstruction
**Algorithm**: Truncated Signed Distance Function
- **Resolution**: 0.1m (finer than occupancy grid)
- **Truncation**: ±0.3m (ignore far surfaces)
- **Fusion**: Weighted average of depth observations
- **Marching Cubes**: Extract mesh for visualization (optional)

**Trade-off**:
- High quality 3D models
- Computationally expensive (disabled in performance mode)

### Planning

#### `nbv.js` — Next-Best-View Selection
**Algorithm**: Score-based frontier ranking
```javascript
score(frontier) = 
  information_gain(frontier) * 0.5 +
  (1 / distance(drone, frontier)) * 0.3 +
  alignment_with_current_heading * 0.2
```

**Information Gain**:
- Count unknown voxels visible from frontier
- Simulate lidar scan at frontier position
- Higher gain = more unexplored space

**Distance Penalty**:
- Prefer nearby frontiers
- Avoid long detours

**Heading Alignment**:
- Prefer frontiers in current direction
- Reduce oscillations

**Fallback**:
If no frontiers (exploration complete):
- Return to start position
- Or continue deepest penetration

### Control Integration

**ETH Controller (Nose-First Mode)**:
```javascript
controller.computeNoseFirstControl(state, waypoint, dt, {
  enableNoseFacing: true,
  aggressiveYawTracking: true,
  maxYawRate: 20.0 // rad/s (reduced from 100 to prevent jitter)
});
```

**Altitude Safety**:
```javascript
const minSafeAltitude = floorHeight + 0.6; // 0.65m default
if (state.position.z < minSafeAltitude) {
  state.velocity.z = Math.max(0, state.velocity.z);
  state.position.z = minSafeAltitude;
}
```

**Path Following**:
- **Waypoint Tolerance**: 0.3m (switch to next waypoint)
- **Lookahead**: 1 waypoint (no trajectory preview)
- **Velocity Profile**: Constant 0.5 m/s (conservative for safety)

## Performance Optimization

### Flags (Toggle via Console)
```javascript
window.caveDemo.enableParticleFilter = false; // Disable PF (use ICP only)
window.caveDemo.enableTSDF = false;          // Disable dense reconstruction
window.caveDemo.enableShadows = false;       // Disable shadows
window.caveDemo.lidarDensity = 'low';        // 8×4 beams instead of 16×8
```

### Performance Modes
```javascript
// High-End (default)
physicsRate: 200 Hz
lidarRate: 10 Hz
particles: 100
TSDF: enabled
shadows: enabled

// Performance Mode
physicsRate: 120 Hz
lidarRate: 5 Hz
particles: 50
TSDF: disabled
shadows: disabled
```

### Bottlenecks
1. **TSDF Update**: Most expensive (marching cubes)
   - **Fix**: Update only every 2 seconds
2. **Particle Filter**: Scan likelihood for 100 particles
   - **Fix**: Reduce to 50 particles or disable
3. **Lidar Ray Casting**: 128 rays × 10 Hz = 1280 rays/sec
   - **Fix**: Reduce to 32 rays (8×4)
4. **A\* Pathfinding**: Large grid (16k voxels)
   - **Fix**: Limit search depth (10m max)

## Known Issues & Solutions

### Issue #1: Drone Gets Stuck in Corners
**Symptom**: Oscillates between two frontiers without making progress  
**Cause**: Frontier selection oscillates when scores are close  
**Fix**: Add hysteresis (prefer current target unless new one is 20% better)
```javascript
if (newScore > currentScore * 1.2) {
  switchTarget();
}
```

### Issue #2: Altitude Keeps Increasing
**Symptom**: Drone slowly climbs to ceiling  
**Cause**: NBV selects high frontiers (more visible area)  
**Fix**: Add altitude penalty in NBV scoring:
```javascript
score -= Math.abs(frontier.z - 2.0) * 0.1; // Prefer 2m altitude
```

### Issue #3: ICP Drift in Long Tunnels
**Symptom**: Estimated position drifts from true position  
**Cause**: ICP has no loop closure, accumulates error  
**Fix**: Use particle filter (already implemented), or add visual odometry

### Issue #4: Path Cuts Through Walls
**Symptom**: A\* path goes through thin obstacles  
**Cause**: Occupancy grid resolution too coarse (0.25m)  
**Fix**: Inflate obstacles by 1 voxel:
```javascript
for (each occupied voxel) {
  mark_neighbors_as_occupied();
}
```

### Issue #5: Lidar Misses Thin Stalactites
**Symptom**: Drone collides with objects not in map  
**Cause**: 128 beams too sparse for small features  
**Fix**: Increase beam density (trade-off with performance)

## Common Tasks for AI Agents

### Easy
- Adjust NBV scoring weights
- Change lidar range/resolution
- Modify waypoint tolerance
- Add telemetry HUD

### Medium
- Implement loop closure detection
- Add visual odometry (camera simulation)
- Optimize A\* with hierarchical planning
- Create different cave presets

### Hard
- Multi-drone cooperative exploration
- Semantic mapping (classify rock types)
- Learning-based NBV (neural network)
- Real-time mesh simplification

## Testing Checklist

- [ ] Drone explores at least 50% of cave volume
- [ ] No collisions with walls/ceiling/floor
- [ ] Occupancy grid matches cave geometry
- [ ] Frontiers appear at exploration boundary
- [ ] A\* paths are collision-free
- [ ] ICP odometry drift <10% of distance traveled
- [ ] Particle filter converges within 5 seconds
- [ ] TSDF reconstruction is smooth (if enabled)
- [ ] Runs at 30+ FPS on target hardware

## Debugging Tips

### Enable Debug Logging
Uncomment in `drone_cave_demo.js`:
```javascript
// Line 691: Control target logging
console.log(`[CTRL_TARGET] ...`);

// Line 567: Frontier count
console.log(`[NBV] Found ${frontiers.length} frontiers`);

// Line 489: ICP odometry
console.log(`[ICP] Δx=${dx} Δy=${dy} Δyaw=${dyaw}`);
```

### Visualize Internal State
```javascript
// In browser console:
window.caveDemo.debugEnabled = true; // Enable all debug visuals
window.caveDemo.showOccupancyGrid = true; // Show voxel grid
window.caveDemo.showParticles = true; // Show particle filter
window.caveDemo.showFrontiers = true; // Highlight frontiers
```

### Monitor Exploration Progress
```javascript
const exploredVoxels = grid.countVoxels(FREE);
const totalVoxels = grid.width * grid.height * grid.depth;
const explorationPercentage = exploredVoxels / totalVoxels * 100;
console.log(`Explored: ${explorationPercentage.toFixed(1)}%`);
```

## Physics Integration Notes

### Coordinate System
- **Cave**: Generated in +Z up frame
- **Lidar**: Returns points in world frame (already transformed)
- **Controller**: Uses +Z up (ETH convention)
- **Collision**: Simple Z-clamp at floor height

### Sensor Noise
```javascript
// Lidar range noise
const noisyRange = trueRange + gaussian(0, 0.02); // σ = 2cm

// IMU orientation (already noisy in physics engine)
// No additional noise added
```

### Update Rates
```javascript
Physics:  200 Hz (5ms)
Lidar:    10 Hz  (100ms) ← sensor bottleneck
SLAM:     10 Hz  (triggered by lidar)
Planning: 1 Hz   (1000ms) ← deliberative
Control:  200 Hz (same as physics)
```

## Dependencies

- `../physics/` — Dynamics, motor model
- `../stationary/eth_controller.js` — SE(3) controller
- `Three.js` — Rendering, vector math
- No external SLAM libraries — all algorithms implemented from scratch

## References

- **ICP**: Besl & McKay, "A Method for Registration of 3-D Shapes" (1992)
- **Frontier Exploration**: Yamauchi, "A Frontier-Based Approach for Autonomous Exploration" (1997)
- **TSDF**: Curless & Levoy, "A Volumetric Method for Building Complex Models" (1996)
- **NBV Planning**: Connolly, "The Application of Harmonic Functions to Robotics" (1990)
- **Particle Filter**: Thrun et al., "Probabilistic Robotics" (2005)

---

**Last Updated**: Dec 2025  
**Maintainer**: Konstantin Krasovitskiy  
**Status**: Production (actively developed)
