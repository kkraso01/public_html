# RacePlanner Integration Fix

## Issue
**Error:** `Uncaught TypeError: Cannot read properties of undefined (reading 'length')` at `view.js:464`

**Root Cause:** Inconsistent return format between RacePlanner and view.js expectations:
- RacePlanner returned: `{ cells, segments, profiles, totalTime }`
- view.js expected: `{ gridPath, segments, totalTime }`
- Code tried to access `this.cachedRacePlan.gridPath.length` which was undefined

## Solution
Updated the return format of `RacePlanner.computeRacePlan()` to match system conventions.

### Changes Made

#### 1. path-planner-oblique.js (line ~467)
Changed `computeRacePlan()` return object:
```javascript
// OLD:
return {
  cells,
  segments: raceSegments,
  profiles: segmentProfiles,
  totalTime
};

// NEW:
return {
  gridPath: cells,          // Changed from 'cells' to 'gridPath' for consistency
  segments: raceSegments,
  profiles: segmentProfiles,
  totalTime
};
```

#### 2. view.js (line ~389)
Updated logging to use consistent property name:
```javascript
// OLD:
`${this.cachedRacePlan.cells.length} cells, ` +

// NEW:
`${this.cachedRacePlan.gridPath.length} cells, ` +
```

## Integration Points Fixed
The RacePlanner now correctly integrates with:

1. **view.js line 461-464** - Sync explorer path:
   ```javascript
   if (this.explorer.discreteOptimalPath !== this.cachedRacePlan.gridPath) {
     this.explorer.discreteOptimalPath = this.cachedRacePlan.gridPath;
   }
   ```

2. **view.js line 468** - Pass path to renderer:
   ```javascript
   this.renderer.setRacingPathInfo(
     this.cachedRacePlan.gridPath,    // ← Now properly defined
     this.explorer.racingPathIndex,
     this.explorer.phase,
     plannerMode
   );
   ```

3. **Consistency with fallback planners:**
   - 4D PathPlanner: `{ gridPath, segments, totalTime }`
   - Legacy ObliquePathOptimizer: `{ gridPath, segments, totalTime }`
   - RacePlanner (8D): `{ gridPath, segments, profiles, totalTime }`

## Backward Compatibility
The RacePlanner maintains backward compatibility through adapter methods for maze-explorer.js:
- `setWallMap(knownWalls)` - Set maze configuration
- `computeOptimalPathOblique(x, y)` - Get grid path (8D)
- `compressToRacingLine(gridPath)` - Convert cells to segments
- `computeProfilesForSegments(segments)` - Compute velocity profiles

## Testing
The system should now:
1. ✓ Load without undefined reference errors
2. ✓ Compute 8D race plans successfully
3. ✓ Sync explorer path with visualized race plan
4. ✓ Display racing path in renderer
5. ✓ Report timing statistics correctly
