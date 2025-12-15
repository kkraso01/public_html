# AI Agent Instructions — Line Follower Simulation

## Overview
Real-time PID controller simulation for line-following robot. Features **interactive gain tuning**, **noise simulation**, **anti-windup protection**, and **derivative filtering** to demonstrate classical control theory concepts.

## Purpose
- **Educational**: Visualize PID control loop behavior
- **Interactive Tuning**: Real-time gain adjustment sliders
- **Realistic Simulation**: Sensor noise, actuator limits
- **Performance Analysis**: Error plots, adaptive speed

## Architecture

### Control Loop
```
┌──────────────┐
│ Track Model  │ → Circular path (configurable radius)
└──────┬───────┘   Line width, curvature
       │
       ▼
┌──────────────┐
│ Sensors (IR) │ → 5 infrared sensors (array)
│              │   Digital output (on line / off line)
└──────┬───────┘   Noise simulation (miss rate)
       │
       ▼
┌──────────────┐
│ PID Control  │ → Proportional: Kp * error
│              │   Integral: Ki * ∫error dt
│              │   Derivative: Kd * d(error)/dt
└──────┬───────┘
       │
       ▼
┌──────────────┐
│ Motor Model  │ → Differential drive (left, right)
│              │   PWM simulation (0-100%)
└──────┬───────┘   Speed constraints
       │
       ▼
┌──────────────┐
│ Robot Kinematics │ → Update position, heading
└──────────────┘     Render on canvas
```

## Files

### `line_following.js` (682 lines)
- **Lines 1-100**: Canvas setup, UI initialization
- **Lines 100-200**: PID controller implementation
- **Lines 200-350**: Sensor simulation (IR array)
- **Lines 350-500**: Motor/kinematics model
- **Lines 500-682**: Rendering, animation loop

## PID Controller Implementation

### Core Algorithm
```javascript
class PIDController {
  constructor(kp, ki, kd) {
    this.kp = kp;
    this.ki = ki;
    this.kd = kd;
    
    this.integral = 0;
    this.prevError = 0;
    this.prevDerivative = 0; // For filtering
  }
  
  compute(error, dt) {
    // Proportional term
    const P = this.kp * error;
    
    // Integral term (with anti-windup)
    this.integral += error * dt;
    if (Math.abs(this.integral) > this.integralLimit) {
      this.integral = Math.sign(this.integral) * this.integralLimit;
    }
    const I = this.ki * this.integral;
    
    // Derivative term (with low-pass filter)
    const rawDerivative = (error - this.prevError) / dt;
    const derivative = this.dFilterEnabled
      ? 0.7 * this.prevDerivative + 0.3 * rawDerivative // Low-pass
      : rawDerivative;
    this.prevDerivative = derivative;
    const D = this.kd * derivative;
    
    this.prevError = error;
    
    return P + I + D;
  }
  
  reset() {
    this.integral = 0;
    this.prevError = 0;
    this.prevDerivative = 0;
  }
}
```

### Default Gains
```javascript
const DEFAULT_GAINS = {
  kp: 2.0,    // Proportional (fast response)
  ki: 0.42,   // Integral (eliminate steady-state error)
  kd: 0.32    // Derivative (damping, reduce overshoot)
};
```

### Tuning Guidelines
**Kp (Proportional)**:
- Too low: Slow response, large steady-state error
- Too high: Oscillations, overshoot
- **Sweet spot**: 1.5 - 3.0

**Ki (Integral)**:
- Too low: Persistent offset
- Too high: Windup, slow settling
- **Sweet spot**: 0.3 - 0.6

**Kd (Derivative)**:
- Too low: Overshoot, slow settling
- Too high: Noise amplification, jitter
- **Sweet spot**: 0.2 - 0.5

### Anti-Windup Protection
Prevents integral from accumulating when actuator saturates:
```javascript
// Clamp integral
const MAX_INTEGRAL = 10.0;
this.integral = Math.max(-MAX_INTEGRAL, Math.min(MAX_INTEGRAL, this.integral));

// Alternative: Back-calculation (advanced)
if (output > MAX_OUTPUT) {
  const excessControl = output - MAX_OUTPUT;
  this.integral -= excessControl / this.ki;
}
```

### Derivative Filtering
Low-pass filter to reduce noise sensitivity:
```javascript
// First-order IIR filter
const alpha = 0.3; // Cutoff (0 = no filter, 1 = raw derivative)
derivative_filtered = alpha * derivative_raw + (1 - alpha) * derivative_prev;
```

## Sensor Model

### IR Sensor Array
```
         Front of Robot
              ╱╲
             ╱  ╲
  [S0] [S1] [S2] [S3] [S4]
   ←────────●────────→
         Center
```

**Sensor Positions** (relative to robot center):
```javascript
const sensorOffsets = [
  -30, // S0: Far left
  -15, // S1: Left
  0,   // S2: Center
  15,  // S3: Right
  30   // S4: Far right
];
```

**Output** (binary):
- `1` (white): Over line
- `0` (black): Off line

**Weighted Position Error**:
```javascript
// Position encoding: -4, -2, 0, +2, +4
const weights = [-4, -2, 0, 2, 4];
let weightedSum = 0;
let activeCount = 0;

for (let i = 0; i < 5; i++) {
  if (sensors[i] === 1) {
    weightedSum += weights[i];
    activeCount++;
  }
}

const error = activeCount > 0 ? weightedSum / activeCount : 0;
```

**Special Cases**:
```javascript
// All sensors black → lost line
if (activeCount === 0) {
  error = prevError > 0 ? 8 : -8; // Continue turning
}

// All sensors white → on line center (unlikely)
if (activeCount === 5) {
  error = 0; // Perfect alignment
}
```

### Noise Simulation
```javascript
if (noiseEnabled) {
  for (let i = 0; i < sensors.length; i++) {
    if (Math.random() < MISS_RATE) {
      sensors[i] = 1 - sensors[i]; // Flip bit
    }
  }
}

const MISS_RATE = 0.05; // 5% error rate
```

## Motor and Kinematics

### Differential Drive Model
```javascript
// PID output → steering correction
const steering = pidOutput;

// Base speed (constant or adaptive)
const baseSpeed = adaptiveSpeed
  ? BASE_SPEED * (1 - 0.5 * Math.abs(error) / 8) // Slow on curves
  : BASE_SPEED;

// Motor speeds
const leftSpeed = baseSpeed - steering;
const rightSpeed = baseSpeed + steering;

// Clamp to [0, MAX_SPEED]
const MAX_SPEED = 150; // PWM units
leftSpeed = Math.max(0, Math.min(MAX_SPEED, leftSpeed));
rightSpeed = Math.max(0, Math.min(MAX_SPEED, rightSpeed));
```

### Kinematics Update
```javascript
// Convert motor speeds to linear/angular velocity
const vLeft = leftSpeed * SPEED_SCALE;   // m/s
const vRight = rightSpeed * SPEED_SCALE;

const wheelBase = 0.15; // meters (distance between wheels)

// Differential drive equations
const vLinear = (vLeft + vRight) / 2;
const vAngular = (vRight - vLeft) / wheelBase;

// Update pose
robot.x += vLinear * Math.cos(robot.heading) * dt;
robot.y += vLinear * Math.sin(robot.heading) * dt;
robot.heading += vAngular * dt;

// Normalize heading to [-π, π]
robot.heading = ((robot.heading + Math.PI) % (2 * Math.PI)) - Math.PI;
```

## Interactive Features

### Real-Time Controls
- **Kp Slider**: Adjust proportional gain (0 - 5)
- **Ki Slider**: Adjust integral gain (0 - 2)
- **Kd Slider**: Adjust derivative gain (0 - 2)
- **Speed Slider**: Base speed (50 - 200 PWM)

### Toggles
- **Noise**: Enable/disable sensor errors
- **Anti-Windup**: Clamp integral term
- **D-Filter**: Low-pass filter on derivative
- **Adaptive Speed**: Slow down on curves

### Reset Button
- Clear integral accumulator
- Reset robot to start position
- Maintain current gains

## Visualization

### Track Rendering
```javascript
// Circular track
ctx.strokeStyle = '#333'; // Dark gray line
ctx.lineWidth = track.width;
ctx.beginPath();
ctx.arc(track.center.x, track.center.y, track.radius, 0, 2 * Math.PI);
ctx.stroke();
```

### Sensor Indicators
```javascript
// Color-code sensors (red = off line, green = on line)
sensorEls.forEach((el, i) => {
  el.style.backgroundColor = sensors[i] === 1 ? '#00ff00' : '#ff0000';
});
```

### Error Plot (Optional)
Real-time graph of error over time:
```javascript
const errorHistory = [];
const MAX_HISTORY = 200; // 200 frames

errorHistory.push(error);
if (errorHistory.length > MAX_HISTORY) {
  errorHistory.shift();
}

// Draw on mini-canvas
plotCanvas.clearRect(0, 0, width, height);
plotCanvas.strokeStyle = '#0080ff';
plotCanvas.beginPath();
for (let i = 0; i < errorHistory.length; i++) {
  const x = (i / MAX_HISTORY) * width;
  const y = height / 2 - (errorHistory[i] / 8) * height / 2;
  if (i === 0) plotCanvas.moveTo(x, y);
  else plotCanvas.lineTo(x, y);
}
plotCanvas.stroke();
```

## Common Tasks for AI Agents

### Easy
- Add new track shapes (square, oval, figure-8)
- Implement PID auto-tuner (Ziegler-Nichols)
- Add motor speed indicators (speedometer)
- Create lap timer

### Medium
- Implement fuzzy logic controller (compare to PID)
- Add obstacle avoidance
- Multi-robot racing (2+ robots)
- Record/replay trajectories

### Hard
- Model Predictive Control (MPC) for path following
- Machine learning controller (neural network)
- Real sensor data integration (Arduino serial)
- 3D visualization (Three.js robot model)

## Known Issues & Solutions

### Issue #1: Robot Wobbles on Straight Line
**Symptom**: Oscillates even when centered  
**Cause**: Derivative gain too high (amplifies noise)  
**Fix**: Reduce `Kd` to 0.1-0.2 or enable D-filter

### Issue #2: Can't Complete Sharp Turns
**Symptom**: Loses line on tight curves  
**Cause**: Speed too high for control authority  
**Fix**: Enable adaptive speed or reduce base speed

### Issue #3: Persistent Offset (Doesn't Center)
**Symptom**: Tracks line but consistently off-center  
**Cause**: Integral gain too low or zero  
**Fix**: Increase `Ki` to 0.3-0.5

### Issue #4: Integral Windup on Startup
**Symptom**: Overshoots wildly when starting  
**Cause**: Integral accumulates during initial misalignment  
**Fix**: Enable anti-windup or reset integral on large errors

### Issue #5: Jittery Motion with Noise Enabled
**Symptom**: Erratic steering corrections  
**Cause**: Derivative amplifies sensor noise  
**Fix**: Enable D-filter (low-pass) or reduce `Kd`

## Performance Considerations

### Animation Frame Rate
```javascript
// Target 60 FPS
const targetFPS = 60;
const dt = 1 / targetFPS;

// Use requestAnimationFrame for smooth animation
function animate() {
  update(dt);
  render();
  requestAnimationFrame(animate);
}
```

### Viewport Optimization
```javascript
// Pause when off-screen (see copilot-instructions.md)
window.ViewportObserver.observe(canvas, control, 0.1);
```

## Testing Checklist

- [ ] Robot completes full lap without losing line
- [ ] Centered on line in straight sections
- [ ] Navigates sharp turns smoothly
- [ ] Stable with default gains
- [ ] Sliders update gains in real-time
- [ ] Reset button returns to start
- [ ] Noise mode still tracks (with oscillations)
- [ ] Adaptive speed reduces on curves

## PID Tuning Tips

### Manual Tuning Process
1. Set all gains to zero
2. Increase `Kp` until oscillations start, then reduce by 20%
3. Increase `Kd` to dampen oscillations (reduce overshoot)
4. Increase `Ki` slowly to eliminate offset (watch for windup)
5. Fine-tune all three iteratively

### Ziegler-Nichols Method
1. Set `Ki = 0`, `Kd = 0`
2. Increase `Kp` until sustained oscillations (critical gain `Kc`)
3. Measure oscillation period `Pc`
4. Apply formulas:
   - `Kp = 0.6 * Kc`
   - `Ki = 1.2 * Kc / Pc`
   - `Kd = 0.075 * Kc * Pc`

## Dependencies

- **Canvas API**: 2D rendering
- **ViewportObserver**: Pause when off-screen (optional)
- **EventBus**: Inter-module communication (optional)
- No external libraries (pure vanilla JS)

## References

- **PID Control**: Åström & Hägglund, "PID Controllers: Theory, Design, and Tuning" (1995)
- **Line Following**: Jones & Flynn, "Mobile Robots: Inspiration to Implementation" (1999)
- **Anti-Windup**: Bohn & Atherton, "An Analysis Package Comparing PID Anti-Windup Strategies" (1995)
- **Derivative Filtering**: Visioli, "Practical PID Control" (2006)

---

**Last Updated**: Dec 2025  
**Maintainer**: Konstantin Krasovitskiy  
**Status**: Production (educational demo)
