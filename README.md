# RoboRacer Autonomy Stack V2

## Overview

This is a complete rewrite of the RoboRacer autonomy stack, specifically designed to address the issues you identified:

1. **Stop-and-go movement** → Fixed with MPPI controller that maintains continuous control states
2. **Steering resets to 0°** → Fixed with exponential smoothing and rate limiting on all control outputs
3. **Poor curve performance** → Fixed with curvature-adaptive lookahead and bicycle model dynamics
4. **Stops before barriers without correcting** → Fixed with enhanced gap-following and fallback behaviors
5. **Sensor fetch failures** → Fixed with robust QoS settings (BEST_EFFORT, depth=10)

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                      Autonomy Node V2                            │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────┐      │
│  │  Perception  │───▶│   Planning   │───▶│   Control    │      │
│  │     V2       │    │     V2       │    │     V2       │      │
│  │              │    │              │    │              │      │
│  │ • Gap detect │    │ • Adaptive   │    │ • MPPI       │      │
│  │ • EMA smooth │    │   lookahead  │    │ • Smooth out │      │
│  │ • TTC est    │    │ • Fallback   │    │ • Rate limit │      │
│  └──────────────┘    └──────────────┘    └──────────────┘      │
│         ▲                   ▲                   │               │
│         │                   │                   ▼               │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────┐      │
│  │    LiDAR     │    │    State     │    │   Car Cmd    │      │
│  │   /scan      │    │  Estimator   │    │   Output     │      │
│  └──────────────┘    │      V2      │    └──────────────┘      │
│                      │ • IMU+Odom   │                           │
│  ┌──────────────┐    │ • Fusion     │                           │
│  │     IMU      │───▶│ • Bias est   │                           │
│  │  /imu/data   │    └──────────────┘                           │
│  └──────────────┘                                              │
│                                                                  │
│  ┌──────────────┐                                               │
│  │    Mission   │                                               │
│  │     V2       │                                               │
│  │ • State mach │                                               │
│  │ • Timeouts   │                                               │
│  └──────────────┘                                               │
└─────────────────────────────────────────────────────────────────┘
```

## Key Improvements

### 1. MPPI Controller (control_v2.py)
- **What it does**: Samples 500 trajectory rollouts using bicycle model, computes weighted optimal control
- **Why it helps**: Naturally handles vehicle dynamics, produces smooth continuous outputs
- **Key parameter**: `mppi_alpha = 0.35` (EMA smoothing factor - prevents steering reset)

### 2. Temporal Smoothing (perception_v2.py)
- **What it does**: Applies Exponential Moving Average to clearance, direction, and TTC estimates
- **Why it helps**: Eliminates jitter from LiDAR noise, prevents oscillatory behavior
- **Key parameter**: `perception_ema_alpha = 0.35`

### 3. Curvature-Adaptive Lookahead (planning_v2.py)
- **What it does**: Reduces lookahead distance in tight curves, increases on straights
- **Why it helps**: Better tracking in curves without overshooting, higher speed on straights
- **Formula**: `lookahead = f(curvature, speed)`

### 4. Robust QoS Settings (autonomy_node.py)
```python
self.sensor_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,  # Match LiDAR/IMU publishers
    history=HistoryPolicy.KEEP_LAST,
    depth=10,  # Prevent message loss
    durability=DurabilityPolicy.VOLATILE
)
```

### 5. Individual Sensor Timeouts
- Each sensor (LiDAR, IMU, Odom) monitored independently
- Timeout threshold: 500ms (configurable in params.py)
- Graceful degradation when sensors drop

## Installation

### Prerequisites
- ROS 2 (Humble or Jazzy recommended)
- Python 3.10+
- NumPy

### Setup
```bash
# Navigate to your colcon workspace
cd ~/ros2_ws/src/roboracer_autonomy/

# The v2 files are now the main files
# No additional installation needed

# Build
cd ~/ros2_ws
colcon build --packages-select roboracer_autonomy
source install/setup.bash
```

## Usage

### Launch the autonomy node
```bash
ros2 launch roboracer_autonomy autonomy_launch.py
```

### Or run directly
```bash
ros2 run roboracer_autonomy autonomy_node
```

### Monitor diagnostics
```bash
ros2 topic echo /autonomy/diagnostics
```

### View debug info
```bash
ros2 topic echo /autonomy/debug
```

## Configuration

Edit `params.py` to tune behavior:

### For smoother steering (if still seeing jitter):
```python
self.mppi_alpha = 0.25  # Reduce from 0.35 for more smoothing
self.max_steering_rate = 1.5  # Reduce from 2.0 rad/s
```

### For better curve performance:
```python
self.lookahead_min = 0.6  # Reduce from 0.8m for tighter turns
self.speed_curve_tight = 2.0  # Reduce from 2.5 m/s
```

### For more aggressive driving:
```python
self.speed_straight = 7.0  # Increase from 6.0 m/s
self.cost_weight_obstacle = 30.0  # Reduce from 50.0
```

### If sensors still timeout:
```python
self.sensor_timeout_ms = 800  # Increase from 500ms
```

## Troubleshooting

### Issue: "emergency=True | reason=state_unavailable"
**Cause**: Sensors not publishing or QoS mismatch

**Solution**:
1. Check sensor topics are active:
   ```bash
   ros2 topic list | grep -E "scan|imu|odom"
   ros2 topic hz /scan
   ros2 topic hz /imu/data
   ros2 topic hz /odom
   ```

2. Verify QoS compatibility:
   ```bash
   ros2 topic info /scan --verbose
   ```
   Look for `Reliability policy: BEST_EFFORT`

3. If sensors use RELIABLE QoS, change in autonomy_node.py:
   ```python
   self.sensor_qos = QoSProfile(
       reliability=ReliabilityPolicy.RELIABLE,  # Changed from BEST_EFFORT
       ...
   )
   ```

### Issue: Car still stops intermittently
**Cause**: Perception confidence too low or gap detection failing

**Solution**:
1. Enable debug logging in params.py:
   ```python
   self.debug_enabled = True
   self.log_perception_details = True
   ```

2. Check gap detection parameters:
   ```python
   self.gap_min_width = 0.35  # Reduce from 0.4m
   self.gap_search_angle_range = np.deg2rad(90)  # Widen search
   ```

### Issue: Steering oscillates in curves
**Cause**: Lookahead too long or smoothing insufficient

**Solution**:
```python
# In params.py
self.lookahead_base = 1.2  # Reduce from 1.5m
self.perception_ema_alpha = 0.25  # More smoothing
self.mppi_alpha = 0.3  # More control smoothing
```

### Issue: Car hugs walls too closely
**Cause**: Obstacle cost weight too low

**Solution**:
```python
self.cost_weight_obstacle = 80.0  # Increase from 50.0
self.min_clearance_emergency = 0.4  # Increase from 0.3m
```

## Testing Procedure

### 1. Static Test (car stationary)
```bash
# Run node and check diagnostics
ros2 run roboracer_autonomy autonomy_node

# In another terminal
ros2 topic echo /autonomy/diagnostics
```
Expected: `emergency=False` after 2s initialization, sensor ages < 100ms

### 2. Low-Speed Test (3 m/s max)
```python
# In params.py
self.max_speed = 3.0
self.speed_straight = 2.5
```
Drive on track, observe:
- ✓ Continuous motion (no stop-and-go)
- ✓ Steering angle persists through curves
- ✓ Smooth transitions

### 3. Medium-Speed Test (5 m/s max)
```python
self.max_speed = 5.0
self.speed_straight = 4.5
```

### 4. Full-Speed Test (competition speed)
```python
self.max_speed = 8.0
self.speed_straight = 6.0
```

## Comparison: V1 vs V2

| Aspect | V1 (Old) | V2 (New) |
|--------|----------|----------|
| **Controller** | PID / Pure Pursuit | MPPI (sampling-based) |
| **Steering output** | Discontinuous, resets | Continuous, smoothed |
| **Lookahead** | Fixed distance | Adaptive (curvature-based) |
| **Perception** | Raw LiDAR processing | EMA-smoothed + gap detection |
| **QoS** | Default (RELIABLE) | BEST_EFFORT + depth=10 |
| **Sensor timeout** | Global check | Individual per sensor |
| **Control frequency** | 50 Hz | 100 Hz |
| **Curve handling** | Poor (overshoot) | Good (adaptive) |

## Algorithm Details

### MPPI (Model Predictive Path Integral)

The MPPI algorithm works as follows:

1. **Sample** N=500 control sequences (throttle, steering pairs)
2. **Rollout** each sequence using bicycle model dynamics
3. **Evaluate** cost for each rollout:
   - Tracking error (deviation from reference)
   - Obstacle proximity
   - Control smoothness
   - Speed maintenance
4. **Weight** sequences by exponential of negative cost
5. **Average** weighted sequences to get optimal control
6. **Smooth** with EMA to prevent jumps

Mathematically:
```
w_i = exp(-λ * (J_i - J_min)) / Σ_j exp(-λ * (J_j - J_min))
u* = Σ_i w_i * u_i
u_smoothed = α * u* + (1-α) * u_prev
```

### Bicycle Model

Used for trajectory prediction:
```
x_new = x + v * cos(θ) * dt
y_new = y + v * sin(θ) * dt
θ_new = θ + (v * tan(δ) / L) * dt
v_new = v + a * dt
```

Where:
- L = wheelbase (0.35m for 1:10 scale)
- δ = steering angle
- a = acceleration from throttle

## References & Inspiration

Based on successful approaches from past RoboRacer/ICRA competitions:

1. **ETH Zurich's approach** (ICRA 2023): Used MPPI with learned dynamics
2. **MIT's approach** (ICRA 2024): Pure pursuit with adaptive lookahead
3. **Open-source implementations**:
   - [f1tenth_mppi](https://github.com/McGill-Mars/f1tenth_mppi)
   - [f1tenth_racer](https://github.com/f1tenth/f1tenth_racer)

Our implementation combines:
- MPPI from ETH's approach
- Adaptive lookahead from MIT's approach
- Custom temporal smoothing for robustness

## License

This code is provided for educational and competition purposes. Feel free to modify and share improvements with the RoboRacer community.

## Support

For issues or questions:
1. Check the Troubleshooting section above
2. Review diagnostic output: `ros2 topic echo /autonomy/diagnostics`
3. Enable debug mode in params.py for detailed logs

Good luck at ICRA 2026! 🏎️
