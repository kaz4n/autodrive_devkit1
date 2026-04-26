# RoboRacer Autonomy Stack v2 - MPPI-Based Racing Controller

## Overview

This is an **improved autonomy stack** for the RoboRacer Autonomous Racing Competition that addresses the key issues you identified:

### Problems Solved

1. **Non-continuous movement (stop-and-go)**: The MPPI controller maintains continuous control states across mode transitions, eliminating the jerky stop-start behavior.

2. **Steering reset to zero**: The new controller preserves steering angle between cycles and uses exponential moving average smoothing, so wheels don't snap back to center.

3. **Poor curve performance**: MPPI naturally handles vehicle dynamics and curvature constraints through its cost function, producing smoother trajectories in curves.

4. **Stopping before barriers without correction**: Enhanced LiDAR perception with better temporal consistency and gap-following fallback allows the car to find alternative paths instead of freezing.

5. **Sensor data fetch issues**: Improved QoS settings and more granular sensor timeout handling provide better diagnostics and graceful degradation.

## Architecture Comparison

### Original Stack (v1)
```
Dead Reckoning → LiDAR Gap Following → Camera Heading Blending → Reactive Scalar Plan → Curvature-to-Steering
```

### New Stack (v2)
```
Wheel Odometry Prior → LiDAR Boundary Extraction → Trajectory Planning → MPPI Controller → Smooth Commands
```

## Key Improvements

### 1. MPPI Controller (`control_v2.py`)
- **Sampling-based optimization**: Rolls out 500 trajectory samples using bicycle model dynamics
- **Cost-weighted averaging**: Produces smooth, optimal controls instead of reactive commands
- **State persistence**: Maintains throttle/steering history across mode changes
- **Natural dynamics handling**: Respects steering rate limits and vehicle kinematics

### 2. Enhanced Perception (`perception_v2.py`)
- **Temporal smoothing**: Centerline estimates blend with previous frames for consistency
- **Better boundary extraction**: More robust in curves with adaptive binning
- **Improved gap fallback**: Higher continuity weight prevents oscillating target angles

### 3. Better Planning (`planning_v2.py`)
- **Curvature-adaptive lookahead**: Reduces lookahead in high-curvature sections
- **Smooth speed profiles**: Backward/forward passes with additional smoothing
- **Graceful fallbacks**: Raceline → Local centerline → Gap following hierarchy

### 4. Improved State Estimation (`state_estimator_v2.py`)
- **Adaptive sensor fusion**: Adjusts IMU fusion based on motion intensity
- **Filtered yaw rate**: Better noise rejection for steering control
- **Continuous prediction**: Maintains state even during brief sensor dropouts

### 5. Enhanced Mission Management (`mission_v2.py`)
- **Granular timeouts**: Separate timeouts for LiDAR, IMU, encoders, steering
- **Stronger hysteresis**: Prevents mode chatter between RACE and AVOID
- **Safety hold timers**: Ensures smooth transitions into/out of safety brake

## File Structure

```
roboracer_autonomy/
├── autonomy_node_v2.py       # Main ROS 2 node (use this instead of autonomy_node.py)
├── params_v2.py              # Configuration parameters
├── control_v2.py             # MPPI controller
├── planning_v2.py            # Trajectory planner
├── perception_v2.py          # LiDAR and camera processing
├── state_estimator_v2.py     # Wheel odometry + IMU fusion
├── mission_v2.py             # Mode management
└── models.py                 # Shared data types (reuse from v1)
```

## Quick Start

### 1. Update Launch File

Replace the node entry in your launch file:

```python
# OLD
Node(
    package='roboracer_autonomy',
    executable='autonomy_node',
    name='roboracer_autonomy',
    ...
)

# NEW
Node(
    package='roboracer_autonomy',
    executable='autonomy_node_v2',  # Changed
    name='roboracer_autonomy_v2',   # Changed
    parameters=[{
        'use_camera': True,
        'max_speed_mps': 6.0,  # Start conservative
        'control_hz': 100.0,   # Higher frequency for MPPI
        'raceline_csv_path': '',  # Optional: path to raceline CSV
        'external_pose_topic': '',  # Optional: for SLAM/particle filter
        'debug_enabled': True,  # Enable detailed diagnostics
    }],
    ...
)
```

### 2. Tuning Order (Critical!)

Follow this exact order - do NOT skip steps:

#### Step 1: Verify Steering Feedback (MOST IMPORTANT)
```bash
ros2 topic echo /roboracer_autonomy/diagnostics
```
Check that `steering` values change smoothly and match actual wheel position.

If steering jumps or resets:
- Check `/autodrive/roboracer_1/steering` topic format (should be normalized [-1, 1])
- Verify `vehicle.max_steer_angle_rad` matches your car's physical limit

#### Step 2: Low-Speed Testing (2-3 m/s)
Run at reduced speed first:
```yaml
max_speed_mps: 3.0
nominal_lookahead_m: 1.80
```

Observe:
- Does the car move continuously without stopping?
- Do wheels maintain angle through turns?
- Is steering smooth (no jitter)?

#### Step 3: LiDAR Processing
Check centerline quality:
```bash
ros2 topic echo /roboracer_autonomy/diagnostics | grep corridor_conf
```

If confidence is low (< 0.3):
- Increase `lidar.smoothing_kernel` to 9
- Increase `lidar.centerline_smoothing_window` to 11
- Reduce `lidar.x_bin_size_m` to 0.10

#### Step 4: Increase Speed Gradually
Once stable at 3 m/s:
```yaml
max_speed_mps: 4.5
lateral_accel_limit_mps2: 5.5
```

Test in curves - if cutting corners:
- Reduce `nominal_lookahead_m` by 0.1
- Increase `steering_kp` to 1.4

#### Step 5: Full Speed (6+ m/s)
```yaml
max_speed_mps: 6.5
lateral_accel_limit_mps2: 6.0
lookahead_speed_gain: 0.18
```

### 3. Parameter Reference

#### Critical Parameters (tune these first)

| Parameter | Default | Effect | Too High | Too Low |
|-----------|---------|--------|----------|---------|
| `vehicle.max_steer_angle_rad` | 0.5236 | Max steering angle | Oversteer, instability | Understeer, wide turns |
| `controller.num_samples` | 500 | MPPI samples | Smoother but slower | Jerky, suboptimal |
| `controller.temperature` | 0.5 | Exploration vs exploitation | Risky, aggressive | Conservative, slow |
| `planner.nominal_lookahead_m` | 1.50 | Path tracking distance | Lazy, cuts corners | Twitchy, oscillates |
| `lidar.centerline_smoothing_window` | 9 | LiDAR smoothing | Laggy response | Noisy, jittery |

#### MPPI-Specific Parameters

```python
# In params_v2.py, ControllerConfig:
num_samples: int = 500          # More = smoother but CPU-intensive
sample_horizon_s: float = 1.2   # How far to predict
dt: float = 0.02                # Rollout timestep

# Cost weights:
cost_tracking_weight: float = 5.0    # Path following importance
cost_smoothness_weight: float = 2.0  # Control smoothness
cost_collision_weight: float = 10.0  # Obstacle avoidance
cost_speed_weight: float = 1.5       # Speed tracking
```

### 4. Diagnostics

The v2 stack provides enhanced diagnostics:

```bash
# Real-time monitoring
ros2 topic echo /roboracer_autonomy/diagnostics

# Example output:
DIAGNOSTICS: emergency=False | reason=race | throttle=0.450 | steering=0.120 | 
lidar_age_ms=12.5 | imu_age_ms=8.3 | encoders_age_ms=10.1 | 
corridor_conf=0.78 | clearance_m=2.34 | ttc_s=1.85 | speed=4.23 | mode=race
```

Key metrics:
- `lidar_age_ms`, `imu_age_ms`, `encoders_age_ms`: Should all be < 50ms
- `corridor_conf`: Track detection confidence (> 0.5 good, > 0.7 excellent)
- `clearance_m`: Distance to nearest obstacle ahead
- `ttc_s`: Time to collision (inf = clear path)
- `mode`: Current mission mode

### 5. Troubleshooting

#### Problem: Car still stops intermittently
**Solution**: 
- Check `mission.safety_ttc_enter_s` - increase to 0.6
- Verify LiDAR isn't detecting false obstacles
- Check `planner.min_speed_mps` - set to 0.3 minimum

#### Problem: Steering oscillates in straights
**Solution**:
- Increase `planner.nominal_lookahead_m` to 1.8
- Reduce `controller.steering_kp` to 1.0
- Increase `controller.control_ema_alpha` to 0.3

#### Problem: Can't complete curves at speed
**Solution**:
- Reduce `planner.lateral_accel_limit_mps2` to 4.5
- Increase `controller.steering_rate_limit_per_s` to 7.0
- Check `vehicle.max_steer_angle_rad` is correct

#### Problem: Sensor timeout errors
**Solution**:
```bash
# Check actual sensor frequencies
ros2 topic hz /autodrive/roboracer_1/lidar
ros2 topic hz /autodrive/roboracer_1/imu
ros2 topic hz /autodrive/roboracer_1/left_encoder

# Adjust timeouts in params_v2.py if needed:
lidar_timeout_s: 0.50  # Increase if LiDAR is slow
imu_timeout_s: 0.40
encoder_timeout_s: 0.50
```

#### Problem: Emergency brakes too frequently
**Solution**:
- Increase `mission.safety_clearance_enter_m` to 0.50
- Increase `mission.safety_ttc_enter_s` to 0.60
- Check LiDAR for noise/spurious readings

## Performance Expectations

With proper tuning, expect:

| Metric | v1 Stack | v2 Stack (Expected) |
|--------|----------|---------------------|
| Lap time consistency | Variable | ±0.5s stddev |
| Steering smoothness | Jerky | Continuous |
| Curve performance | Poor at >4 m/s | Good at 6+ m/s |
| Safety interventions | Frequent | Rare (only when needed) |
| Recovery from errors | Hard stops | Graceful degradation |

## Advanced: Adding External Localization

For best performance, add SLAM or particle filter localization:

```python
# In launch file:
'external_pose_topic': '/slam_toolbox/pose',  # or your localizer
'require_external_pose_for_raceline': False,  # Use raceline even without external pose
```

This enables:
- Global raceline following with absolute positioning
- Better long-term accuracy
- Reduced drift in wheel odometry

## Migration from v1

The v2 stack is designed as a **drop-in replacement**:

1. Keep `models.py` from v1 (unchanged)
2. Replace node executable: `autonomy_node` → `autonomy_node_v2`
3. Update parameters to use v2 defaults
4. Test at low speed first!

## Contributing / Further Improvements

Potential future enhancements:
- Full LiDAR integration in MPPI cost function (currently simplified)
- Dynamic obstacle prediction and avoidance
- Online raceline optimization
- Multi-lap learning

## References

Inspired by:
- ICRA RoboRacer past participants' open-source solutions
- Model Predictive Path Integral Control (Williams et al.)
- F1TENTH autonomous racing stacks
- MIT Racecar project

## Support

For issues specific to the v2 stack:
1. Check diagnostics output first
2. Verify sensor data is arriving (`ros2 topic echo`)
3. Test at low speed before increasing
4. Tune parameters in the specified order

Good luck with ICRA 2026! 🏎️
