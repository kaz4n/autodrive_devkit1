# Project Description: RoboRacer Free-Space MPC Stack with Automatic Multi-Track Mapping

## 1. Objective

The goal of this package is to replace the old **reactive mission + planner + pure-pursuit** stack with a smoother and more continuous racing architecture built around a **free-space model predictive controller (MPC)**.

The new nominal runtime loop is:

`wheel odometry prior -> LiDAR corridor extraction -> automatic multi-track map manager -> free-space MPC -> throttle / steering`

The design target is not only to keep the vehicle safe, but to make the car:

- smoother in straight sections,
- more predictive in curves,
- less prone to stop-go mode chatter,
- less dependent on hand-made logging laps,
- and easier to retune for different tracks.

## 2. Architecture Summary

### 2.1 State Estimation

**File:** `state_estimator.py`

This module provides a **competition-legal wheel-odometry prior** using only:

- IMU orientation and yaw-rate,
- rear wheel encoders,
- steering feedback.

It does not try to be a full SLAM or global localization solution. Instead, it serves as the fast local prior used by all downstream modules.

### 2.2 Corridor Perception

**File:** `perception.py`

This module converts raw LiDAR scans into a local geometric corridor:

- left boundary,
- right boundary,
- centerline,
- width estimates,
- heading hint,
- curvature hint,
- forward clearance,
- TTC,
- confidence.

It also keeps an optional **gap-follow debug signal** for diagnostics, but gap-follow is no longer part of the nominal control loop.

### 2.3 Automatic Multi-Track Mapping

**File:** `map_manager.py`

This module gives the stack a lightweight mapping and localization capability without requiring a dedicated manual record lap.

It supports:

- automatic track identification,
- automatic track creation,
- persistent on-disk map storage,
- reloading multiple saved maps,
- incremental map updates every run,
- ICP-like pose correction against saved boundary clouds,
- local map priors for corridor blending.

### 2.4 Continuous Free-Space MPC

**File:** `free_space_mpc.py`

This is the nominal controller. It directly outputs:

- `throttle`
- `steering`

It no longer depends on:

- a discrete mission manager,
- a separate planner node,
- a pure-pursuit controller.

Instead, it predicts the vehicle state over a short horizon inside the live LiDAR corridor, optionally blended with a local map prior.

### 2.5 Minimal Safety Layer

**File:** `mission.py`

Despite the filename, this module is **not a mission FSM**. It is only a minimal safety monitor. It handles:

- stale LiDAR,
- hard clearance violations,
- hard TTC violations,
- repeated MPC emergency/failure cycles.

Its job is to prevent unsafe commands, not to switch between nominal driving modes.

### 2.6 ROS 2 Integration

**File:** `autonomy_node.py`

This node wires together all components and publishes:

- throttle command,
- steering command,
- pose estimate,
- wheel odometry,
- track id,
- map status,
- controller status,
- reference source,
- target speed.

## 3. Detailed Algorithm by Module

## 3.1 `state_estimator.py`

### Role
A fast prior pose estimate for control and scan-to-map refinement.

### Algorithm

1. Steering feedback is interpreted as a normalized actuator value in `[-1, 1]`.
2. It is converted to a physical steering angle using `max_steer_angle_rad`.
3. Rear wheel speeds are estimated from encoder angle differences.
4. IMU yaw and yaw-rate are fused into the local state.
5. A bicycle-model prediction updates:
   - `x`,
   - `y`,
   - `yaw`,
   - `speed`,
   - `steering_angle`.
6. Confidence is decayed over time if fresh sensor updates stop arriving.

### Critical implementation points

- Steering feedback must be treated as **normalized steering**, not raw radians.
- Encoder deltas must be unwrapped correctly to avoid sign flips.
- This estimator is a **prior**, not the final truth.

## 3.2 `perception.py`

### Role
Turn raw LiDAR into a corridor model the MPC can optimize inside.

### Algorithm

1. Clip invalid ranges into configured min/max limits.
2. Smooth the range array with a moving average.
3. Fill short “leaks” in the scan so tiny missing segments do not break wall continuity.
4. Convert ranges into local `(x, y)` points.
5. Keep only forward/focus points within lookahead and view limits.
6. Extract left and right boundaries by binning in `x` and selecting the closest valid side sample.
7. Reject side jumps that exceed the side outlier threshold.
8. Build a centerline from the two boundaries or from a width estimate if only one side is visible.
9. Smooth the centerline and blend it slightly with the previous one for temporal continuity.
10. Compute:
    - `heading_hint`,
    - `curvature_hint`,
    - `forward_clearance`,
    - `ttc`,
    - `confidence`.

### Critical implementation points

- `x_bin_size_m` trades spatial detail for stability.
- `smoothing_kernel` and `centerline_smoothing_window` strongly affect straight-line smoothness.
- `side_outlier_jump_m` determines whether the extracted walls are robust or noisy.
- Corridor confidence matters indirectly because weak corridors reduce map updates and can force MPC fallback.

## 3.3 `map_manager.py`

### Role
Auto-build and reuse maps across repeated runs and across multiple tracks.

### Algorithm

1. Build a compact fingerprint from the current LiDAR observation.
2. Compare it to saved track fingerprints.
3. If a similar track exists, activate it.
4. Otherwise, auto-create a new track record.
5. Transform local observed boundaries into world coordinates using the current pose estimate.
6. Voxel-downsample the accumulated point cloud.
7. Periodically save map data to disk.
8. When enough saved map points exist, run a lightweight ICP-like least-squares correction to refine pose.
9. Reproject the active map back into the local vehicle frame to generate a **map prior corridor**.

### Critical implementation points

- `fingerprint_similarity_threshold` controls whether the wrong map gets reused.
- `icp_inlier_distance_m`, `icp_max_translation_m`, and `icp_max_yaw_rad` control localization aggressiveness.
- `min_map_points_for_localization` and `min_map_points_for_prior` decide how quickly map refinement activates.
- `use_map_prior_weight` in the MPC determines how much the controller trusts this saved map.

## 3.4 `free_space_mpc.py`

### Role
Generate direct throttle and steering commands continuously from the corridor.

### Internal model
The controller uses a **kinematic bicycle model** with state:

- `x`
- `y`
- `yaw`
- `speed`
- `steering angle`

and controls:

- longitudinal acceleration,
- steering-rate.

### Corridor construction

1. Sample an `x_grid` ahead of the car.
2. Interpolate live left/right wall positions onto that grid.
3. If a map prior is available, blend it into the live boundaries.
4. Fill missing sides using the current width estimate.
5. Compute:
   - centerline profile,
   - heading reference,
   - curvature reference,
   - speed reference from lateral acceleration limits.

### Optimization strategy

1. Parameterize control with a small number of knots.
2. Expand knots to a full control sequence over the horizon.
3. Simulate the vehicle forward.
4. Minimize a cost that includes:
   - lateral error,
   - heading error,
   - speed tracking error,
   - steering-rate penalty,
   - steering magnitude penalty,
   - acceleration penalty,
   - boundary violation penalty,
   - input smoothness penalty,
   - terminal lateral error,
   - progress reward.
5. Solve with SciPy `minimize` when available.
6. Warm-start each solve with the shifted previous solution.
7. If the solver fails, fall back to a lightweight geometric heuristic.

### Critical implementation points

- `horizon_steps` and `control_knots` are the biggest performance-vs-reactivity knobs.
- `w_progress` pushes the car forward; too much makes the car aggressive.
- `w_boundary` and `corridor_margin_m` define how strongly the car avoids walls.
- `w_steer_rate`, `w_input_smooth`, and `w_steer_abs` determine steering continuity.
- `lateral_accel_limit_mps2` shapes corner speed.

## 3.5 `mission.py` / Safety monitor

### Role
A thin safety filter around the nominal controller.

### Behavior

Precheck:

- brake if LiDAR is stale,
- brake if hard clearance is violated,
- brake if hard TTC is violated.

Postcheck:

- if the MPC keeps failing repeatedly, hold the current steering and apply fallback braking.

### Critical implementation points

- `stale_lidar_timeout_s` must be larger than your actual worst-case LiDAR delay.
- `fallback_hold_cycles` determines how tolerant the stack is to short solver hiccups.
- This layer is intentionally small; it should not become another hidden planner.

## 3.6 `autonomy_node.py`

### Role
Main runtime orchestration.

### Runtime flow

1. Read sensors.
2. Update wheel odometry.
3. Optionally select external pose if configured and fresh.
4. Run safety precheck.
5. Run corridor extraction.
6. Run map localization/refinement and update the active map.
7. Build local map prior.
8. Solve MPC.
9. Run safety postcheck.
10. Publish control and debug messages.

### Critical implementation points

- Debug topics are essential for tuning. The controller status and map status topics should be logged on every run.
- `control_hz` must match the actual solver budget.

## 4. Why This Stack Is Better Than the Old One

The old discrete stack had three structural problems:

1. It mixed perception, planning, and safety into discrete modes.
2. It used geometric tracking instead of predictive control.
3. It had no integrated notion of corridor continuity and only weak map reuse.

This stack addresses those problems by:

- removing nominal mode switching,
- using direct MPC instead of pure pursuit,
- building a reusable map automatically,
- blending live perception with prior track structure,
- producing continuous steering and throttle.

## 5. Best-Practice Operating Rules

### 5.1 Start simple, then add speed

Always begin tuning at modest speed and conservative acceleration limits. A fast unstable stack is much harder to debug than a slower stable one.

### 5.2 Tune in this order

1. wheel odometry consistency,
2. LiDAR corridor quality,
3. map association and ICP,
4. MPC smoothness,
5. corner speed,
6. straight-line speed,
7. emergency thresholds.

### 5.3 Do not tune everything at once

Change one parameter family at a time:

- perception,
- localization/map,
- MPC cost,
- vehicle limits,
- safety thresholds.

### 5.4 Log the right topics

For every test run, log:

- `controller_status`
- `map_status`
- `pose_estimate`
- `wheel_odom`
- `target_speed`
- raw LiDAR scan

### 5.5 Treat map priors carefully

A good map prior improves continuity.
A wrong map prior creates systematic steering errors.

If the car suddenly becomes consistently biased to one side after the first lap, suspect:

- wrong track selection,
- overly permissive fingerprint matching,
- ICP over-correction,
- too much `use_map_prior_weight`.

## 6. Recommended Future Upgrades

This implementation is already a large step forward, but the cleanest future upgrades are:

- replace the SciPy optimizer with CasADi/acados,
- replace the lightweight map manager with a stronger SLAM/localization backend,
- add a bounded reverse-recovery primitive for dead-end cases,
- add learned camera priors only after the LiDAR pipeline is stable,
- optionally add a raceline layer on top of the map for outright lap-time optimization.

## 7. Summary

This stack is a continuous-control RoboRacer architecture centered around:

- legal runtime sensing,
- LiDAR corridor geometry,
- automatic per-track mapping,
- direct MPC control,
- minimal safety overrides.

Its main strength is not just that it is more advanced than the old stack. Its main strength is that the modules now fit together coherently:

- perception creates a corridor,
- mapping stabilizes that corridor over repeated runs,
- MPC uses both live and mapped structure,
- safety only catches genuine failures.

That is the right foundation for smooth, high-performance autonomous racing.
