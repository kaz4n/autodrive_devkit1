# RoboRacer Autonomy Tuning & Performance Test Guide

This guide gives a practical, repeatable tuning workflow for the path-centric stack.
It is intentionally ordered so you do not waste time tuning downstream modules on top of unstable upstream signals.

## 1) Performance metrics to log every run

Track these metrics first. Do not tune blind.

- **Lap time**: median of 3 clean laps.
- **Lap consistency**: stddev of lap times.
- **Tracking error**: 95th percentile lateral error to planned path.
- **Steering smoothness**: stddev of steering command derivative.
- **Safety interventions**: count and duration of `SAFETY_BRAKE`, `AVOID`, `RECOVERY`.
- **Path health**:
  - track confidence (mean + p10)
  - centerline points count (mean + p10)
  - blocked ratio
- **Speed realization**: commanded speed vs actual speed error (mean absolute error).

Minimum acceptance target for “faster but still robust”:
- lower median lap time
- no increase in safety-brake count
- no increase in 95th percentile tracking error

## 2) Tuning order (must follow)

1. Steering feedback interpretation + odometry prior
2. LiDAR boundary extraction stability
3. Lookahead schedule for pure pursuit
4. Speed profile limits (curvature + accel/decel)
5. Safety thresholds and hysteresis
6. Steering/throttle bandwidth
7. External localization (if used)
8. MPC/MPCC swap (future)

---

## 3) Stage-by-stage checklist

### Stage A: Steering feedback + odometry prior

Primary parameters:
- `vehicle.max_steer_angle_rad`
- `localization.yaw_fusion_alpha`
- `localization.speed_fusion_alpha`
- `localization.odom_pose_confidence_decay_s`

What to test:
- constant radius circle (left/right)
- slalom at low speed
- 10–20 s straight-line drift check

Pass criteria:
- estimated yaw-rate matches IMU trend without oscillation
- heading drift remains bounded in straight motion
- confidence decays gracefully during bad data, not instantly to zero

Failure signatures:
- oversteer/understeer prediction mismatch in both turns => steering normalization likely wrong
- fast oscillatory heading estimate => `yaw_fusion_alpha` too aggressive or noisy IMU fusion

### Stage B: LiDAR corridor/centerline stability

Primary parameters:
- `lidar.smoothing_kernel`
- `lidar.x_bin_size_m`
- `lidar.centerline_smoothing_window`
- `lidar.side_outlier_jump_m`
- `lidar.min_boundary_points_per_side`
- `lidar.nominal_track_width_m`

What to test:
- one representative lap with no dynamic obstacles
- one lap with intentional partial occlusion/noise (if simulator supports)

Pass criteria:
- centerline is continuous through corners
- track width estimate is stable, no large frame-to-frame jumps
- FTG fallback does not trigger in normal cornering

Quick tuning heuristics:
- jittery centerline => increase `smoothing_kernel` or `centerline_smoothing_window`
- slow/laggy path in chicanes => decrease `x_bin_size_m` or smoothing window slightly
- frequent side swaps/outliers => tighten `side_outlier_jump_m`

### Stage C: Lookahead schedule (path tracking behavior)

Primary parameters:
- `planner.nominal_lookahead_m`
- `planner.min_lookahead_m`
- `planner.max_lookahead_m`
- `planner.lookahead_speed_gain`

What to test:
- medium-speed lap and high-speed lap
- inspect steering rate and lateral error in tight turns

Pass criteria:
- no corner-cutting at speed
- no steering chatter on straights
- lateral error decreases or stays equal vs baseline

Quick tuning heuristics:
- corner-cutting / late apex miss => reduce lookahead (especially max)
- nervous steering on straights => increase nominal/min lookahead
- unstable at high speed only => reduce `lookahead_speed_gain` or cap `max_lookahead_m`

### Stage D: Speed profile and grip envelope

Primary parameters:
- `planner.max_speed_mps`
- `planner.lateral_accel_limit_mps2`
- `planner.max_brake_decel_mps2`
- `planner.max_accel_mps2`
- `planner.clearance_speed_gain`
- `planner.narrow_width_slowdown_m`

What to test:
- full lap pace sweep (e.g., max speed increments)
- confirm curvature-limited speed in high-curvature segments

Pass criteria:
- no persistent under-speeding on straights
- no repeated overshoot in same corners
- braking profile is smooth (no abrupt sawtooth target speed)

Quick tuning heuristics:
- repeated corner entry overspeed => lower `lateral_accel_limit_mps2` or raise braking decel authority
- too conservative everywhere => increase `max_speed_mps` gradually and re-check safety mode counts

### Stage E: Safety logic and mission hysteresis

Primary parameters:
- `mission.safety_ttc_enter_s`, `mission.safety_ttc_exit_s`
- `mission.safety_clearance_enter_m`, `mission.safety_clearance_exit_m`
- `mission.avoid_enter_consecutive_scans`, `mission.avoid_exit_consecutive_scans`
- `lidar.caution_ttc_s`, `lidar.hard_ttc_s`

What to test:
- clean lap (ensure no false positives)
- obstacle-in-lane scenario (ensure timely braking)

Pass criteria:
- no mode chatter between `RACE` and `AVOID`
- intervention only when truly needed

Quick tuning heuristics:
- false `AVOID` chatter => widen hysteresis gap and/or increase entry consecutive scans
- late braking => increase TTC thresholds or stop distances

### Stage F: Inner-loop bandwidth and actuator realism

Primary parameters:
- `controller.steering_kp`, `controller.steering_kd`
- `controller.steering_feedforward_gain`
- `controller.steering_rate_limit_per_s`
- `controller.steer_yaw_rate_damping`
- `controller.throttle_kp`, `controller.throttle_ki`, `controller.throttle_kd`
- `controller.throttle_rate_limit_per_s`

What to test:
- step/sine steering demand (if available)
- high-speed corner transitions
- speed step tracking

Pass criteria:
- steering tracks commanded trend without high-frequency oscillation
- throttle loop converges quickly without sustained oscillation

Quick tuning heuristics:
- steering oscillation => reduce `steering_kp` or increase damping
- sluggish steering => increase feedforward and/or rate limit (within actuator limits)
- speed hunting => lower throttle gains or tighten derivative filtering strategy

---

## 4) Practical experiment matrix

For each stage, run a small matrix and keep only one change category per experiment:

- **Baseline**: current defaults
- **Conservative**: lower aggressiveness (smoother, safer)
- **Aggressive**: higher pace candidate

Example for lookahead stage:
- Baseline: defaults
- Conservative: +15% nominal/min lookahead, -10% speed gain
- Aggressive: -10% nominal lookahead, +10% speed gain

Pick winner by objective metrics, not feel.

## 5) Recommended first parameters to tune in this repository

If you want a concrete “start here” list, tune these first in order:

1. `vehicle.max_steer_angle_rad`
2. `localization.yaw_fusion_alpha`
3. `lidar.centerline_smoothing_window`
4. `lidar.x_bin_size_m`
5. `planner.nominal_lookahead_m`
6. `planner.lookahead_speed_gain`
7. `planner.lateral_accel_limit_mps2`
8. `controller.steering_kp`
9. `controller.steering_rate_limit_per_s`
10. `mission.safety_ttc_enter_s`

## 6) Common anti-patterns

- Tuning throttle gains before path/centerline stability is solved
- Increasing max speed before braking/curvature constraints are validated
- Disabling safety thresholds to force faster lap times during development
- Simultaneously changing planner + controller + safety parameters in one run

## 7) Upgrade path once baseline is stable

After consistent laps with low intervention counts:

1. Enable external localization (`localization.external_pose_topic`)
2. Validate global raceline mode (`planner.raceline_csv_path`)
3. Re-tune lookahead and lateral accel limit on raceline source
4. Only then evaluate MPC/MPCC replacement
