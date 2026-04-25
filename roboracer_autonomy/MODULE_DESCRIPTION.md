# RoboRacer Autonomy Stack Upgrade

This package replaces the original reactive heading-blending stack with a path-centric racing stack:

`wheel odometry prior -> optional external localization hook -> LiDAR boundary extraction -> local / global trajectory planning -> adaptive pure pursuit -> FTG fallback`

The package is still competition-legal at runtime when used with LiDAR, front camera, IMU, wheel encoders, and steering feedback only. A higher-level localizer may be connected through a custom pose topic as long as that localizer itself uses only legal runtime sensors.

## What changed

The old stack was built around:

- dead reckoning
- LiDAR gap following
- camera heading blending
- a scalar reactive plan
- direct curvature-to-steering control

The upgraded stack is built around:

- **wheel odometry prior** with a bicycle model
- **optional external pose input** for scan-matching / particle-filter localization
- **LiDAR track extraction** that outputs left boundary, right boundary, and centerline
- **trajectory planning** with optional offline raceline loading
- **adaptive pure pursuit** with an inner steering loop
- **follow-the-gap fallback** only when the path is blocked or the track model is weak

## File-by-file module guide

### `models.py`

Core dataclasses used by the stack.

#### Main objects
- `VehicleState` / `PoseEstimate`
- `TrackBoundaries` / `LidarObservation`
- `Waypoint`
- `TrajectoryPlan` / `Plan`
- `ControlCommand`
- `SensorHeartbeat`

#### Critical points
- `MissionMode` now supports `BOOTSTRAP`, `LOCALIZE`, `RACE`, `AVOID`, `RECOVERY`, `SAFETY_BRAKE`
- `TRACK` and `GAP_AVOID` are preserved as enum aliases for compatibility
- `Plan` is now **trajectory-based**, not just a scalar heading/curvature command

---

### `state_estimator.py`

Implements `WheelOdometryEstimator` and preserves `SimpleStateEstimator` as a compatibility alias.

#### Algorithm
1. Read normalized steering feedback from `/autodrive/roboracer_1/steering`
2. Convert normalized steering to wheel angle in radians
3. Compute wheel speed from encoder angle differences
4. Fuse IMU yaw with bicycle-model yaw propagation
5. Propagate `x, y, yaw` with:
   - `x += v cos(yaw) dt`
   - `y += v sin(yaw) dt`
   - `yaw += v tan(delta) / L * dt`
6. Produce a confidence score that decays when fresh IMU / encoder data are missing

#### Critical points
- **The simulator steering feedback is normalized [-1, 1]**, so it must be converted to radians before using bicycle kinematics.
- This module is now a **prior**, not the final localization authority.
- If you connect a particle filter, scan matcher, or SLAM-based pose source, feed it into the node through `external_pose_topic`.

#### Tunable knobs
- `LocalizationConfig.yaw_fusion_alpha`
- `LocalizationConfig.speed_fusion_alpha`
- `LocalizationConfig.odom_pose_confidence_decay_s`

---

### `perception.py`

Contains three perception behaviors:

- `LidarTrackExtractor`
- `GapFallback`
- `CameraBoundaryAux`

#### `LidarTrackExtractor`
Primary racing perception module.

##### Algorithm
1. Preprocess scan:
   - replace `inf`
   - clip to valid range
   - smooth with moving average
   - close small range leaks
2. Convert scan to local XY
3. Split points into left and right candidates
4. Bin by forward distance `x`
5. For each bin:
   - left side uses the closest positive `y`
   - right side uses the closest negative `y`
6. Smooth the left and right border traces
7. Build a centerline:
   - average both borders when both are visible
   - offset from one side using a persistent width estimate when only one side is visible
8. Estimate:
   - width statistics
   - heading hint
   - curvature hint
   - forward clearance
   - TTC
   - confidence

##### Critical points
- This is now the **primary local path source**.
- The persistent width estimate is what prevents large steering jumps when one border disappears.
- Boundary extraction is done in **forward-distance bins**, which is much more stable for track following than directly chasing instantaneous gap angles.

#### `GapFallback`
Reactive fallback only.

##### Algorithm
1. Apply FTG bubble around the nearest obstacle
2. Find the largest gap
3. Choose the target inside that gap with:
   - range reward
   - angle penalty
   - continuity penalty against the previous target angle

##### Critical points
- This is **not** the main planner anymore.
- It is used only when:
   - the path is blocked
   - boundaries are weak
   - no valid trajectory is available

#### `CameraBoundaryAux`
Secondary perception module.

##### Algorithm
1. Crop ROI
2. Convert to grayscale and blur
3. Run Canny
4. Run probabilistic Hough transform
5. Separate left/right boundary candidates by slope
6. Estimate center offset and heading error

##### Critical points
- Camera has intentionally low fusion weight
- Camera is treated as **auxiliary**, not dominant
- It is most useful when LiDAR boundary confidence drops

#### Tunable knobs
- `LidarConfig.x_bin_size_m`
- `LidarConfig.centerline_smoothing_window`
- `LidarConfig.nominal_track_width_m`
- `LidarConfig.side_outlier_jump_m`
- `LidarConfig.gap_continuity_weight`
- `CameraConfig.process_period_s`

---

### `planning.py`

Contains `TrajectoryPlanner` and preserves `ReactivePlanner` as an alias for compatibility.

#### Planning modes
1. **Global raceline mode**
   - load a raceline CSV
   - find nearest point on raceline
   - extract a forward horizon window
   - validate it against the local LiDAR corridor
2. **Local centerline mode**
   - transform LiDAR centerline into world coordinates
   - build a local trajectory online
3. **Gap fallback mode**
   - generate a short constant-curvature recovery path from FTG target angle

#### Speed profile algorithm
For any path generated from points:
1. compute arc length `s`
2. compute heading `yaw`
3. compute curvature `kappa`
4. compute curvature-limited speed
5. run a backward braking pass
6. run a forward acceleration pass

#### Critical points
- The planner always tries to return a **trajectory**, not a heading.
- If a global raceline exists, it is validated against the local LiDAR corridor before use.
- Corridor mismatch must persist (cycle hysteresis + low-progress timeout) before the planner leaves raceline mode.
- If localization or corridor agreement remains poor, the planner falls back to the local centerline.
- If even that fails, it falls back to FTG.

#### Runtime stability updates
- Target speed is temporally stabilized (low-pass + rise/fall rate limits).
- Safety risk scales speed with a bounded multiplicative factor rather than hard clipping in nominal driving.
- A minimum-progress floor is used in healthy `RACE`/`AVOID` contexts to reduce stop-go dithering.

#### Raceline CSV format
Accepted columns:

- required: `x`, `y`
- optional: `s`, `yaw`, `curvature`, `target_speed`, `width`

If only `x, y` are provided, the planner computes the rest automatically.

#### Tunable knobs
- `PlannerConfig.raceline_csv_path`
- `PlannerConfig.require_external_pose_for_raceline`
- `PlannerConfig.local_horizon_m`
- `PlannerConfig.local_horizon_points`
- `PlannerConfig.lateral_accel_limit_mps2`
- `PlannerConfig.max_brake_decel_mps2`
- `PlannerConfig.max_accel_mps2`
- `PlannerConfig.corridor_validation_error_m`

---

### `control.py`

Contains `AdaptivePurePursuitController` and preserves `LowLevelController` as an alias.

#### Algorithm
1. Find the closest point on the planned trajectory
2. March forward until the dynamic lookahead is reached
3. Interpolate the exact goal point
4. Transform goal point into vehicle frame
5. Compute pure-pursuit curvature:
   - `k_pp = 2 y_goal / Ld^2`
6. Blend PP curvature with the reference path curvature
7. Convert curvature to steering angle
8. Apply steering feedforward + steering feedback
9. Apply yaw-rate damping
10. Rate-limit the normalized steering command
11. Use feedforward + PID for longitudinal control

#### Critical points
- The steering loop now uses **measured steering feedback**, not just open-loop curvature.
- The steering rate limit is set closer to the simulated actuator envelope.
- PID is reset on mission-mode changes to avoid integral windup during safety events.
- During stop/coast phases, steering holds the previous command instead of recentering, preserving steering continuity.

#### Tunable knobs
- `ControllerConfig.steering_kp`
- `ControllerConfig.steering_kd`
- `ControllerConfig.steering_feedforward_gain`
- `ControllerConfig.throttle_kp`
- `ControllerConfig.throttle_ki`
- `ControllerConfig.throttle_kd`
- `ControllerConfig.steering_rate_limit_per_s`

---

### `mission.py`

Finite-state mission manager.

#### Mission states
- `BOOTSTRAP`
- `LOCALIZE`
- `RACE`
- `AVOID`
- `RECOVERY`
- `SAFETY_BRAKE`

#### Algorithm
1. Check sensor freshness
2. Debounce stale data before braking
3. Hold safety brake for a minimum time
4. Enter `AVOID` when:
   - path is blocked
   - track confidence is too low
5. Exit `AVOID` only after several clean scans
6. Use `LOCALIZE` when pose confidence is low but the track model is still usable
7. Use `RACE` when both path and pose are healthy enough

#### Critical points
- The mission manager now uses **path validity**, not gap angle, for normal behavior switching.
- Hysteresis is applied to `AVOID` entry and exit.
- Safety braking is based on TTC and clearance with hard/soft thresholds and a hold timer.
- After hold expiry, `SAFETY_BRAKE` can transition into controlled `AVOID` crawl if hard-danger conditions clear.

---

### `autonomy_node.py`

ROS 2 integration layer.

#### Runtime data flow
1. sensor callbacks update the wheel-odometry estimator and perception buffers
2. optional external pose is received on a custom odometry topic
3. control loop picks the best available pose estimate
4. mission manager selects the mode
5. planner generates a trajectory
6. controller generates throttle + steering commands
7. node publishes:
   - `/roboracer_autonomy/pose_estimate`
   - `/roboracer_autonomy/wheel_odom`
   - `/roboracer_autonomy/mission_mode`
   - `/roboracer_autonomy/target_speed`
   - `/roboracer_autonomy/reference_source`

#### Parameters
- `use_camera`
- `max_speed_mps`
- `control_hz`
- `raceline_csv_path`
- `external_pose_topic`

#### Critical points
- The node still subscribes only to legal simulator topics directly.
- `external_pose_topic` is optional and should come from a localizer based on legal sensors.
- Camera processing is rate-limited to avoid injecting step disturbances into the fast control loop.

## Recommended integration path

### Best immediate deployment
Use the upgraded package with:

- wheel odometry prior
- LiDAR centerline extraction
- local centerline trajectory planning
- adaptive pure pursuit
- FTG fallback

This is already a major improvement over the original stack.

### Best race configuration
Add:

- offline SLAM map build
- scan-based or particle-filter localization
- saved raceline CSV loaded through `raceline_csv_path`
- optional MPC later

## Tuning order

### 1. Steering interpretation
Make sure steering feedback is being interpreted correctly as normalized command feedback. If this is wrong, everything else will look unstable.

### 2. Wheel odometry prior
Tune:
- wheel radius
- wheelbase
- yaw fusion alpha
- speed fusion alpha

### 3. LiDAR boundary extraction
Tune:
- `x_bin_size_m`
- `nominal_track_width_m`
- `centerline_smoothing_window`
- `side_outlier_jump_m`

Watch:
- centerline smoothness
- width stability
- dropout behavior in corners

### 4. Pure pursuit
Tune:
- `nominal_lookahead_m`
- `min_lookahead_m`
- `max_lookahead_m`
- `lookahead_speed_gain`

Rules of thumb:
- too twitchy -> increase lookahead
- too lazy -> reduce lookahead slightly
- clips apex late -> increase steering gain or improve raceline

### 5. Speed profile
Tune:
- `lateral_accel_limit_mps2`
- `max_brake_decel_mps2`
- `max_accel_mps2`
- `max_speed_mps`

### 6. Safety envelope
Tune:
- `stop_distance_m`
- `caution_ttc_s`
- mission safety thresholds

### 7. Steering bandwidth
Increase `steering_rate_limit_per_s` only after trajectory tracking is already stable.

## Recommended next steps

### Highest-value next upgrade
Add a legal runtime localizer and connect it to `external_pose_topic`.

Good options:
- `slam_toolbox` localization mode
- AMCL / particle filter
- scan matching against a prerecorded map

### Highest-ceiling later upgrade
Replace pure pursuit with:
- kinematic MPC
- MPCC

The current code is structured to make that swap easy because the planner now already outputs a real trajectory.
