# Tuning Guide: RoboRacer Free-Space MPC Stack with Automatic Multi-Track Mapping

## 1. Tuning Philosophy

This stack should be tuned like a **continuous predictive controller**, not like a finite-state machine.

Your job is to improve:

1. **corridor quality**,
2. **pose quality**,
3. **MPC smoothness**,
4. **speed envelope**,
5. **map reuse quality**.

Do not tune it by chasing individual steering spikes in isolation. Most bad behavior comes from one of four root causes:

- noisy corridor geometry,
- bad or drifting pose,
- over-aggressive MPC weights,
- map-prior mismatch.

## 2. The Tuning Order That Works Best

## Step 1: Verify wheel odometry first

Check that:

- speed sign is correct,
- steering feedback is correctly normalized,
- yaw evolves in the correct direction,
- wheel odometry is stable over several seconds.

If odometry is wrong, stop here. Nothing downstream will tune correctly.

**Main parameters:**

- `VehicleGeometry.wheelbase_m`
- `VehicleGeometry.wheel_radius_m`
- `VehicleGeometry.max_steer_angle_rad`
- `LocalizationConfig.yaw_fusion_alpha`
- `LocalizationConfig.speed_fusion_alpha`

## Step 2: Tune LiDAR corridor quality

Drive slowly and inspect:

- left/right wall stability,
- centerline smoothness,
- width estimate stability,
- confidence level,
- forward clearance and TTC.

If the corridor itself zig-zags, the MPC will also zig-zag.

**Main parameters:**

- `LidarConfig.smoothing_kernel`
- `LidarConfig.leak_fill_max_bins`
- `LidarConfig.x_bin_size_m`
- `LidarConfig.side_outlier_jump_m`
- `LidarConfig.centerline_smoothing_window`
- `LidarConfig.boundary_lookahead_m`

## Step 3: Tune mapping and localization only after perception is clean

The map should reduce jitter, not add it.

Start with low map trust and increase it only when:

- track association is stable,
- ICP corrections are small,
- map priors agree with the live corridor.

**Main parameters:**

- `MappingConfig.fingerprint_similarity_threshold`
- `MappingConfig.icp_inlier_distance_m`
- `MappingConfig.icp_max_translation_m`
- `MappingConfig.icp_max_yaw_rad`
- `MappingConfig.min_map_points_for_localization`
- `MappingConfig.min_map_points_for_prior`
- `MPCConfig.use_map_prior_weight`

## Step 4: Tune the MPC for smoothness before speed

First make the car smooth and predictable.
Only then increase speed.

**Main parameters:**

- `MPCConfig.w_lat`
- `MPCConfig.w_heading`
- `MPCConfig.w_steer_rate`
- `MPCConfig.w_steer_abs`
- `MPCConfig.w_input_smooth`
- `MPCConfig.horizon_steps`
- `MPCConfig.control_knots`
- `control_hz`

## Step 5: Tune speed envelope last

Once tracking is clean, increase speed.

**Main parameters:**

- `max_speed_mps`
- `MPCConfig.max_accel_mps2`
- `MPCConfig.max_brake_mps2`
- `MPCConfig.lateral_accel_limit_mps2`
- `MPCConfig.w_speed`
- `MPCConfig.w_progress`

## 3. What To Log On Every Run

Always save or monitor:

- `controller_status.solve_success`
- `controller_status.solve_time_ms`
- `controller_status.solve_iterations`
- `controller_status.target_speed_mps`
- `controller_status.forward_clearance_m`
- `controller_status.ttc_s`
- `map_status.pose_source`
- `map_status.map_corrected`
- `map_status.map_confidence`
- `map_status.live_confidence`
- `map_status.track_id`

If you are not logging those values, you are mostly tuning blind.

## 4. Parameter Families and What They Do

## 4.1 Corridor and wall behavior

### `smoothing_kernel`
Higher value:

- smoother walls,
- less jitter,
- more lag in sharp changes.

### `x_bin_size_m`
Higher value:

- fewer samples,
- more stability,
- less geometric detail.

Lower value:

- finer detail,
- more sensitivity to noise.

### `side_outlier_jump_m`
Lower value:

- stronger rejection of wall discontinuities,
- less wall jumping,
- risk of losing boundary points in hard turns.

## 4.2 Map behavior

### `fingerprint_similarity_threshold`
Higher value:

- safer map selection,
- less chance of using the wrong track,
- more chance of creating duplicate maps.

### `icp_inlier_distance_m`
Lower value:

- stricter localization,
- less chance of wrong corrections,
- more chance of no correction when scans are sparse.

### `use_map_prior_weight`
Higher value:

- smoother repeated laps,
- more predictive behavior,
- bigger risk if map/localization is wrong.

## 4.3 MPC shape and smoothness

### `w_lat`
Higher value:

- stronger centering in the corridor,
- more wall-avoidance behavior,
- can make the car too conservative if overdone.

### `w_heading`
Higher value:

- stronger alignment with corridor direction,
- better straight-line heading stability,
- can create strong steering responses if corridor heading is noisy.

### `w_steer_rate`
Higher value:

- smoother steering,
- less oscillation,
- slower turn-in.

### `w_steer_abs`
Higher value:

- discourages large steering magnitudes,
- stabilizes straights,
- may create understeer in tight corners.

### `w_input_smooth`
Higher value:

- less throttle/steering jerk,
- smoother transitions,
- slower response.

### `horizon_steps`
Higher value:

- more anticipation,
- better corner preview,
- heavier solve time.

### `control_knots`
Higher value:

- more flexible control sequence,
- better agility,
- less smoothness and heavier optimization.

Lower value:

- smoother commands,
- less agility.

## 4.4 Speed behavior

### `w_speed`
Higher value:

- stronger tracking of the internally generated speed profile,
- less overspeed,
- more caution in turns.

### `w_progress`
Higher value:

- more aggressive forward push,
- faster lap potential,
- more risk of late braking and wall pressure.

### `lateral_accel_limit_mps2`
Higher value:

- higher corner speed,
- more aggressive speed reference,
- more demand on corridor quality and localization.

## 4.5 Safety behavior

### `corridor_margin_m`
Higher value:

- more wall clearance,
- safer but slower,
- may over-constrain narrow sections.

### `emergency_clearance_m` / `emergency_ttc_s`
Higher values:

- earlier braking,
- safer,
- more false positives if perception is noisy.

## 5. Scenario Playbook

## 5.1 The car is bad on straight lines

### Symptom A: It weaves left-right on straights

**Likely causes**

- wall extraction is noisy,
- heading reference is noisy,
- map prior is pulling the vehicle inconsistently,
- steering penalties are too weak,
- progress reward is too aggressive.

**What to change first**

1. Increase `w_steer_rate` by 20-40%.
2. Increase `w_input_smooth` by 20-40%.
3. Increase `w_heading` slightly.
4. Reduce `w_progress` slightly.
5. Increase `smoothing_kernel` from 5 to 7.
6. Increase `centerline_smoothing_window` from 7 to 9.
7. If the map is new or unstable, reduce `use_map_prior_weight`.

**What to inspect**

- Does `map_corrected` toggle on/off rapidly?
- Is `solve_success` always true?
- Does `solve_time_ms` spike?
- Is `track_id` stable?
- Do the left/right boundaries visually jump between frames?

### Symptom B: It is stable on straights but too slow

**Likely causes**

- `max_speed_mps` too low,
- `lateral_accel_limit_mps2` too low,
- `w_speed` too large,
- `w_accel` or `w_input_smooth` too conservative,
- curvature estimate is noisy so the internal speed profile drops too much.

**What to change first**

1. Raise `max_speed_mps` gradually.
2. Raise `lateral_accel_limit_mps2` in small steps.
3. Lower `w_accel` slightly.
4. Lower `w_input_smooth` slightly.
5. If speed keeps dropping on visually straight sections, improve LiDAR smoothing before making the MPC more aggressive.

### Symptom C: It drifts to one side on straights

**Likely causes**

- map prior misalignment,
- wrong track selected,
- ICP correction bias,
- persistent one-sided wall extraction error.

**What to change first**

1. Increase `fingerprint_similarity_threshold`.
2. Set `track_name_override` explicitly for controlled experiments.
3. Lower `use_map_prior_weight`.
4. Tighten `icp_inlier_distance_m`.
5. Reduce `icp_max_translation_m` and `icp_max_yaw_rad` if corrections are too large.

## 5.2 The car is bad in curves

### Symptom D: It turns too late / understeers / misses the curve

**Likely causes**

- not enough preview,
- steering penalties are too strong,
- heading/lateral penalties are too weak,
- map prior is too weak,
- speed envelope is too aggressive for the available grip estimate.

**What to change first**

1. Increase `horizon_steps` by 2-4.
2. Increase `w_heading` and `w_lat` moderately.
3. Reduce `w_steer_abs` slightly.
4. Reduce `w_steer_rate` slightly if steering is overly damped.
5. If the map is already reliable, increase `use_map_prior_weight` slightly.
6. If still late, reduce `max_speed_mps` or `lateral_accel_limit_mps2` before making steering even more aggressive.

### Symptom E: It oscillates or snaps in curves

**Likely causes**

- steering is too agile for the corridor quality,
- `control_knots` too high,
- steering smoothness penalties too weak,
- map/live blend disagreement,
- solve quality too inconsistent.

**What to change first**

1. Increase `w_steer_rate`.
2. Increase `w_input_smooth`.
3. Increase `w_steer_abs` slightly.
4. Reduce `control_knots`.
5. Reduce `w_progress`.
6. Reduce `use_map_prior_weight` if map and live corridor disagree.
7. Lower speed while debugging.

### Symptom F: It brakes too much before curves

**Likely causes**

- curvature profile too noisy,
- `lateral_accel_limit_mps2` too low,
- `w_speed` too strong,
- `corridor_margin_m` and `w_boundary` too conservative,
- live corridor too narrow because one side is being lost.

**What to change first**

1. Improve corridor stability first.
2. Increase `lateral_accel_limit_mps2` gradually.
3. Lower `w_speed` slightly.
4. Lower `corridor_margin_m` slightly if truly too conservative.
5. Lower `w_boundary` only after confirming the corridor is correct.

### Symptom G: It stops near a wall and does not recover

**Likely causes**

- hard safety clearance / TTC trip,
- repeated solver failure,
- corridor collapse,
- true dead-end situation.

**What to do**

1. Read `controller_status.reason`.
2. If reason is `stale_lidar`, fix timing first.
3. If reason is `hard_clearance` or `hard_ttc`, the safety layer is doing its job; improve corridor or reduce speed earlier.
4. If reason is `solver_failure` or `weak_corridor`, reduce optimization difficulty:
   - lower `horizon_steps`,
   - lower `control_knots`,
   - lower `w_boundary`,
   - reduce `use_map_prior_weight`.
5. Add an explicit reverse-recovery behavior if you want wall-adjacent deadlock recovery. The current stack intentionally does not do that automatically.

## 5.3 Mapping and multi-track scenarios

### Symptom H: It chooses the wrong saved track

**What to change**

- raise `fingerprint_similarity_threshold`,
- use `track_name_override` for fixed-track experiments,
- delete obviously corrupted maps,
- reduce `use_map_prior_weight` until track selection is reliable.

### Symptom I: Map helps on lap 2 but hurts on lap 1

**Interpretation**

That is normal on a fresh track. The first lap is still mostly live-corridor-driven while the map builds.

**What to change**

- do not over-trust the map too early,
- keep `use_map_prior_weight` moderate,
- ensure `save_period_s` is not too slow,
- make sure `maps_root` persists across runs.

### Symptom J: ICP corrections are noisy

**What to change**

- tighten `icp_inlier_distance_m`,
- tighten `icp_max_translation_m`,
- tighten `icp_max_yaw_rad`,
- increase `min_map_points_for_localization`,
- temporarily disable map prior blending to isolate whether the issue is localization or control.

## 6. Real-Time and Solver Best Practices

The current implementation uses SciPy optimization.
That means solve time matters.

### Watch these numbers

- `solve_time_ms`
- `solve_iterations`
- control period = `1000 / control_hz`

### Rule of thumb

Average solve time should stay comfortably below the control period.

If it does not:

1. lower `control_hz`,
2. reduce `horizon_steps`,
3. reduce `control_knots`,
4. reduce `solver_max_iter`,
5. later migrate to CasADi/acados.

## 7. Recommended Baseline Settings

A strong conservative baseline is:

- `control_hz = 12-15`
- `max_speed_mps = 4-6` for first tuning runs
- `horizon_steps = 12-14`
- `control_knots = 5-6`
- `w_lat = 12-16`
- `w_heading = 4-6`
- `w_steer_rate = 1.2-1.8`
- `w_boundary = 150-220`
- `w_progress = 1.8-2.5`
- `use_map_prior_weight = 0.15-0.35`

Only after stable repeated laps should you push:

- higher `max_speed_mps`,
- higher `lateral_accel_limit_mps2`,
- higher `w_progress`.

## 8. Best-Practice Test Procedure

### Phase 1: Corridor-only validation

- drive slowly,
- log LiDAR corridor,
- ignore lap time.

### Phase 2: MPC smoothness validation

- medium speed,
- watch straight-line stability,
- watch solve time.

### Phase 3: Map reuse validation

- run multiple consecutive laps,
- verify that lap 2 is smoother than lap 1,
- verify correct track selection across different tracks.

### Phase 4: Speed optimization

- increase speed only after the first three phases are stable.

## 9. Final Advice

When the car behaves badly:

- do not blame the MPC first,
- do not blame the map first,
- do not blame the LiDAR first.

Instead, use the debug topics to identify which layer degraded first:

1. corridor quality,
2. pose quality,
3. map correction quality,
4. optimization quality.

If you tune in that order, this stack becomes much easier to stabilize and much faster to push toward competitive lap times.
