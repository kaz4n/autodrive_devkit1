# RoboRacer Free-Space MPC Stack

This package rewrites the nominal autonomy loop around a single corridor-based MPC:

`wheel odometry prior -> LiDAR corridor extraction -> free-space MPC -> throttle / steering`

## What changed

- Removed the nominal mission/planner/controller pipeline from the control loop.
- Removed any dependence on saved maps or manual lap recording in nominal operation.
- Kept ROS 2 topic compatibility with the AutoDRIVE RoboRacer bridge.
- Kept wheel odometry and optional external pose fusion.
- Kept LiDAR boundary extraction, but the MPC now consumes left/right boundaries directly.

## File roles

- `autonomy_node.py`: ROS 2 node that connects the devkit topics to perception, estimation, and MPC.
- `state_estimator.py`: wheel-odometry prior using IMU, encoders, and steering feedback.
- `perception.py`: extracts left/right corridor boundaries in vehicle frame.
- `free_space_mpc.py`: corridor-based nonlinear shooting MPC with warm start and solver diagnostics.
- `planning.py`, `control.py`, `mission.py`, `map_manager.py`: compatibility shims / deprecated stubs.

## Runtime behavior

1. The node reads only live sensor streams from the bridge in the nominal loop.
2. LiDAR is reduced to left/right boundaries in the vehicle frame.
3. MPC rebuilds a local corridor every cycle and directly outputs throttle and steering.
4. If LiDAR is stale or forward clearance / TTC is unsafe, the car brakes immediately.
5. If the MPC fails repeatedly, the node holds steering and then applies brake fallback.

## Parameters you will tune first

- `max_speed_mps`
- `control_hz`
- `MPCConfig.w_lat`, `w_heading`, `w_boundary`, `w_progress`
- `MPCConfig.horizon_steps`, `control_knots`
- `MPCConfig.corridor_margin_m`

## Migration notes

- The nominal stack no longer uses a map prior.
- The nominal stack no longer switches modes.
- `external_pose_topic` is still available, but left disabled by default.
- The compatibility files remain only to avoid import breakage in older launch setups.
