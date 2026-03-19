# roboracer_autonomy

Reactive autonomy stack for the AutoDRIVE RoboRacer competition.

## Runtime inputs used

- `/autodrive/roboracer_1/lidar`
- `/autodrive/roboracer_1/front_camera` (optional)
- `/autodrive/roboracer_1/imu`
- `/autodrive/roboracer_1/left_encoder`
- `/autodrive/roboracer_1/right_encoder`
- `/autodrive/roboracer_1/steering`

## Runtime outputs used

- `/autodrive/roboracer_1/throttle_command`
- `/autodrive/roboracer_1/steering_command`

## Internal modules

- `state_estimator.py` — IMU + encoder dead reckoning
- `perception.py` — LiDAR gap logic, duct-gap sealing, camera boundary extraction
- `planning.py` — reactive heading and speed selection
- `mission.py` — finite-state mission switching
- `control.py` — steering conversion plus throttle PID
- `autonomy_node.py` — ROS 2 node wiring
