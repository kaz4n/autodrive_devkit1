from __future__ import annotations

from .models import CameraObservation, LidarObservation, MissionMode, SensorHeartbeat, VehicleState
from .params import MissionConfig


class MissionManager:
    def __init__(self, config: MissionConfig) -> None:
        self._config = config
        self._boot_started_at: float = 0.0
        self._previous_mode = MissionMode.BOOTSTRAP
        self._safety_hold_until: float = 0.0
        self._stale_since: float = 0.0
        self._avoid_enter_count: int = 0
        self._avoid_exit_count: int = 0

    def decide(
        self,
        now: float,
        state: VehicleState,
        lidar: LidarObservation,
        camera: CameraObservation,
        heartbeats: SensorHeartbeat,
    ) -> MissionMode:
        if self._boot_started_at <= 0.0:
            self._boot_started_at = now

        if not self._essential_sensors_fresh(now, heartbeats):
            if self._stale_since <= 0.0:
                self._stale_since = now
            stale_hold_s = max(
                self._config.sensor_timeout_s,
                0.02 * float(max(self._config.stale_cycles_before_brake, 1)),
            )
            if now - self._stale_since >= stale_hold_s:
                self._safety_hold_until = now + self._config.safety_brake_hold_s
                self._previous_mode = MissionMode.SAFETY_BRAKE
                return MissionMode.SAFETY_BRAKE
            return self._previous_mode
        self._stale_since = 0.0

        if now - self._boot_started_at < self._config.bootstrap_time_s:
            self._previous_mode = MissionMode.BOOTSTRAP
            return MissionMode.BOOTSTRAP

        in_safety_zone = (
            lidar.forward_clearance < self._config.safety_clearance_enter_m
            or lidar.ttc < self._config.safety_ttc_enter_s
        )
        hard_safety_zone = (
            lidar.forward_clearance < self._config.safety_clearance_hard_m
            or lidar.ttc < self._config.safety_ttc_hard_s
        )
        safe_to_exit = (
            lidar.forward_clearance > self._config.safety_clearance_exit_m
            and lidar.ttc > self._config.safety_ttc_exit_s
        )
        if in_safety_zone:
            self._safety_hold_until = now + self._config.safety_brake_hold_s
            self._previous_mode = MissionMode.SAFETY_BRAKE
            return MissionMode.SAFETY_BRAKE

        if self._previous_mode == MissionMode.SAFETY_BRAKE and not safe_to_exit:
            # Allow a controlled AVOID crawl after the minimum hold time if we are no longer
            # in hard-danger range; this prevents deadlock near walls.
            if now >= self._safety_hold_until and not hard_safety_zone:
                self._previous_mode = MissionMode.AVOID
                return MissionMode.AVOID
            self._previous_mode = MissionMode.SAFETY_BRAKE
            return MissionMode.SAFETY_BRAKE
        if now < self._safety_hold_until:
            self._previous_mode = MissionMode.SAFETY_BRAKE
            return MissionMode.SAFETY_BRAKE

        pose_ok = state.valid and state.confidence >= self._config.min_pose_confidence
        track_ok = lidar.confidence >= self._config.min_track_confidence and lidar.centerline.shape[0] >= self._config.min_centerline_points

        should_enter_avoid = lidar.blocked or not track_ok
        should_exit_avoid = (
            not lidar.blocked
            and track_ok
            and lidar.forward_clearance > self._config.safety_clearance_exit_m
            and lidar.ttc > self._config.safety_ttc_exit_s
        )

        if self._previous_mode == MissionMode.AVOID:
            if should_exit_avoid:
                self._avoid_exit_count += 1
            else:
                self._avoid_exit_count = 0
            if self._avoid_exit_count >= self._config.avoid_exit_consecutive_scans:
                self._avoid_exit_count = 0
                self._avoid_enter_count = 0
                self._previous_mode = MissionMode.RACE if pose_ok else MissionMode.LOCALIZE
                return self._previous_mode
            self._previous_mode = MissionMode.AVOID
            return MissionMode.AVOID

        if should_enter_avoid:
            self._avoid_enter_count += 1
        else:
            self._avoid_enter_count = 0

        if self._avoid_enter_count >= self._config.avoid_enter_consecutive_scans:
            self._avoid_enter_count = 0
            self._avoid_exit_count = 0
            self._previous_mode = MissionMode.AVOID
            return MissionMode.AVOID

        self._avoid_exit_count = 0
        self._previous_mode = MissionMode.RACE if pose_ok else MissionMode.LOCALIZE
        return self._previous_mode

    def _essential_sensors_fresh(self, now: float, heartbeats: SensorHeartbeat) -> bool:
        return (
            now - heartbeats.lidar_stamp <= self._config.sensor_timeout_s
            and now - heartbeats.imu_stamp <= self._config.sensor_timeout_s
            and now - heartbeats.newest_encoder_stamp() <= self._config.sensor_timeout_s
            and now - heartbeats.steering_stamp <= self._config.steering_timeout_s
        )
