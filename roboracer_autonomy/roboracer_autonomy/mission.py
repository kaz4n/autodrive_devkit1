from __future__ import annotations

from .models import CameraObservation, LidarObservation, MissionMode, SensorHeartbeat, VehicleState
from .params import MissionConfig


class MissionManager:
    def __init__(self, config: MissionConfig) -> None:
        self._config = config
        self._boot_started_at: float = 0.0
        self._previous_mode = MissionMode.BOOTSTRAP
        self._safety_hold_until: float = 0.0
        self._stale_cycle_count: int = 0
        self._gap_enter_count: int = 0
        self._gap_exit_count: int = 0

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
            self._stale_cycle_count += 1
            if self._stale_cycle_count >= self._config.stale_cycles_before_brake:
                self._safety_hold_until = now + self._config.safety_brake_hold_s
                self._previous_mode = MissionMode.SAFETY_BRAKE
                return MissionMode.SAFETY_BRAKE
            # Debounce one-off stale samples to avoid brake jitter.
            return self._previous_mode
        self._stale_cycle_count = 0

        if now - self._boot_started_at < self._config.bootstrap_time_s:
            self._previous_mode = MissionMode.BOOTSTRAP
            return MissionMode.BOOTSTRAP

        in_safety_zone = (
            lidar.forward_clearance < self._config.safety_clearance_enter_m
            or lidar.ttc < self._config.safety_ttc_enter_s
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
            self._previous_mode = MissionMode.SAFETY_BRAKE
            return MissionMode.SAFETY_BRAKE

        if now < self._safety_hold_until:
            self._previous_mode = MissionMode.SAFETY_BRAKE
            return MissionMode.SAFETY_BRAKE

        should_enter_gap_avoid = (
            lidar.blocked
            or abs(lidar.gap_target_angle) > self._config.gap_enter_angle_rad
        )
        should_exit_gap_avoid = (
            not lidar.blocked
            and abs(lidar.gap_target_angle) < self._config.gap_exit_angle_rad
            and lidar.forward_clearance > self._config.safety_clearance_exit_m
            and lidar.ttc > self._config.safety_ttc_exit_s
        )

        if self._previous_mode == MissionMode.GAP_AVOID:
            if should_exit_gap_avoid:
                self._gap_exit_count += 1
            else:
                self._gap_exit_count = 0

            if self._gap_exit_count >= self._config.gap_exit_consecutive_scans:
                self._gap_exit_count = 0
                self._gap_enter_count = 0
                self._previous_mode = MissionMode.TRACK
                return MissionMode.TRACK

            self._gap_enter_count = 0
            self._previous_mode = MissionMode.GAP_AVOID
            return MissionMode.GAP_AVOID

        if should_enter_gap_avoid:
            self._gap_enter_count += 1
        else:
            self._gap_enter_count = 0

        if self._gap_enter_count >= self._config.gap_enter_consecutive_scans:
            self._gap_enter_count = 0
            self._gap_exit_count = 0
            self._previous_mode = MissionMode.GAP_AVOID
            return MissionMode.GAP_AVOID

        self._gap_exit_count = 0
        self._previous_mode = MissionMode.TRACK
        return MissionMode.TRACK

    def _essential_sensors_fresh(self, now: float, heartbeats: SensorHeartbeat) -> bool:
        return (
            now - heartbeats.lidar_stamp <= self._config.sensor_timeout_s
            and now - heartbeats.imu_stamp <= self._config.sensor_timeout_s
            and now - heartbeats.newest_encoder_stamp() <= self._config.sensor_timeout_s
            and now - heartbeats.steering_stamp <= self._config.steering_timeout_s
        )
