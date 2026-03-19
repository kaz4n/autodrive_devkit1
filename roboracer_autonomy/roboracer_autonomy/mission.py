from __future__ import annotations

from .models import CameraObservation, LidarObservation, MissionMode, SensorHeartbeat, VehicleState
from .params import MissionConfig


class MissionManager:
    def __init__(self, config: MissionConfig) -> None:
        self._config = config
        self._boot_started_at: float = 0.0
        self._previous_mode = MissionMode.BOOTSTRAP

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
            self._previous_mode = MissionMode.SAFETY_BRAKE
            return MissionMode.SAFETY_BRAKE

        if now - self._boot_started_at < self._config.bootstrap_time_s:
            self._previous_mode = MissionMode.BOOTSTRAP
            return MissionMode.BOOTSTRAP

        if lidar.forward_clearance < 0.35 or lidar.ttc < 0.45:
            self._previous_mode = MissionMode.SAFETY_BRAKE
            return MissionMode.SAFETY_BRAKE

        if lidar.blocked or abs(lidar.gap_target_angle) > 0.30:
            self._previous_mode = MissionMode.GAP_AVOID
            return MissionMode.GAP_AVOID

        self._previous_mode = MissionMode.TRACK
        return MissionMode.TRACK

    def _essential_sensors_fresh(self, now: float, heartbeats: SensorHeartbeat) -> bool:
        return (
            now - heartbeats.lidar_stamp <= self._config.sensor_timeout_s
            and now - heartbeats.imu_stamp <= self._config.sensor_timeout_s
            and now - heartbeats.newest_encoder_stamp() <= self._config.sensor_timeout_s
            and now - heartbeats.steering_stamp <= self._config.sensor_timeout_s
        )
