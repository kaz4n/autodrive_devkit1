from __future__ import annotations

import math

from .math_utils import clamp
from .models import CameraObservation, LidarObservation, MissionMode, Plan, VehicleState
from .params import PlannerConfig


class ReactivePlanner:
    def __init__(self, config: PlannerConfig) -> None:
        self._config = config

    def plan(
        self,
        state: VehicleState,
        lidar: LidarObservation,
        camera: CameraObservation,
        mission_mode: MissionMode,
        stamp: float,
    ) -> Plan:
        if not lidar.processed_ranges.size:
            return Plan(stamp=stamp, mode=MissionMode.SAFETY_BRAKE)

        lookahead = clamp(
            self._config.nominal_lookahead_m + self._config.lookahead_speed_gain * max(state.speed, 0.0),
            self._config.min_lookahead_m,
            self._config.max_lookahead_m,
        )

        desired_heading = 0.0
        if mission_mode == MissionMode.GAP_AVOID:
            desired_heading = 0.78 * lidar.gap_target_angle + 0.22 * lidar.center_bias
        else:
            desired_heading = 0.55 * lidar.center_bias + 0.45 * lidar.gap_target_angle

        if camera.confidence >= self._config.camera_confidence_threshold:
            camera_weight = min(0.25, 0.25 * camera.confidence)
            desired_heading = (1.0 - camera_weight) * desired_heading + camera_weight * camera.heading_error

        desired_heading = float(clamp(desired_heading, -self._config.steering_bias_limit_rad, self._config.steering_bias_limit_rad))
        curvature = 2.0 * math.sin(desired_heading) / max(lookahead, 1e-3)

        if abs(curvature) < 1e-4:
            curve_speed = self._config.straight_speed_mps
        else:
            curve_speed = math.sqrt(max(self._config.lateral_accel_limit_mps2 / abs(curvature), 0.0))
        clearance_speed = self._config.clearance_speed_gain * max(lidar.forward_clearance - 0.35, 0.0)

        target_speed = min(
            self._config.max_speed_mps if mission_mode == MissionMode.TRACK else self._config.gap_mode_speed_mps,
            curve_speed,
            clearance_speed,
        )
        if mission_mode == MissionMode.TRACK and abs(desired_heading) < 0.08 and lidar.forward_clearance > 2.5:
            target_speed = max(target_speed, min(self._config.straight_speed_mps, self._config.max_speed_mps))
        if lidar.lane_width_estimate > 0.0 and lidar.lane_width_estimate < 1.05:
            target_speed = min(target_speed, 1.8)
        if lidar.ttc < 0.55 or lidar.forward_clearance < 0.40:
            target_speed = 0.0

        target_speed = float(clamp(target_speed, self._config.min_speed_mps, self._config.max_speed_mps))
        confidence = float(clamp(0.6 * lidar.confidence + 0.4 * camera.confidence, 0.0, 1.0))

        return Plan(
            stamp=stamp,
            mode=mission_mode,
            desired_heading=desired_heading,
            curvature=float(curvature),
            lookahead=lookahead,
            target_speed=target_speed,
            forward_clearance=lidar.forward_clearance,
            ttc=lidar.ttc,
            confidence=confidence,
            metadata={
                'curve_speed': float(curve_speed),
                'clearance_speed': float(clearance_speed),
                'gap_target_angle': float(lidar.gap_target_angle),
                'center_bias': float(lidar.center_bias),
            },
        )
