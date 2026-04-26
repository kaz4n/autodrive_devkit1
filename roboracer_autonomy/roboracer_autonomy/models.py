from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, List

import numpy as np


class MissionMode(str, Enum):
    BOOTSTRAP = 'bootstrap'
    LOCALIZE = 'localize'
    RACE = 'race'
    AVOID = 'avoid'
    SAFETY_BRAKE = 'safety_brake'


@dataclass
class VehicleState:
    stamp: float = 0.0
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    speed: float = 0.0
    yaw_rate: float = 0.0
    steering_angle: float = 0.0
    steering_normalized: float = 0.0
    linear_accel_x: float = 0.0
    valid: bool = False
    confidence: float = 0.0
    source: str = 'wheel_odom'
    covariance: np.ndarray = field(default_factory=lambda: np.zeros((6, 6), dtype=float))


@dataclass
class Waypoint:
    s: float = 0.0
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    curvature: float = 0.0
    target_speed: float = 0.0
    width: float = 0.0


@dataclass
class TrackBoundaries:
    stamp: float = 0.0
    left_boundary: np.ndarray = field(default_factory=lambda: np.zeros((0, 2), dtype=float))
    right_boundary: np.ndarray = field(default_factory=lambda: np.zeros((0, 2), dtype=float))
    centerline: np.ndarray = field(default_factory=lambda: np.zeros((0, 2), dtype=float))
    width_mean: float = 0.0
    width_min: float = 0.0
    width_std: float = 0.0
    center_bias: float = 0.0
    heading_error: float = 0.0
    gap_target_angle: float = 0.0
    heading_hint: float = 0.0
    curvature_hint: float = 0.0
    forward_clearance: float = float('inf')
    ttc: float = float('inf')
    blocked: bool = False
    confidence: float = 0.0
    processed_ranges: np.ndarray = field(default_factory=lambda: np.asarray([], dtype=float))
    angles: np.ndarray = field(default_factory=lambda: np.asarray([], dtype=float))
    metadata: Dict[str, float] = field(default_factory=dict)

    def has_centerline(self) -> bool:
        return self.centerline.shape[0] >= 2

    def has_corridor(self) -> bool:
        return self.left_boundary.shape[0] >= 2 or self.right_boundary.shape[0] >= 2


LidarObservation = TrackBoundaries


@dataclass
class CameraObservation:
    stamp: float = 0.0
    center_offset: float = 0.0
    heading_error: float = 0.0
    confidence: float = 0.0
    has_left_boundary: bool = False
    has_right_boundary: bool = False
    metadata: Dict[str, float] = field(default_factory=dict)


@dataclass
class Plan:
    stamp: float = 0.0
    mode: MissionMode = MissionMode.BOOTSTRAP
    reference_source: str = 'none'
    waypoints: List[Waypoint] = field(default_factory=list)
    desired_heading: float = 0.0
    curvature: float = 0.0
    lookahead: float = 0.0
    target_speed: float = 0.0
    target_steering_angle: float = 0.0
    forward_clearance: float = float('inf')
    ttc: float = float('inf')
    blocked: bool = False
    fallback_active: bool = False
    confidence: float = 0.0
    metadata: Dict[str, float] = field(default_factory=dict)


@dataclass
class ControlCommand:
    stamp: float = 0.0
    throttle: float = 0.0
    steering: float = 0.0
    target_speed: float = 0.0
    emergency: bool = False
    reason: str = 'stop'
    metadata: Dict[str, float] = field(default_factory=dict)


@dataclass
class SensorHeartbeat:
    lidar_stamp: float = 0.0
    imu_stamp: float = 0.0
    left_encoder_stamp: float = 0.0
    right_encoder_stamp: float = 0.0
    steering_stamp: float = 0.0
    camera_stamp: float = 0.0
    external_pose_stamp: float = 0.0
