from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, Optional

import numpy as np


class MissionMode(str, Enum):
    BOOTSTRAP = 'bootstrap'
    TRACK = 'track'
    GAP_AVOID = 'gap_avoid'
    SAFETY_BRAKE = 'safety_brake'
    RECOVERY = 'recovery'


@dataclass
class VehicleState:
    stamp: float = 0.0
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    speed: float = 0.0
    yaw_rate: float = 0.0
    steering_angle: float = 0.0
    linear_accel_x: float = 0.0
    valid: bool = False


@dataclass
class LidarObservation:
    stamp: float = 0.0
    forward_clearance: float = np.inf
    center_bias: float = 0.0
    gap_target_angle: float = 0.0
    lane_width_estimate: float = 0.0
    ttc: float = np.inf
    blocked: bool = False
    confidence: float = 0.0
    angles: np.ndarray = field(default_factory=lambda: np.asarray([], dtype=float))
    processed_ranges: np.ndarray = field(default_factory=lambda: np.asarray([], dtype=float))
    metadata: Dict[str, float] = field(default_factory=dict)


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
    desired_heading: float = 0.0
    curvature: float = 0.0
    lookahead: float = 0.0
    target_speed: float = 0.0
    forward_clearance: float = np.inf
    ttc: float = np.inf
    confidence: float = 0.0
    metadata: Dict[str, float] = field(default_factory=dict)


@dataclass
class ControlCommand:
    stamp: float = 0.0
    throttle: float = 0.0
    steering: float = 0.0
    emergency: bool = False
    reason: str = ''


@dataclass
class SensorHeartbeat:
    lidar_stamp: float = 0.0
    camera_stamp: float = 0.0
    imu_stamp: float = 0.0
    left_encoder_stamp: float = 0.0
    right_encoder_stamp: float = 0.0
    steering_stamp: float = 0.0

    def newest_encoder_stamp(self) -> float:
        return max(self.left_encoder_stamp, self.right_encoder_stamp)

    def as_dict(self) -> Dict[str, float]:
        return {
            'lidar': self.lidar_stamp,
            'camera': self.camera_stamp,
            'imu': self.imu_stamp,
            'left_encoder': self.left_encoder_stamp,
            'right_encoder': self.right_encoder_stamp,
            'steering': self.steering_stamp,
        }
