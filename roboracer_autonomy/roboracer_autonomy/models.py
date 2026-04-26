from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional

import numpy as np


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

    def pose_xy(self) -> np.ndarray:
        return np.asarray([self.x, self.y], dtype=float)


@dataclass
class TrackBoundaries:
    stamp: float = 0.0
    left_boundary: np.ndarray = field(default_factory=lambda: np.zeros((0, 2), dtype=float))
    right_boundary: np.ndarray = field(default_factory=lambda: np.zeros((0, 2), dtype=float))
    centerline: np.ndarray = field(default_factory=lambda: np.zeros((0, 2), dtype=float))
    width_mean: float = 0.0
    width_min: float = 0.0
    width_std: float = 0.0
    heading_hint: float = 0.0
    curvature_hint: float = 0.0
    forward_clearance: float = np.inf
    ttc: float = np.inf
    blocked: bool = False
    confidence: float = 0.0
    processed_ranges: np.ndarray = field(default_factory=lambda: np.asarray([], dtype=float))
    angles: np.ndarray = field(default_factory=lambda: np.asarray([], dtype=float))
    metadata: Dict[str, float] = field(default_factory=dict)

    def has_corridor(self) -> bool:
        return self.left_boundary.shape[0] >= 2 or self.right_boundary.shape[0] >= 2


@dataclass
class ControlCommand:
    stamp: float = 0.0
    throttle: float = 0.0
    steering: float = 0.0
    target_speed: float = 0.0
    emergency: bool = False
    reason: str = ''
    metadata: Dict[str, float] = field(default_factory=dict)


@dataclass
class MapLocalizerOutput:
    pose: VehicleState = field(default_factory=VehicleState)
    corrected: bool = False
    track_id: str = ''
    map_confidence: float = 0.0
    metadata: Dict[str, float] = field(default_factory=dict)


@dataclass
class TrackMapRecord:
    track_id: str
    display_name: str
    fingerprint: np.ndarray = field(default_factory=lambda: np.asarray([], dtype=float))
    left_points: np.ndarray = field(default_factory=lambda: np.zeros((0, 2), dtype=float))
    right_points: np.ndarray = field(default_factory=lambda: np.zeros((0, 2), dtype=float))
    centerline_points: np.ndarray = field(default_factory=lambda: np.zeros((0, 2), dtype=float))
    created_at: float = 0.0
    updated_at: float = 0.0
    last_used_at: float = 0.0
    updates: int = 0
    laps_observed: int = 0
    metadata: Dict[str, float] = field(default_factory=dict)


@dataclass
class SolverDebug:
    success: bool = False
    cost: float = 0.0
    solve_time_ms: float = 0.0
    iterations: int = 0
    target_speed: float = 0.0
    progress_m: float = 0.0
    predicted_path: np.ndarray = field(default_factory=lambda: np.zeros((0, 2), dtype=float))


@dataclass
class SensorHeartbeat:
    lidar_stamp: float = 0.0
    imu_stamp: float = 0.0
    left_encoder_stamp: float = 0.0
    right_encoder_stamp: float = 0.0
    steering_stamp: float = 0.0
    external_pose_stamp: float = 0.0

    def newest_encoder_stamp(self) -> float:
        return max(self.left_encoder_stamp, self.right_encoder_stamp)

    def as_dict(self) -> Dict[str, float]:
        return {
            'lidar': self.lidar_stamp,
            'imu': self.imu_stamp,
            'left_encoder': self.left_encoder_stamp,
            'right_encoder': self.right_encoder_stamp,
            'steering': self.steering_stamp,
            'external_pose': self.external_pose_stamp,
        }
