"""
Enhanced Planning v2 for RoboRacer.

Key improvements:
- Smoother trajectory generation with better curvature handling
- Adaptive lookahead based on curvature and speed
- Better speed profiling with longitudinal dynamics
- Improved fallback behavior when primary path is unavailable
"""

from __future__ import annotations

import csv
import math
import os
from typing import List, Optional, Sequence, Tuple

import numpy as np

from .math_utils import (
    clamp,
    cumulative_arc_length,
    curvature_from_points,
    headings_from_points,
    moving_average_points,
    nearest_point_index,
    resample_polyline,
    transform_points_local_to_world,
    transform_points_world_to_local,
)
from .models import CameraObservation, LidarObservation, MissionMode, Plan, VehicleState, Waypoint
from .params_v2 import PlannerConfig


class TrajectoryPlannerV2:
    """Enhanced trajectory planner with smoother paths."""

    def __init__(self, config: PlannerConfig) -> None:
        self._config = config
        self._raceline_waypoints: List[Waypoint] = []
        self._raceline_points = np.zeros((0, 2), dtype=float)
        self._raceline_s = np.asarray([], dtype=float)
        if config.raceline_csv_path:
            self.load_raceline_csv(config.raceline_csv_path)
        
        # State for smooth transitions
        self._previous_curvature = 0.0
        self._previous_speed_profile = np.zeros(0)

    @property
    def has_raceline(self) -> bool:
        return len(self._raceline_waypoints) >= 2

    def load_raceline_csv(self, path: str) -> bool:
        if not path or not os.path.exists(path):
            return False

        try:
            data = np.genfromtxt(path, delimiter=',', names=True, dtype=float)
        except Exception:
            data = None

        if data is None or getattr(data, 'dtype', None) is None or data.dtype.names is None:
            try:
                points = np.loadtxt(path, delimiter=',', dtype=float)
            except Exception:
                return False
            if points.ndim == 1:
                points = points.reshape(1, -1)
            if points.shape[1] < 2:
                return False
            x = np.asarray(points[:, 0], dtype=float)
            y = np.asarray(points[:, 1], dtype=float)
            self._set_raceline_from_xy(np.column_stack((x, y)))
            return True

        names = set(data.dtype.names)
        if 'x' not in names or 'y' not in names:
            return False
        x = np.atleast_1d(np.asarray(data['x'], dtype=float))
        y = np.atleast_1d(np.asarray(data['y'], dtype=float))
        points = np.column_stack((x, y))
        s = np.atleast_1d(np.asarray(data['s'], dtype=float)) if 's' in names else None
        yaw = np.atleast_1d(np.asarray(data['yaw'], dtype=float)) if 'yaw' in names else None
        curvature = np.atleast_1d(np.asarray(data['curvature'], dtype=float)) if 'curvature' in names else None
        target_speed = np.atleast_1d(np.asarray(data['target_speed'], dtype=float)) if 'target_speed' in names else None
        width = np.atleast_1d(np.asarray(data['width'], dtype=float)) if 'width' in names else None
        self._set_raceline_from_xy(points, s=s, yaw=yaw, curvature=curvature, target_speed=target_speed, width=width)
        return True

    def plan(
        self,
        state: VehicleState,
        lidar: LidarObservation,
        camera: CameraObservation,
        mission_mode: MissionMode,
        stamp: float,
    ) -> Plan:
        if mission_mode == MissionMode.BOOTSTRAP:
            return Plan(stamp=stamp, mode=mission_mode)

        # Adaptive lookahead based on speed AND curvature
        base_lookahead = clamp(
            self._config.nominal_lookahead_m + self._config.lookahead_speed_gain * max(state.speed, 0.0),
            self._config.min_lookahead_m,
            self._config.max_lookahead_m,
        )
        
        # Reduce lookahead in high curvature sections
        if abs(lidar.curvature_hint) > 0.3:
            base_lookahead *= 0.8
        if abs(lidar.curvature_hint) > 0.5:
            base_lookahead *= 0.7
        
        lookahead = base_lookahead

        source = 'none'
        fallback_active = False
        waypoints: List[Waypoint] = []

        # Try raceline first
        use_raceline = self.has_raceline and state.valid
        if use_raceline and self._config.require_external_pose_for_raceline and state.source == 'wheel_odom':
            use_raceline = False
        if use_raceline and state.confidence < self._config.min_pose_confidence:
            use_raceline = False

        if use_raceline:
            waypoints = self._extract_global_window(state)
            source = 'raceline'
            if waypoints and lidar.has_centerline() and lidar.confidence >= self._config.min_track_confidence:
                if not self._corridor_is_valid(state, lidar, waypoints):
                    waypoints = []
                    source = 'none'

        # Fall back to local centerline
        if not waypoints and lidar.has_centerline() and lidar.confidence >= self._config.min_track_confidence:
            world_points = transform_points_local_to_world(lidar.centerline, state)
            waypoints = self._build_waypoints_from_points(world_points, closed=False, width=lidar.width_mean)
            source = 'local_centerline'

        # Last resort: gap following
        if not waypoints:
            fallback_heading = lidar.gap_target_angle
            if camera.confidence >= self._config.camera_confidence_threshold:
                camera_age = max(0.0, stamp - camera.stamp)
                freshness = self._camera_freshness(camera_age)
                camera_heading = camera.heading_error + self._config.camera_center_offset_gain * camera.center_offset
                blend = camera.confidence * camera.metadata.get('line_count', 0.0)
                _ = blend
                fallback_heading = (1.0 - 0.18 * freshness) * fallback_heading + 0.18 * freshness * camera_heading
            waypoints = self._build_gap_fallback(state, lidar, fallback_heading)
            source = 'gap_fallback'
            fallback_active = True

        # Compute desired heading from waypoints
        desired_heading = self._desired_heading_from_waypoints(state, waypoints)
        
        # Get reference curvature with smoothing
        raw_curvature = waypoints[min(len(waypoints) - 1, 2)].curvature if waypoints else 0.0
        reference_curvature = low_pass(self._previous_curvature, raw_curvature, 0.4)
        self._previous_curvature = reference_curvature
        
        # Speed profile
        reference_speed = self._reference_speed(waypoints)
        target_speed = self._apply_speed_envelope(reference_speed, lidar, mission_mode, fallback_active)
        
        # Mode-specific speed limits
        if mission_mode == MissionMode.LOCALIZE:
            target_speed = min(target_speed, self._config.localize_mode_speed_mps)
        if mission_mode == MissionMode.SAFETY_BRAKE:
            target_speed = 0.0
            
        # Emergency braking only when truly needed
        if lidar.ttc < 0.30 or lidar.forward_clearance < 0.25:
            target_speed = 0.0

        # Confidence calculation
        confidence = clamp(
            0.65 * lidar.confidence + 0.35 * state.confidence,
            0.0,
            1.0,
        )
        
        return Plan(
            stamp=stamp,
            mode=mission_mode,
            reference_source=source,
            waypoints=waypoints,
            desired_heading=float(desired_heading),
            curvature=float(reference_curvature),
            lookahead=float(lookahead),
            target_speed=float(target_speed),
            target_steering_angle=0.0,
            forward_clearance=float(lidar.forward_clearance),
            ttc=float(lidar.ttc),
            blocked=bool(lidar.blocked),
            fallback_active=bool(fallback_active),
            confidence=float(confidence),
            metadata={
                'path_point_count': float(len(waypoints)),
                'reference_speed': float(reference_speed),
                'pose_source': state.source,
            },
        )

    def _camera_freshness(self, camera_age: float) -> float:
        stale_after = max(self._config.camera_stale_after_s, 1e-3)
        full_decay = max(self._config.camera_stale_full_decay_s, stale_after + 1e-3)
        if camera_age <= stale_after:
            return 1.0
        if camera_age >= full_decay:
            return 0.0
        return 1.0 - (camera_age - stale_after) / (full_decay - stale_after)

    def _set_raceline_from_xy(
        self,
        points: np.ndarray,
        *,
        s: Optional[np.ndarray] = None,
        yaw: Optional[np.ndarray] = None,
        curvature: Optional[np.ndarray] = None,
        target_speed: Optional[np.ndarray] = None,
        width: Optional[np.ndarray] = None,
    ) -> None:
        if points.ndim != 2 or points.shape[1] < 2:
            return
        points = points[:, :2].astype(float)
        points = points[np.isfinite(points).all(axis=1)]
        if points.shape[0] < 2:
            return
        if self._config.raceline_closed_loop and np.linalg.norm(points[0] - points[-1]) > 0.20:
            points = np.vstack([points, points[0]])
        points = moving_average_points(points, 5)
        points = resample_polyline(points, count=max(self._config.local_horizon_points * 3, points.shape[0]))
        if s is None or len(np.atleast_1d(s)) != points.shape[0]:
            s = cumulative_arc_length(points)
        if yaw is None or len(np.atleast_1d(yaw)) != points.shape[0]:
            yaw = headings_from_points(points, closed=self._config.raceline_closed_loop)
        if curvature is None or len(np.atleast_1d(curvature)) != points.shape[0]:
            curvature = curvature_from_points(points, closed=self._config.raceline_closed_loop)
        if target_speed is None or len(np.atleast_1d(target_speed)) != points.shape[0]:
            target_speed = self._speed_profile_from_curvature(np.asarray(curvature, dtype=float), np.asarray(s, dtype=float))
        if width is None or len(np.atleast_1d(width)) != points.shape[0]:
            width = np.full((points.shape[0],), 0.0, dtype=float)

        self._raceline_points = points
        self._raceline_s = np.asarray(s, dtype=float)
        self._raceline_waypoints = [
            Waypoint(
                s=float(self._raceline_s[i]),
                x=float(points[i, 0]),
                y=float(points[i, 1]),
                yaw=float(np.asarray(yaw, dtype=float)[i]),
                curvature=float(np.asarray(curvature, dtype=float)[i]),
                target_speed=float(np.asarray(target_speed, dtype=float)[i]),
                width=float(np.asarray(width, dtype=float)[i]),
            )
            for i in range(points.shape[0])
        ]

    def _build_waypoints_from_points(self, points: np.ndarray, *, closed: bool, width: float = 0.0) -> List[Waypoint]:
        if points.shape[0] < 2:
            return []
        points = moving_average_points(points.astype(float), 5)
        points = resample_polyline(points, count=max(self._config.local_horizon_points, points.shape[0]))
        s = cumulative_arc_length(points)
        yaw = headings_from_points(points, closed=closed)
        curvature = curvature_from_points(points, closed=closed)
        target_speed = self._speed_profile_from_curvature(curvature, s)
        return [
            Waypoint(
                s=float(s[i]),
                x=float(points[i, 0]),
                y=float(points[i, 1]),
                yaw=float(yaw[i]),
                curvature=float(curvature[i]),
                target_speed=float(target_speed[i]),
                width=float(width),
            )
            for i in range(points.shape[0])
        ]

    def _speed_profile_from_curvature(self, curvature: np.ndarray, s: np.ndarray) -> np.ndarray:
        n = curvature.size
        if n == 0:
            return np.asarray([], dtype=float)
        speeds = np.full((n,), self._config.max_speed_mps, dtype=float)
        mask = np.abs(curvature) > 1e-4
        speeds[mask] = np.sqrt(
            np.maximum(self._config.lateral_accel_limit_mps2 / np.abs(curvature[mask]), 0.0)
        )
        speeds = np.clip(speeds, self._config.min_speed_mps, self._config.max_speed_mps)

        # Backward pass for braking
        for idx in range(n - 2, -1, -1):
            ds = max(float(s[idx + 1] - s[idx]), 1e-3)
            braking_limit = math.sqrt(max(speeds[idx + 1] ** 2 + 2.0 * self._config.max_brake_decel_mps2 * ds, 0.0))
            speeds[idx] = min(speeds[idx], braking_limit)

        # Forward pass for acceleration
        for idx in range(1, n):
            ds = max(float(s[idx] - s[idx - 1]), 1e-3)
            accel_limit = math.sqrt(max(speeds[idx - 1] ** 2 + 2.0 * self._config.max_accel_mps2 * ds, 0.0))
            speeds[idx] = min(speeds[idx], accel_limit)
            
        # Smooth the speed profile
        speeds = moving_average_1d(speeds, 5)
        return speeds

    def _extract_global_window(self, state: VehicleState) -> List[Waypoint]:
        if not self.has_raceline:
            return []
        current_xy = np.asarray([state.x, state.y], dtype=float)
        idx = nearest_point_index(self._raceline_points, current_xy)
        n = len(self._raceline_waypoints)
        horizon = max(self._config.local_horizon_m, self._config.max_lookahead_m + 1.0)
        indices = [idx]
        traversed = 0.0
        cursor = idx
        while traversed < horizon and len(indices) < self._config.local_horizon_points:
            nxt = (cursor + 1) % n if self._config.raceline_closed_loop else min(cursor + 1, n - 1)
            if nxt == cursor:
                break
            p0 = self._raceline_points[cursor]
            p1 = self._raceline_points[nxt]
            traversed += float(np.linalg.norm(p1 - p0))
            indices.append(nxt)
            cursor = nxt
            if not self._config.raceline_closed_loop and cursor == n - 1:
                break
        return [self._raceline_waypoints[i] for i in indices]

    def _corridor_is_valid(
        self,
        state: VehicleState,
        lidar: LidarObservation,
        waypoints: Sequence[Waypoint],
    ) -> bool:
        if not lidar.has_centerline() or not waypoints:
            return True
        world_points = np.asarray([[wp.x, wp.y] for wp in waypoints], dtype=float)
        local_points = transform_points_world_to_local(world_points, state)
        overlap = (
            (local_points[:, 0] >= lidar.centerline[0, 0])
            & (local_points[:, 0] <= lidar.centerline[-1, 0])
        )
        if np.count_nonzero(overlap) < 3:
            return True
        ref_y = np.interp(local_points[overlap, 0], lidar.centerline[:, 0], lidar.centerline[:, 1])
        error = np.median(np.abs(ref_y - local_points[overlap, 1]))
        return bool(error <= self._config.corridor_validation_error_m)

    def _build_gap_fallback(
        self,
        state: VehicleState,
        lidar: LidarObservation,
        target_heading: float,
    ) -> List[Waypoint]:
        horizon = clamp(
            min(self._config.local_horizon_m, max(2.0, lidar.forward_clearance)),
            2.0,
            self._config.local_horizon_m,
        )
        s_vals = np.linspace(0.30, horizon, self._config.local_horizon_points)
        
        # Curvature from target heading: positive heading = turn left (positive curvature)
        # Using circular arc geometry: curvature = 2*sin(theta)/lookahead for small angles
        lookahead = max(self._config.nominal_lookahead_m, 0.75)
        curvature = 2.0 * math.sin(target_heading) / lookahead
        
        # Generate circular arc trajectory
        if abs(curvature) < 1e-4:
            # Nearly straight path
            x_local = s_vals
            y_local = np.tan(target_heading) * s_vals
        else:
            # Circular arc: x = sin(k*s)/k, y = (1-cos(k*s))/k for positive curvature (left turn)
            radius = 1.0 / abs(curvature)
            if curvature > 0:
                # Left turn: arc curves upward (positive y)
                x_local = radius * np.sin(s_vals / radius)
                y_local = radius * (1.0 - np.cos(s_vals / radius))
            else:
                # Right turn: arc curves downward (negative y)
                x_local = radius * np.sin(s_vals / radius)
                y_local = -radius * (1.0 - np.cos(s_vals / radius))
        
        # Add bias based on lane position
        y_local += 0.15 * lidar.center_bias * (s_vals / max(horizon, 1e-3))
        local_points = np.column_stack((x_local, y_local)).astype(float)
        world_points = transform_points_local_to_world(local_points, state)
        return self._build_waypoints_from_points(world_points, closed=False, width=lidar.width_mean)

    def _desired_heading_from_waypoints(self, state: VehicleState, waypoints: Sequence[Waypoint]) -> float:
        if len(waypoints) < 2:
            return 0.0
        first = np.asarray([waypoints[0].x, waypoints[0].y], dtype=float)
        second = np.asarray([waypoints[min(2, len(waypoints) - 1)].x, waypoints[min(2, len(waypoints) - 1)].y], dtype=float)
        local = transform_points_world_to_local(np.vstack([first, second]), state)
        dx = float(local[1, 0] - local[0, 0])
        dy = float(local[1, 1] - local[0, 1])
        return float(clamp(math.atan2(dy, max(dx, 1e-3)), -0.85, 0.85))

    def _reference_speed(self, waypoints: Sequence[Waypoint]) -> float:
        if not waypoints:
            return 0.0
        window = waypoints[: min(6, len(waypoints))]
        return float(min(wp.target_speed for wp in window))

    def _apply_speed_envelope(
        self,
        reference_speed: float,
        lidar: LidarObservation,
        mission_mode: MissionMode,
        fallback_active: bool,
    ) -> float:
        target_speed = float(reference_speed)
        target_speed = min(target_speed, self._config.max_speed_mps)
        
        # Clearance-based speed modulation
        clearance_speed = self._config.clearance_speed_gain * max(lidar.forward_clearance - 0.35, 0.0)
        target_speed = min(target_speed, clearance_speed)

        # Risk-based speed reduction
        clearance_risk = clamp((0.80 - lidar.forward_clearance) / 0.45, 0.0, 1.0)
        ttc_risk = 0.0 if lidar.ttc == float('inf') else clamp((1.20 - lidar.ttc) / 0.75, 0.0, 1.0)
        risk = max(clearance_risk, ttc_risk)
        target_speed *= clamp(1.0 - 0.85 * (risk ** 1.5), 0.15, 1.0)

        # Narrow track slowdown
        if lidar.width_mean > 0.0 and lidar.width_mean < self._config.narrow_width_slowdown_m:
            target_speed *= 0.75
            
        # Mode-specific limits
        if mission_mode == MissionMode.AVOID or fallback_active:
            target_speed = min(target_speed, self._config.avoid_mode_speed_mps)
        if mission_mode == MissionMode.SAFETY_BRAKE:
            target_speed = 0.0
            
        return float(clamp(target_speed, self._config.min_speed_mps, self._config.max_speed_mps))


def low_pass(previous: float, new_value: float, alpha: float) -> float:
    """Simple low-pass filter."""
    alpha = clamp(alpha, 0.0, 1.0)
    return alpha * new_value + (1.0 - alpha) * previous


def moving_average_1d(values: np.ndarray, kernel_size: int) -> np.ndarray:
    """1D moving average."""
    if kernel_size <= 1 or values.size == 0:
        return values.copy()
    kernel_size = int(max(1, kernel_size))
    if kernel_size % 2 == 0:
        kernel_size += 1
    kernel = np.ones(kernel_size, dtype=float) / float(kernel_size)
    padded = np.pad(values, (kernel_size // 2,), mode='edge')
    return np.convolve(padded, kernel, mode='valid')


# Alias for compatibility
ReactivePlannerV2 = TrajectoryPlannerV2
