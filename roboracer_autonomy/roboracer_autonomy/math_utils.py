from __future__ import annotations

import math
from typing import Iterable, Optional

import numpy as np


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def wrap_to_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def safe_mean(values: Iterable[float], default: float = 0.0) -> float:
    arr = np.asarray(list(values), dtype=float)
    arr = arr[np.isfinite(arr)]
    if arr.size == 0:
        return default
    return float(np.mean(arr))


def low_pass(previous: float, new_value: float, alpha: float) -> float:
    alpha = clamp(alpha, 0.0, 1.0)
    return alpha * new_value + (1.0 - alpha) * previous


def unwrap_delta(current: float, previous: float) -> float:
    delta = current - previous
    if delta > math.pi:
        delta -= 2.0 * math.pi
    elif delta < -math.pi:
        delta += 2.0 * math.pi
    return delta


def moving_average_1d(values: np.ndarray, kernel_size: int) -> np.ndarray:
    if kernel_size <= 1 or values.size == 0:
        return values.copy()
    kernel_size = int(max(1, kernel_size))
    if kernel_size % 2 == 0:
        kernel_size += 1
    kernel = np.ones(kernel_size, dtype=float) / float(kernel_size)
    padded = np.pad(values, (kernel_size // 2,), mode='edge')
    return np.convolve(padded, kernel, mode='valid')


def moving_average_points(points: np.ndarray, kernel_size: int) -> np.ndarray:
    if points.size == 0 or kernel_size <= 1:
        return points.copy()
    smoothed = points.copy()
    smoothed[:, 0] = moving_average_1d(points[:, 0], kernel_size)
    smoothed[:, 1] = moving_average_1d(points[:, 1], kernel_size)
    return smoothed


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def cumulative_arc_length(points: np.ndarray) -> np.ndarray:
    if points.shape[0] == 0:
        return np.asarray([], dtype=float)
    if points.shape[0] == 1:
        return np.asarray([0.0], dtype=float)
    diffs = np.diff(points, axis=0)
    seg = np.linalg.norm(diffs, axis=1)
    return np.concatenate(([0.0], np.cumsum(seg)))


def headings_from_points(points: np.ndarray, closed: bool = False) -> np.ndarray:
    n = points.shape[0]
    if n == 0:
        return np.asarray([], dtype=float)
    if n == 1:
        return np.asarray([0.0], dtype=float)
    pts = points
    if closed and np.linalg.norm(points[0] - points[-1]) > 1e-6:
        pts = np.vstack([points, points[0]])
    dx = np.gradient(pts[:, 0])
    dy = np.gradient(pts[:, 1])
    yaw = np.arctan2(dy, dx)
    if pts.shape[0] != n:
        yaw = yaw[:-1]
    return yaw.astype(float)


def curvature_from_points(points: np.ndarray, closed: bool = False) -> np.ndarray:
    n = points.shape[0]
    if n < 3:
        return np.zeros((n,), dtype=float)
    pts = points
    if closed and np.linalg.norm(points[0] - points[-1]) > 1e-6:
        pts = np.vstack([points[-1], points, points[0]])
    x = pts[:, 0]
    y = pts[:, 1]
    dx = np.gradient(x)
    dy = np.gradient(y)
    ddx = np.gradient(dx)
    ddy = np.gradient(dy)
    denom = np.power(dx * dx + dy * dy, 1.5) + 1e-9
    curvature = (dx * ddy - dy * ddx) / denom
    if pts.shape[0] != n:
        curvature = curvature[1:-1]
    return curvature.astype(float)


def resample_polyline(
    points: np.ndarray,
    *,
    step: Optional[float] = None,
    count: Optional[int] = None,
) -> np.ndarray:
    if points.shape[0] <= 1:
        return points.copy()
    s = cumulative_arc_length(points)
    total = float(s[-1])
    if total <= 1e-9:
        return points[[0]].copy()
    if count is not None:
        count = max(2, int(count))
        s_new = np.linspace(0.0, total, count)
    else:
        step = max(1e-3, float(step or 0.1))
        n = max(2, int(math.floor(total / step)) + 1)
        s_new = np.linspace(0.0, total, n)
    x_new = np.interp(s_new, s, points[:, 0])
    y_new = np.interp(s_new, s, points[:, 1])
    return np.column_stack((x_new, y_new)).astype(float)


def nearest_point_index(points: np.ndarray, query_xy: np.ndarray) -> int:
    if points.shape[0] == 0:
        return 0
    diff = points - np.asarray(query_xy, dtype=float).reshape(1, 2)
    dist2 = np.sum(diff * diff, axis=1)
    return int(np.argmin(dist2))


def transform_points_local_to_world(points: np.ndarray, pose) -> np.ndarray:
    if points.size == 0:
        return points.copy()
    c = math.cos(float(pose.yaw))
    s = math.sin(float(pose.yaw))
    rot = np.asarray([[c, -s], [s, c]], dtype=float)
    transformed = points @ rot.T
    transformed[:, 0] += float(pose.x)
    transformed[:, 1] += float(pose.y)
    return transformed


def transform_points_world_to_local(points: np.ndarray, pose) -> np.ndarray:
    if points.size == 0:
        return points.copy()
    translated = points.copy().astype(float)
    translated[:, 0] -= float(pose.x)
    translated[:, 1] -= float(pose.y)
    c = math.cos(float(pose.yaw))
    s = math.sin(float(pose.yaw))
    rot_t = np.asarray([[c, s], [-s, c]], dtype=float)
    return translated @ rot_t.T


def interpolate_path_value(path: np.ndarray, x_samples: np.ndarray) -> np.ndarray:
    if path.shape[0] < 2:
        return np.full_like(x_samples, np.nan, dtype=float)
    x = path[:, 0]
    y = path[:, 1]
    valid = np.isfinite(x) & np.isfinite(y)
    x = x[valid]
    y = y[valid]
    if x.size < 2:
        return np.full_like(x_samples, np.nan, dtype=float)
    order = np.argsort(x)
    x = x[order]
    y = y[order]
    values = np.interp(x_samples, x, y, left=np.nan, right=np.nan)
    inside = (x_samples >= x[0]) & (x_samples <= x[-1])
    values[~inside] = np.nan
    return values
