from __future__ import annotations

import hashlib
import math
from typing import Iterable, Optional, Tuple

import numpy as np


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def wrap_to_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def low_pass(previous: float, new_value: float, alpha: float) -> float:
    alpha = clamp(alpha, 0.0, 1.0)
    return alpha * new_value + (1.0 - alpha) * previous


def safe_mean(values: Iterable[float], default: float = 0.0) -> float:
    arr = np.asarray(list(values), dtype=float)
    arr = arr[np.isfinite(arr)]
    if arr.size == 0:
        return default
    return float(np.mean(arr))


def moving_average_1d(values: np.ndarray, kernel_size: int) -> np.ndarray:
    arr = np.asarray(values, dtype=float)
    if arr.size == 0 or kernel_size <= 1:
        return arr.copy()
    k = int(max(1, kernel_size))
    if k % 2 == 0:
        k += 1
    kernel = np.ones(k, dtype=float) / float(k)
    padded = np.pad(arr, (k // 2,), mode='edge')
    return np.convolve(padded, kernel, mode='valid')


def moving_average_points(points: np.ndarray, kernel_size: int) -> np.ndarray:
    pts = np.asarray(points, dtype=float)
    if pts.size == 0 or kernel_size <= 1:
        return pts.copy()
    out = pts.copy()
    out[:, 0] = moving_average_1d(out[:, 0], kernel_size)
    out[:, 1] = moving_average_1d(out[:, 1], kernel_size)
    return out


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def unwrap_delta(current: float, previous: float) -> float:
    delta = current - previous
    if delta > math.pi:
        delta -= 2.0 * math.pi
    elif delta < -math.pi:
        delta += 2.0 * math.pi
    return delta


def cumulative_arc_length(points: np.ndarray) -> np.ndarray:
    pts = np.asarray(points, dtype=float)
    if pts.shape[0] == 0:
        return np.asarray([], dtype=float)
    if pts.shape[0] == 1:
        return np.asarray([0.0], dtype=float)
    diffs = np.diff(pts, axis=0)
    seg = np.linalg.norm(diffs, axis=1)
    return np.concatenate(([0.0], np.cumsum(seg)))


def resample_polyline(points: np.ndarray, *, step: Optional[float] = None, count: Optional[int] = None) -> np.ndarray:
    pts = np.asarray(points, dtype=float)
    if pts.shape[0] <= 1:
        return pts.copy()
    s = cumulative_arc_length(pts)
    total = float(s[-1])
    if total <= 1.0e-9:
        return pts[[0]].copy()
    if count is not None:
        n = max(2, int(count))
        s_new = np.linspace(0.0, total, n)
    else:
        ds = max(1.0e-3, float(step or 0.1))
        n = max(2, int(math.floor(total / ds)) + 1)
        s_new = np.linspace(0.0, total, n)
    x_new = np.interp(s_new, s, pts[:, 0])
    y_new = np.interp(s_new, s, pts[:, 1])
    return np.column_stack((x_new, y_new)).astype(float)


def interpolate_path_value(path: np.ndarray, x_samples: np.ndarray) -> np.ndarray:
    pts = np.asarray(path, dtype=float)
    xs = np.asarray(x_samples, dtype=float)
    if pts.shape[0] < 2:
        return np.full_like(xs, np.nan, dtype=float)
    x = pts[:, 0]
    y = pts[:, 1]
    valid = np.isfinite(x) & np.isfinite(y)
    x = x[valid]
    y = y[valid]
    if x.size < 2:
        return np.full_like(xs, np.nan, dtype=float)
    order = np.argsort(x)
    x = x[order]
    y = y[order]
    y_interp = np.interp(xs, x, y, left=np.nan, right=np.nan)
    inside = (xs >= x[0]) & (xs <= x[-1])
    y_interp[~inside] = np.nan
    return y_interp


def nearest_point_index(points: np.ndarray, query_xy: np.ndarray) -> int:
    pts = np.asarray(points, dtype=float)
    if pts.shape[0] == 0:
        return 0
    query = np.asarray(query_xy, dtype=float).reshape(1, 2)
    d2 = np.sum((pts - query) ** 2, axis=1)
    return int(np.argmin(d2))


def transform_points_local_to_world(points: np.ndarray, pose) -> np.ndarray:
    pts = np.asarray(points, dtype=float)
    if pts.size == 0:
        return pts.copy()
    c = math.cos(float(pose.yaw))
    s = math.sin(float(pose.yaw))
    rot = np.asarray([[c, -s], [s, c]], dtype=float)
    transformed = pts @ rot.T
    transformed[:, 0] += float(pose.x)
    transformed[:, 1] += float(pose.y)
    return transformed


def transform_points_world_to_local(points: np.ndarray, pose) -> np.ndarray:
    pts = np.asarray(points, dtype=float)
    if pts.size == 0:
        return pts.copy()
    translated = pts.copy()
    translated[:, 0] -= float(pose.x)
    translated[:, 1] -= float(pose.y)
    c = math.cos(float(pose.yaw))
    s = math.sin(float(pose.yaw))
    rot_t = np.asarray([[c, s], [-s, c]], dtype=float)
    return translated @ rot_t.T


def headings_from_points(points: np.ndarray, closed: bool = False) -> np.ndarray:
    pts = np.asarray(points, dtype=float)
    if pts.shape[0] == 0:
        return np.asarray([], dtype=float)
    if pts.shape[0] == 1:
        return np.asarray([0.0], dtype=float)
    if closed and pts.shape[0] >= 3:
        wrap_pts = np.vstack((pts[-1], pts, pts[0]))
        dx = np.gradient(wrap_pts[:, 0])[1:-1]
        dy = np.gradient(wrap_pts[:, 1])[1:-1]
    else:
        dx = np.gradient(pts[:, 0])
        dy = np.gradient(pts[:, 1])
    return np.arctan2(dy, dx).astype(float)


def curvature_from_points(points: np.ndarray, closed: bool = False) -> np.ndarray:
    pts = np.asarray(points, dtype=float)
    n = pts.shape[0]
    if n < 3:
        return np.zeros((n,), dtype=float)
    if closed:
        wrap_pts = np.vstack((pts[-1], pts, pts[0]))
        x = wrap_pts[:, 0]
        y = wrap_pts[:, 1]
        dx = np.gradient(x)[1:-1]
        dy = np.gradient(y)[1:-1]
        ddx = np.gradient(np.gradient(x))[1:-1]
        ddy = np.gradient(np.gradient(y))[1:-1]
    else:
        x = pts[:, 0]
        y = pts[:, 1]
        dx = np.gradient(x)
        dy = np.gradient(y)
        ddx = np.gradient(dx)
        ddy = np.gradient(dy)
    denom = np.power(dx * dx + dy * dy, 1.5) + 1.0e-9
    return ((dx * ddy - dy * ddx) / denom).astype(float)


def pairwise_width_profile(left_boundary: np.ndarray, right_boundary: np.ndarray, x_grid: np.ndarray) -> np.ndarray:
    left_y = interpolate_path_value(left_boundary, x_grid)
    right_y = interpolate_path_value(right_boundary, x_grid)
    width = left_y - right_y
    width[~np.isfinite(width)] = np.nan
    return width


def voxel_downsample(points: np.ndarray, voxel_size: float, max_points: Optional[int] = None) -> np.ndarray:
    pts = np.asarray(points, dtype=float)
    if pts.shape[0] == 0:
        return np.zeros((0, 2), dtype=float)
    if voxel_size <= 1.0e-6:
        return pts[:max_points].copy() if max_points is not None else pts.copy()
    keys = np.floor(pts / float(voxel_size)).astype(np.int64)
    accum = {}
    counts = {}
    for key, point in zip(map(tuple, keys), pts):
        if key in accum:
            accum[key] += point
            counts[key] += 1
        else:
            accum[key] = point.copy()
            counts[key] = 1
    down = np.asarray([accum[k] / counts[k] for k in accum], dtype=float)
    if max_points is not None and down.shape[0] > max_points:
        order = np.argsort(down[:, 0])
        down = down[order]
        step = max(1, int(math.ceil(down.shape[0] / max_points)))
        down = down[::step][:max_points]
    return down


def hash_name_to_id(name: str) -> str:
    digest = hashlib.sha1(name.encode('utf-8')).hexdigest()[:12]
    safe = ''.join(ch.lower() if ch.isalnum() else '-' for ch in name.strip())
    safe = '-'.join(part for part in safe.split('-') if part)
    safe = safe[:32] if safe else 'track'
    return f'{safe}-{digest}'


def fingerprint_to_unit(vec: np.ndarray) -> np.ndarray:
    arr = np.asarray(vec, dtype=float).reshape(-1)
    arr[~np.isfinite(arr)] = 0.0
    norm = float(np.linalg.norm(arr))
    if norm <= 1.0e-9:
        return np.zeros_like(arr)
    return arr / norm


def cosine_similarity(a: np.ndarray, b: np.ndarray) -> float:
    ua = fingerprint_to_unit(a)
    ub = fingerprint_to_unit(b)
    if ua.size == 0 or ub.size == 0 or ua.size != ub.size:
        return 0.0
    return float(np.clip(np.dot(ua, ub), -1.0, 1.0))


def finite_difference_heading(y_profile: np.ndarray, x_profile: np.ndarray) -> np.ndarray:
    y = np.asarray(y_profile, dtype=float)
    x = np.asarray(x_profile, dtype=float)
    if y.size < 2:
        return np.zeros_like(y)
    dy = np.gradient(y)
    dx = np.gradient(x)
    dx = np.where(np.abs(dx) < 1.0e-6, 1.0e-6, dx)
    return np.arctan2(dy, dx)


def curvature_from_profile(y_profile: np.ndarray, x_profile: np.ndarray) -> np.ndarray:
    y = np.asarray(y_profile, dtype=float)
    x = np.asarray(x_profile, dtype=float)
    if y.size < 3:
        return np.zeros_like(y)
    dy = np.gradient(y, x)
    ddy = np.gradient(dy, x)
    denom = np.power(1.0 + dy * dy, 1.5) + 1.0e-9
    return ddy / denom
