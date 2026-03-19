from __future__ import annotations

import math
from typing import Iterable

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


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)
