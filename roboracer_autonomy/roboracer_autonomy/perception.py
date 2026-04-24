from __future__ import annotations

import math
from typing import List, Optional, Tuple

import numpy as np

try:
    import cv2
except ImportError:  # pragma: no cover - OpenCV is available in the competition image.
    cv2 = None

from .math_utils import clamp, moving_average_1d
from .models import CameraObservation, LidarObservation
from .params import CameraConfig, LidarConfig


class LidarPerception:
    def __init__(self, config: LidarConfig) -> None:
        self._config = config
        self._previous_gap_target_angle: float = 0.0

    def process(
        self,
        ranges: np.ndarray,
        angle_min: float,
        angle_increment: float,
        speed_mps: float,
        stamp: float,
    ) -> LidarObservation:
        if ranges.size == 0:
            return LidarObservation(stamp=stamp)

        processed = np.asarray(ranges, dtype=float)
        finite_mask = np.isfinite(processed)
        processed[~finite_mask] = self._config.range_max_clip_m
        processed = np.clip(processed, 0.0, self._config.range_max_clip_m)
        processed = moving_average_1d(processed, self._config.smoothing_kernel)
        processed = self._close_small_leaks(processed)

        angles = angle_min + np.arange(processed.size, dtype=float) * angle_increment
        focus_mask = np.abs(np.degrees(angles)) <= self._config.focus_half_angle_deg
        focus_ranges = processed.copy()
        focus_ranges[~focus_mask] = 0.0

        forward_mask = np.abs(np.degrees(angles)) <= self._config.forward_sector_deg
        forward_sector = processed[forward_mask]
        if forward_sector.size == 0:
            forward_clearance = self._config.range_max_clip_m
        else:
            forward_clearance = float(np.percentile(forward_sector, 20))

        nearest_index = int(np.argmin(np.where(focus_ranges > 0.0, focus_ranges, np.inf)))
        nearest_range = float(focus_ranges[nearest_index]) if np.isfinite(focus_ranges[nearest_index]) else self._config.range_max_clip_m
        bubble_bins = self._bubble_bin_count(nearest_range, angle_increment)
        gap_ranges = focus_ranges.copy()
        start = max(0, nearest_index - bubble_bins)
        end = min(gap_ranges.size, nearest_index + bubble_bins + 1)
        gap_ranges[start:end] = 0.0

        gap_start, gap_end = self._largest_gap(gap_ranges > self._config.min_free_distance_m)
        if gap_start is None or gap_end is None:
            target_index = int(np.argmax(gap_ranges))
            gap_start = target_index
            gap_end = target_index
        else:
            candidate_indices = np.arange(gap_start, gap_end + 1)
            candidate_angles = angles[candidate_indices]
            continuity_penalty = np.abs(candidate_angles - self._previous_gap_target_angle)
            score = (
                gap_ranges[candidate_indices]
                - 0.45 * np.abs(candidate_angles)
                - self._config.gap_continuity_weight * continuity_penalty
            )
            target_index = int(candidate_indices[np.argmax(score)])

        gap_target_angle = float(angles[target_index])
        self._previous_gap_target_angle = gap_target_angle
        lane_width_estimate, center_bias = self._estimate_corridor_bias(processed, angles)
        ttc = float(np.inf)
        if speed_mps > 0.15:
            ttc = forward_clearance / max(speed_mps, 1e-3)

        blocked = (
            forward_clearance < self._config.stop_distance_m
            or ttc < self._config.caution_ttc_s
        )
        confidence = float(
            clamp(
                0.45
                + 0.25 * min(1.0, forward_clearance / max(self._config.stop_distance_m, 1e-3))
                + 0.30 * min(1.0, lane_width_estimate / max(self._config.lane_width_confident_m, 1e-3)),
                0.0,
                1.0,
            )
        )

        return LidarObservation(
            stamp=stamp,
            forward_clearance=forward_clearance,
            center_bias=center_bias,
            gap_target_angle=gap_target_angle,
            lane_width_estimate=lane_width_estimate,
            ttc=ttc,
            blocked=blocked,
            confidence=confidence,
            angles=angles,
            processed_ranges=processed,
            metadata={
                'nearest_range': nearest_range,
                'gap_start_angle': float(angles[gap_start]),
                'gap_end_angle': float(angles[gap_end]),
            },
        )

    def _bubble_bin_count(self, nearest_range: float, angle_increment: float) -> int:
        safe_range = max(nearest_range, 0.10)
        bubble_angle = math.atan2(self._config.bubble_radius_m, safe_range)
        return max(1, int(bubble_angle / max(angle_increment, 1e-4)))

    def _close_small_leaks(self, ranges: np.ndarray) -> np.ndarray:
        fixed = ranges.copy()
        threshold = self._config.range_max_clip_m * 0.92
        i = 0
        n = fixed.size
        while i < n:
            if fixed[i] < threshold:
                i += 1
                continue
            j = i
            while j < n and fixed[j] >= threshold:
                j += 1
            run_length = j - i
            if (
                0 < i < n - 1
                and 0 < run_length <= self._config.leak_fill_max_bins
                and j < n
                and fixed[i - 1] < threshold
                and fixed[j] < threshold
            ):
                fill_value = min(fixed[i - 1], fixed[j])
                fixed[i:j] = fill_value
            i = j
        return fixed

    def _largest_gap(self, free_mask: np.ndarray) -> Tuple[Optional[int], Optional[int]]:
        best_start: Optional[int] = None
        best_end: Optional[int] = None
        best_length = -1
        start = None
        for idx, is_free in enumerate(free_mask):
            if is_free and start is None:
                start = idx
            if (not is_free or idx == free_mask.size - 1) and start is not None:
                end = idx if is_free and idx == free_mask.size - 1 else idx - 1
                length = end - start + 1
                if length > best_length:
                    best_start = start
                    best_end = end
                    best_length = length
                start = None
        return best_start, best_end

    def _estimate_corridor_bias(self, ranges: np.ndarray, angles: np.ndarray) -> Tuple[float, float]:
        left_mask = (np.degrees(angles) >= 60.0) & (np.degrees(angles) <= 100.0)
        right_mask = (np.degrees(angles) <= -60.0) & (np.degrees(angles) >= -100.0)
        left = ranges[left_mask]
        right = ranges[right_mask]

        left = left[np.isfinite(left)]
        right = right[np.isfinite(right)]
        if left.size == 0 or right.size == 0:
            return 0.0, 0.0

        left_dist = float(np.percentile(left, 35))
        right_dist = float(np.percentile(right, 35))
        lane_width_estimate = left_dist + right_dist
        center_bias = math.atan2(left_dist - right_dist, max(0.80, 0.5 * lane_width_estimate))
        center_bias = clamp(center_bias, -0.60, 0.60)
        return lane_width_estimate, float(center_bias)


class CameraPerception:
    def __init__(self, config: CameraConfig) -> None:
        self._config = config

    def process(self, rgb_image: np.ndarray, stamp: float) -> CameraObservation:
        if not self._config.enabled or cv2 is None or rgb_image.size == 0:
            return CameraObservation(stamp=stamp)

        height, width = rgb_image.shape[:2]
        roi_start = int(clamp(self._config.roi_vertical_start, 0.0, 0.95) * height)
        roi = rgb_image[roi_start:height, :, :]
        if roi.size == 0:
            return CameraObservation(stamp=stamp)

        gray = cv2.cvtColor(roi, cv2.COLOR_RGB2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(gray, self._config.canny_low, self._config.canny_high)
        lines = cv2.HoughLinesP(
            edges,
            1,
            np.pi / 180.0,
            threshold=self._config.hough_threshold,
            minLineLength=self._config.hough_min_line_length,
            maxLineGap=self._config.hough_max_line_gap,
        )

        if lines is None:
            return CameraObservation(stamp=stamp)

        left_points: List[Tuple[int, int]] = []
        right_points: List[Tuple[int, int]] = []
        for line in lines[:, 0, :]:
            x1, y1, x2, y2 = [int(v) for v in line]
            if x2 == x1:
                continue
            slope = (y2 - y1) / float(x2 - x1)
            if abs(slope) < self._config.min_slope_abs:
                continue
            if slope < 0:
                left_points.extend([(x1, y1), (x2, y2)])
            else:
                right_points.extend([(x1, y1), (x2, y2)])

        left_line = self._fit_line(left_points)
        right_line = self._fit_line(right_points)
        has_left = left_line is not None
        has_right = right_line is not None
        if not has_left and not has_right:
            return CameraObservation(stamp=stamp)

        bottom_y = roi.shape[0] - 1
        upper_y = int(roi.shape[0] * 0.35)
        left_x_bottom = self._line_x(left_line, bottom_y) if left_line else None
        right_x_bottom = self._line_x(right_line, bottom_y) if right_line else None
        left_x_upper = self._line_x(left_line, upper_y) if left_line else None
        right_x_upper = self._line_x(right_line, upper_y) if right_line else None

        if left_x_bottom is not None and right_x_bottom is not None:
            center_bottom = 0.5 * (left_x_bottom + right_x_bottom)
            center_upper = 0.5 * (left_x_upper + right_x_upper)
        elif left_x_bottom is not None:
            # One-sided fallback. Confidence remains modest.
            center_bottom = left_x_bottom + 0.35 * width
            center_upper = (left_x_upper if left_x_upper is not None else left_x_bottom) + 0.25 * width
        else:
            center_bottom = right_x_bottom - 0.35 * width
            center_upper = (right_x_upper if right_x_upper is not None else right_x_bottom) - 0.25 * width

        center_offset = (center_bottom - 0.5 * width) / max(0.5 * width, 1.0)
        heading_error = math.atan2(center_upper - 0.5 * width, max(roi.shape[0], 1))
        heading_error = clamp(heading_error * self._config.heading_scale_rad, -0.55, 0.55)

        confidence = 0.20
        confidence += 0.30 if has_left else 0.0
        confidence += 0.30 if has_right else 0.0
        confidence += 0.20 * min(1.0, len(lines) / 14.0)
        confidence = float(clamp(confidence, 0.0, 1.0))

        return CameraObservation(
            stamp=stamp,
            center_offset=float(center_offset),
            heading_error=float(heading_error),
            confidence=confidence,
            has_left_boundary=has_left,
            has_right_boundary=has_right,
            metadata={
                'line_count': float(len(lines)),
                'roi_start': float(roi_start),
            },
        )

    def _fit_line(self, points: List[Tuple[int, int]]) -> Optional[Tuple[float, float]]:
        if len(points) < 4:
            return None
        pts = np.asarray(points, dtype=float)
        ys = pts[:, 1]
        xs = pts[:, 0]
        slope, intercept = np.polyfit(ys, xs, 1)
        return float(slope), float(intercept)

    def _line_x(self, line: Optional[Tuple[float, float]], y: float) -> Optional[float]:
        if line is None:
            return None
        slope, intercept = line
        return slope * y + intercept
