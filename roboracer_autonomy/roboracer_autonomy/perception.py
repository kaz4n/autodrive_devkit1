from __future__ import annotations

import math
from typing import List, Optional, Tuple

import numpy as np

from .math_utils import clamp, interpolate_path_value, low_pass, moving_average_1d
from .models import TrackBoundaries
from .params import LidarConfig


class GapDebug:
    """Optional follow-the-gap debug signal kept outside the control loop."""

    def __init__(self, config: LidarConfig) -> None:
        self._config = config
        self._previous_target_angle: float = 0.0

    def compute(self, processed_ranges: np.ndarray, angles: np.ndarray) -> Tuple[float, dict]:
        if processed_ranges.size == 0:
            return 0.0, {}
        focus_mask = np.abs(np.degrees(angles)) <= self._config.focus_half_angle_deg
        focus_ranges = processed_ranges.copy()
        focus_ranges[~focus_mask] = 0.0
        finite_positive = np.where(focus_ranges > 0.0, focus_ranges, np.inf)
        nearest_index = int(np.argmin(finite_positive))
        nearest_range = float(finite_positive[nearest_index]) if np.isfinite(finite_positive[nearest_index]) else self._config.range_max_clip_m
        if angles.size >= 2:
            angle_increment = float(abs(angles[1] - angles[0]))
        else:
            angle_increment = math.radians(0.25)
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
            score = gap_ranges[candidate_indices] - 0.45 * np.abs(angles[candidate_indices])
            score -= self._config.gap_continuity_weight * np.abs(angles[candidate_indices] - self._previous_target_angle)
            target_index = int(candidate_indices[np.argmax(score)])
        target_angle = float(angles[target_index])
        self._previous_target_angle = low_pass(self._previous_target_angle, target_angle, 0.40)
        return self._previous_target_angle, {
            'debug_gap_target_angle': float(self._previous_target_angle),
            'debug_gap_nearest_range': nearest_range,
            'debug_gap_start_angle': float(angles[gap_start]),
            'debug_gap_end_angle': float(angles[gap_end]),
        }

    def _bubble_bin_count(self, nearest_range: float, angle_increment: float) -> int:
        safe_range = max(nearest_range, 0.10)
        bubble_angle = math.atan2(self._config.bubble_radius_m, safe_range)
        return max(1, int(bubble_angle / max(angle_increment, 1.0e-4)))

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


class LidarTrackExtractor:
    """Extracts a corridor directly from the live LiDAR scan.

    The nominal controller only consumes left/right boundaries in vehicle frame.
    The centerline is still published as a lightweight debug signal.
    """

    def __init__(self, config: LidarConfig) -> None:
        self._config = config
        self._gap_debug = GapDebug(config)
        self._width_estimate = config.nominal_track_width_m

    def process(self, ranges: np.ndarray, angle_min: float, angle_increment: float, speed_mps: float, stamp: float) -> TrackBoundaries:
        if ranges.size == 0:
            return TrackBoundaries(stamp=stamp)
        processed = np.asarray(ranges, dtype=float)
        finite_mask = np.isfinite(processed)
        processed[~finite_mask] = self._config.range_max_clip_m
        processed = np.clip(processed, self._config.range_min_clip_m, self._config.range_max_clip_m)
        processed = moving_average_1d(processed, self._config.smoothing_kernel)
        processed = self._close_small_leaks(processed)
        angles = angle_min + np.arange(processed.size, dtype=float) * angle_increment

        forward_mask = np.abs(np.degrees(angles)) <= self._config.forward_sector_deg
        forward_sector = processed[forward_mask]
        forward_clearance = float(np.percentile(forward_sector, 20)) if forward_sector.size else self._config.range_max_clip_m
        ttc = float(np.inf)
        if speed_mps > 0.15:
            ttc = forward_clearance / max(speed_mps, 1.0e-3)

        xs = processed * np.cos(angles)
        ys = processed * np.sin(angles)
        points = np.column_stack((xs, ys))
        focus = (
            (points[:, 0] > 0.05)
            & (points[:, 0] <= self._config.boundary_lookahead_m)
            & (np.abs(np.degrees(angles)) <= self._config.focus_half_angle_deg)
        )
        points = points[focus]

        left_boundary = self._extract_boundary(points, side='left')
        right_boundary = self._extract_boundary(points, side='right')
        centerline, width_stats = self._build_debug_centerline(left_boundary, right_boundary)
        gap_angle, gap_meta = self._gap_debug.compute(processed, angles)

        if centerline.shape[0] >= 2:
            heading_hint = self._heading_from_centerline(centerline)
            curvature_hint = self._curvature_hint(centerline)
        else:
            heading_hint = 0.0
            curvature_hint = 0.0

        left_points = left_boundary.shape[0]
        right_points = right_boundary.shape[0]
        centerline_points = centerline.shape[0]
        side_score = 0.25 if left_points >= self._config.min_boundary_points_per_side else 0.0
        side_score += 0.25 if right_points >= self._config.min_boundary_points_per_side else 0.0
        count_score = min(1.0, centerline_points / 25.0)
        width_score = 1.0 - min(abs(self._width_estimate - self._config.nominal_track_width_m) / max(self._config.nominal_track_width_m, 1.0e-3), 1.0)
        confidence = clamp(0.15 + 0.35 * count_score + side_score + 0.25 * width_score, 0.0, 1.0)
        blocked = bool(forward_clearance < self._config.stop_distance_m or ttc < self._config.caution_ttc_s or centerline_points < 3)

        metadata = {
            'left_boundary_points': float(left_points),
            'right_boundary_points': float(right_points),
            'centerline_points': float(centerline_points),
            'width_estimate': float(self._width_estimate),
            'debug_gap_target_angle': float(gap_angle),
        }
        metadata.update(gap_meta)
        return TrackBoundaries(
            stamp=stamp,
            left_boundary=left_boundary,
            right_boundary=right_boundary,
            centerline=centerline,
            width_mean=float(width_stats[0]),
            width_min=float(width_stats[1]),
            width_std=float(width_stats[2]),
            heading_hint=float(heading_hint),
            curvature_hint=float(curvature_hint),
            forward_clearance=float(forward_clearance),
            ttc=float(ttc),
            blocked=blocked,
            confidence=float(confidence),
            processed_ranges=processed,
            angles=angles,
            metadata=metadata,
        )

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
            if 0 < i < n - 1 and 0 < run_length <= self._config.leak_fill_max_bins and j < n and fixed[i - 1] < threshold and fixed[j] < threshold:
                fill_value = min(fixed[i - 1], fixed[j])
                fixed[i:j] = fill_value
            i = j
        return fixed

    def _extract_boundary(self, points: np.ndarray, side: str) -> np.ndarray:
        if points.size == 0:
            return np.zeros((0, 2), dtype=float)
        if side == 'left':
            side_points = points[points[:, 1] > 0.0]
        else:
            side_points = points[points[:, 1] < 0.0]
        if side_points.shape[0] == 0:
            return np.zeros((0, 2), dtype=float)

        x_bin = self._config.x_bin_size_m
        x_centers = np.arange(0.20, self._config.boundary_lookahead_m + 1.0e-6, x_bin)
        samples: List[Tuple[float, float]] = []
        previous_y: Optional[float] = None
        for x_center in x_centers:
            mask = (side_points[:, 0] >= x_center - 0.5 * x_bin) & (side_points[:, 0] < x_center + 0.5 * x_bin)
            if not np.any(mask):
                continue
            y_candidates = side_points[mask, 1]
            y_value = float(np.min(y_candidates)) if side == 'left' else float(np.max(y_candidates))
            if previous_y is not None and abs(y_value - previous_y) > self._config.side_outlier_jump_m:
                nearest_idx = int(np.argmin(np.abs(y_candidates - previous_y)))
                alternative = float(y_candidates[nearest_idx])
                if abs(alternative - previous_y) > 1.5 * self._config.side_outlier_jump_m:
                    continue
                y_value = alternative
            samples.append((x_center, y_value))
            previous_y = y_value
        if len(samples) < 2:
            return np.zeros((0, 2), dtype=float)
        boundary = np.asarray(samples, dtype=float)
        order = np.argsort(boundary[:, 0])
        return boundary[order]

    def _build_debug_centerline(self, left_boundary: np.ndarray, right_boundary: np.ndarray) -> Tuple[np.ndarray, Tuple[float, float, float]]:
        if left_boundary.shape[0] == 0 and right_boundary.shape[0] == 0:
            return np.zeros((0, 2), dtype=float), (0.0, 0.0, 0.0)
        max_x = 0.0
        min_x = 0.35
        if left_boundary.shape[0] > 0:
            max_x = max(max_x, float(left_boundary[-1, 0]))
            min_x = min(min_x, float(left_boundary[0, 0]))
        if right_boundary.shape[0] > 0:
            max_x = max(max_x, float(right_boundary[-1, 0]))
            min_x = min(min_x, float(right_boundary[0, 0]))
        if max_x <= min_x:
            return np.zeros((0, 2), dtype=float), (self._width_estimate, self._width_estimate, 0.0)

        x_grid = np.arange(max(0.35, min_x), max_x + 1.0e-6, self._config.x_bin_size_m)
        left_y = interpolate_path_value(left_boundary, x_grid)
        right_y = interpolate_path_value(right_boundary, x_grid)
        both = np.isfinite(left_y) & np.isfinite(right_y)
        widths = np.asarray([], dtype=float)
        if np.any(both):
            widths = left_y[both] - right_y[both]
            widths = widths[np.isfinite(widths)]
            widths = widths[(widths >= self._config.min_track_width_m) & (widths <= self._config.max_track_width_m)]
        if widths.size > 0:
            measured_width = float(np.median(widths))
            self._width_estimate = low_pass(self._width_estimate, measured_width, 0.30)
        else:
            self._width_estimate = clamp(self._width_estimate, self._config.min_track_width_m, self._config.max_track_width_m)

        center_y = np.full_like(x_grid, np.nan, dtype=float)
        for idx in range(x_grid.size):
            has_left = math.isfinite(left_y[idx])
            has_right = math.isfinite(right_y[idx])
            if has_left and has_right:
                center_y[idx] = 0.5 * (left_y[idx] + right_y[idx])
            elif has_left:
                center_y[idx] = left_y[idx] - 0.5 * self._width_estimate
            elif has_right:
                center_y[idx] = right_y[idx] + 0.5 * self._width_estimate
        valid = np.isfinite(center_y)
        if np.count_nonzero(valid) < 2:
            return np.zeros((0, 2), dtype=float), (self._width_estimate, self._width_estimate, 0.0)
        centerline = np.column_stack((x_grid[valid], center_y[valid])).astype(float)
        if centerline.shape[0] >= 3:
            centerline[:, 1] = moving_average_1d(centerline[:, 1], self._config.centerline_smoothing_window)
        width_mean = float(np.mean(widths)) if widths.size else float(self._width_estimate)
        width_min = float(np.min(widths)) if widths.size else float(self._width_estimate)
        width_std = float(np.std(widths)) if widths.size else 0.0
        return centerline, (width_mean, width_min, width_std)

    def _heading_from_centerline(self, centerline: np.ndarray) -> float:
        if centerline.shape[0] < 2:
            return 0.0
        idx = min(centerline.shape[0] - 1, max(1, centerline.shape[0] // 3))
        dx = float(centerline[idx, 0] - centerline[0, 0])
        dy = float(centerline[idx, 1] - centerline[0, 1])
        return float(clamp(math.atan2(dy, max(dx, 1.0e-3)), -0.85, 0.85))

    def _curvature_hint(self, centerline: np.ndarray) -> float:
        if centerline.shape[0] < 3:
            return 0.0
        p0 = centerline[0]
        p1 = centerline[min(centerline.shape[0] - 1, centerline.shape[0] // 2)]
        p2 = centerline[-1]
        area2 = float((p1[0] - p0[0]) * (p2[1] - p0[1]) - (p1[1] - p0[1]) * (p2[0] - p0[0]))
        a = float(np.linalg.norm(p1 - p0))
        b = float(np.linalg.norm(p2 - p1))
        c = float(np.linalg.norm(p2 - p0))
        denom = max(a * b * c, 1.0e-6)
        return float(2.0 * area2 / denom)


LidarPerception = LidarTrackExtractor
