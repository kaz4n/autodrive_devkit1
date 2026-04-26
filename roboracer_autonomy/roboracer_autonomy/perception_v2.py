"""
Enhanced Perception v2 for RoboRacer.

Key improvements:
- Better temporal smoothing of LiDAR data
- More robust boundary extraction in curves
- Improved gap detection with continuity
- Camera fusion only when highly confident
"""

from __future__ import annotations

import math
from typing import List, Optional, Tuple

import numpy as np

try:
    import cv2
except ImportError:
    cv2 = None

from .math_utils import clamp, interpolate_path_value, low_pass, moving_average_1d
from .models import CameraObservation, TrackBoundaries
from .params_v2 import CameraConfig, LidarConfig


class GapFallbackV2:
    """Improved follow-the-gap with better continuity."""

    def __init__(self, config: LidarConfig) -> None:
        self._config = config
        self._previous_target_angle: float = 0.0
        self._angle_history: List[float] = []

    def compute(self, processed_ranges: np.ndarray, angles: np.ndarray) -> Tuple[float, dict]:
        if processed_ranges.size == 0:
            return 0.0, {}

        # Focus on forward sector
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
            # Higher continuity weight for smoother transitions
            score -= self._config.gap_continuity_weight * np.abs(
                angles[candidate_indices] - self._previous_target_angle
            )
            target_index = int(candidate_indices[np.argmax(score)])

        target_angle = float(angles[target_index])
        
        # Smoother temporal filtering
        alpha = 0.35 if abs(target_angle - self._previous_target_angle) < 0.3 else 0.25
        self._previous_target_angle = low_pass(self._previous_target_angle, target_angle, alpha)
        
        # Keep short history for diagnostics
        self._angle_history.append(target_angle)
        if len(self._angle_history) > 10:
            self._angle_history.pop(0)
        
        return self._previous_target_angle, {
            'nearest_range': nearest_range,
            'gap_start_angle': float(angles[gap_start]),
            'gap_end_angle': float(angles[gap_end]),
        }

    def _bubble_bin_count(self, nearest_range: float, angle_increment: float) -> int:
        safe_range = max(nearest_range, 0.10)
        bubble_angle = math.atan2(self._config.bubble_radius_m, safe_range)
        return max(1, int(bubble_angle / max(angle_increment, 1e-4)))

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


class LidarPerceptionV2:
    """Enhanced LiDAR perception with better temporal consistency."""

    def __init__(self, config: LidarConfig) -> None:
        self._config = config
        self._gap_fallback = GapFallbackV2(config)
        self._width_estimate = config.nominal_track_width_m
        self._previous_centerline = np.zeros((0, 2), dtype=float)
        self._centerline_velocity = np.zeros((0, 2), dtype=float)

    def process(
        self,
        ranges: np.ndarray,
        angle_min: float,
        angle_increment: float,
        speed_mps: float,
        stamp: float,
    ) -> TrackBoundaries:
        if ranges.size == 0:
            return TrackBoundaries(stamp=stamp)

        # Preprocess ranges
        processed = np.asarray(ranges, dtype=float)
        finite_mask = np.isfinite(processed)
        processed[~finite_mask] = self._config.range_max_clip_m
        processed = np.clip(
            processed,
            self._config.range_min_clip_m,
            self._config.range_max_clip_m,
        )
        # More aggressive smoothing for stability
        processed = moving_average_1d(processed, self._config.smoothing_kernel)
        processed = self._close_small_leaks(processed)

        angles = angle_min + np.arange(processed.size, dtype=float) * angle_increment
        
        # Forward clearance and TTC
        forward_mask = np.abs(np.degrees(angles)) <= self._config.forward_sector_deg
        forward_sector = processed[forward_mask]
        if forward_sector.size == 0:
            forward_clearance = self._config.range_max_clip_m
        else:
            forward_clearance = float(np.percentile(forward_sector, 20))
        ttc = float(np.inf)
        if speed_mps > 0.15:
            ttc = forward_clearance / max(speed_mps, 1e-3)

        # Convert to points
        xs = processed * np.cos(angles)
        ys = processed * np.sin(angles)
        points = np.column_stack((xs, ys))
        
        # Focus region for boundary extraction
        focus = (
            (points[:, 0] > 0.05)
            & (points[:, 0] <= self._config.boundary_lookahead_m)
            & (np.abs(np.degrees(angles)) <= self._config.focus_half_angle_deg)
        )
        points = points[focus]

        # Extract boundaries
        left_boundary = self._extract_boundary(points, side='left')
        right_boundary = self._extract_boundary(points, side='right')
        centerline, width_stats = self._build_centerline(left_boundary, right_boundary)

        # Gap fallback
        gap_target_angle, gap_meta = self._gap_fallback.compute(processed, angles)
        
        # Compute metrics from centerline
        if centerline.shape[0] >= 2:
            center_bias = self._center_bias_from_centerline(centerline)
            heading_error = self._heading_from_centerline(centerline)
            curvature_hint = self._curvature_hint(centerline)
        else:
            center_bias = 0.0
            heading_error = 0.0
            curvature_hint = 0.0

        # Confidence calculation
        centerline_points = centerline.shape[0]
        left_points = left_boundary.shape[0]
        right_points = right_boundary.shape[0]
        side_score = 0.0
        side_score += 0.25 if left_points >= self._config.min_boundary_points_per_side else 0.0
        side_score += 0.25 if right_points >= self._config.min_boundary_points_per_side else 0.0
        count_score = min(1.0, centerline_points / 25.0)
        width_score = 1.0 - min(
            abs(self._width_estimate - self._config.nominal_track_width_m)
            / max(self._config.nominal_track_width_m, 1e-3),
            1.0,
        )
        confidence = clamp(0.15 + 0.35 * count_score + side_score + 0.25 * width_score, 0.0, 1.0)
        
        # Blocked detection - more conservative
        blocked = (
            forward_clearance < self._config.stop_distance_m
            or ttc < self._config.caution_ttc_s
            or centerline_points < 3
        )

        metadata = {
            'left_boundary_points': float(left_points),
            'right_boundary_points': float(right_points),
            'centerline_points': float(centerline_points),
            'width_estimate': float(self._width_estimate),
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
            center_bias=float(center_bias),
            heading_error=float(heading_error),
            curvature_hint=float(curvature_hint),
            gap_target_angle=float(gap_target_angle),
            forward_clearance=float(forward_clearance),
            ttc=float(ttc),
            blocked=bool(blocked),
            confidence=float(confidence),
            angles=angles,
            processed_ranges=processed,
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

    def _extract_boundary(self, points: np.ndarray, side: str) -> np.ndarray:
        if points.size == 0:
            return np.zeros((0, 2), dtype=float)
        if side == 'left':
            points = points[points[:, 1] > 0.0]
        else:
            points = points[points[:, 1] < 0.0]
        if points.shape[0] == 0:
            return np.zeros((0, 2), dtype=float)

        x_bin = self._config.x_bin_size_m
        x_centers = np.arange(0.20, self._config.boundary_lookahead_m + 1e-6, x_bin)
        samples: List[Tuple[float, float]] = []
        previous_y: Optional[float] = None

        for x_center in x_centers:
            mask = (points[:, 0] >= x_center - 0.5 * x_bin) & (points[:, 0] < x_center + 0.5 * x_bin)
            if not np.any(mask):
                continue
            y_candidates = points[mask, 1]
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
        boundary[:, 1] = moving_average_1d(boundary[:, 1], 5)
        return boundary

    def _build_centerline(
        self,
        left_boundary: np.ndarray,
        right_boundary: np.ndarray,
    ) -> Tuple[np.ndarray, Tuple[float, float, float]]:
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

        x_grid = np.arange(max(0.35, min_x), max_x + 1e-6, self._config.x_bin_size_m)
        left_y = interpolate_path_value(left_boundary, x_grid)
        right_y = interpolate_path_value(right_boundary, x_grid)
        both = np.isfinite(left_y) & np.isfinite(right_y)

        if np.any(both):
            widths = left_y[both] - right_y[both]
            widths = widths[np.isfinite(widths)]
            widths = widths[
                (widths >= self._config.min_track_width_m)
                & (widths <= self._config.max_track_width_m)
            ]
        else:
            widths = np.asarray([], dtype=float)

        if widths.size > 0:
            measured_width = float(np.median(widths))
            # Slower adaptation for stability
            self._width_estimate = low_pass(self._width_estimate, measured_width, 0.25)
        else:
            self._width_estimate = clamp(
                self._width_estimate,
                self._config.min_track_width_m,
                self._config.max_track_width_m,
            )

        center_y = np.full_like(x_grid, np.nan, dtype=float)
        for idx, x_value in enumerate(x_grid):
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
        
        # Smoother centerline filtering
        centerline[:, 1] = moving_average_1d(centerline[:, 1], self._config.centerline_smoothing_window)
        
        # Temporal smoothing with previous frame
        if self._previous_centerline.shape[0] >= 2:
            prev_y = interpolate_path_value(self._previous_centerline, centerline[:, 0])
            prev_valid = np.isfinite(prev_y)
            # Blend with previous estimate for temporal consistency
            centerline[prev_valid, 1] = 0.65 * centerline[prev_valid, 1] + 0.35 * prev_y[prev_valid]
        self._previous_centerline = centerline

        if widths.size == 0:
            width_mean = self._width_estimate
            width_min = self._width_estimate
            width_std = 0.0
        else:
            width_mean = float(np.mean(widths))
            width_min = float(np.min(widths))
            width_std = float(np.std(widths))
        return centerline, (width_mean, width_min, width_std)

    def _center_bias_from_centerline(self, centerline: np.ndarray) -> float:
        if centerline.shape[0] == 0:
            return 0.0
        ref_idx = min(centerline.shape[0] - 1, max(0, centerline.shape[0] // 4))
        ref_point = centerline[ref_idx]
        return float(clamp(math.atan2(ref_point[1], max(ref_point[0], 0.75)), -0.65, 0.65))

    def _heading_from_centerline(self, centerline: np.ndarray) -> float:
        if centerline.shape[0] < 2:
            return 0.0
        tail_idx = min(centerline.shape[0] - 1, max(1, centerline.shape[0] // 3))
        dx = float(centerline[tail_idx, 0] - centerline[0, 0])
        dy = float(centerline[tail_idx, 1] - centerline[0, 1])
        return float(clamp(math.atan2(dy, max(dx, 1e-3)), -0.85, 0.85))

    def _curvature_hint(self, centerline: np.ndarray) -> float:
        if centerline.shape[0] < 3:
            return 0.0
        p0 = centerline[0]
        p1 = centerline[min(centerline.shape[0] - 1, centerline.shape[0] // 2)]
        p2 = centerline[-1]
        area2 = float(
            (p1[0] - p0[0]) * (p2[1] - p0[1])
            - (p1[1] - p0[1]) * (p2[0] - p0[0])
        )
        a = float(np.linalg.norm(p1 - p0))
        b = float(np.linalg.norm(p2 - p1))
        c = float(np.linalg.norm(p2 - p0))
        denom = max(a * b * c, 1e-6)
        return float(2.0 * area2 / denom)


class CameraPerceptionV2:
    """Simplified camera perception - used only as auxiliary."""

    def __init__(self, config: CameraConfig) -> None:
        self._config = config
        self._last_confidence = 0.0

    def process(self, rgb_image: np.ndarray, stamp: float) -> CameraObservation:
        if not self._config.enabled or cv2 is None or rgb_image.size == 0:
            self._last_confidence = 0.0
            return CameraObservation(stamp=stamp)

        height, width = rgb_image.shape[:2]
        roi_start = int(clamp(self._config.roi_vertical_start, 0.0, 0.95) * height)
        roi = rgb_image[roi_start:height, :, :]
        if roi.size == 0:
            self._last_confidence = 0.0
            return CameraObservation(stamp=stamp)

        gray = cv2.cvtColor(roi, cv2.COLOR_RGB2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blurred, self._config.canny_low, self._config.canny_high)

        lines = cv2.HoughLinesP(
            edges,
            rho=1,
            theta=np.pi / 180,
            threshold=self._config.hough_threshold,
            minLineLength=self._config.hough_min_line_length,
            maxLineGap=self._config.hough_max_line_gap,
        )

        if lines is None or len(lines) == 0:
            self._last_confidence *= (1 - self._config.confidence_decay_rate)
            return CameraObservation(
                stamp=stamp,
                confidence=float(self._last_confidence),
            )

        left_lines = []
        right_lines = []
        for line in lines:
            x1, y1, x2, y2 = line[0]
            if x2 == x1:
                continue
            slope = (y2 - y1) / (x2 - x1)
            if abs(slope) < self._config.min_slope_abs:
                continue
            if slope > 0:
                left_lines.append(line[0])
            else:
                right_lines.append(line[0])

        has_left = len(left_lines) >= 2
        has_right = len(right_lines) >= 2
        
        # Conservative confidence calculation
        line_count = len(left_lines) + len(right_lines)
        base_confidence = min(1.0, line_count / 10.0)
        if has_left and has_right:
            base_confidence = min(1.0, base_confidence + 0.2)
        
        # Decay confidence when conditions are poor
        if line_count < 4:
            self._last_confidence *= (1 - self._config.confidence_decay_rate)
        else:
            self._last_confidence = low_pass(self._last_confidence, base_confidence, 0.3)
        
        # Compute simple heading error from line angles
        all_slopes = []
        for line in left_lines + right_lines:
            x1, y1, x2, y2 = line
            if x2 != x1:
                all_slopes.append((y2 - y1) / (x2 - x1))
        
        if all_slopes:
            avg_slope = np.median(all_slopes)
            heading_error = math.atan(avg_slope) * self._config.heading_scale_rad
        else:
            heading_error = 0.0

        return CameraObservation(
            stamp=stamp,
            center_offset=0.0,  # Simplified
            heading_error=float(heading_error),
            confidence=float(self._last_confidence),
            has_left_boundary=has_left,
            has_right_boundary=has_right,
            metadata={'line_count': float(line_count)},
        )
