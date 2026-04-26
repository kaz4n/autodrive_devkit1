#!/usr/bin/env python3
"""
Perception Module V2 - Enhanced LiDAR Processing
Features:
- Temporal smoothing with EMA to reduce jitter
- Robust gap detection for corridor following
- Clearance and TTC estimation with filtering
"""

import numpy as np
from collections import deque
from typing import Dict, Optional, Tuple


class PerceptionV2:
    def __init__(self, params):
        self.params = params
        
        # Temporal smoothing buffers
        self.clearance_history = deque(maxlen=5)
        self.direction_history = deque(maxlen=params.direction_history_size)
        self.ttc_history = deque(maxlen=5)
        
        # Initialize with default values
        self.prev_clearance = 1.0
        self.prev_direction = 0.0
        self.prev_ttc = 10.0
        
        # Gap tracking
        self.current_gap_center = 0.0
        self.current_gap_width = 0.0
        
    def process_scan(self, scan_msg, dt: float) -> Dict:
        """
        Process LiDAR scan with temporal smoothing.
        Returns perception output dictionary.
        """
        # Convert to numpy array
        ranges = np.array(scan_msg.ranges)
        angles = np.linspace(
            scan_msg.angle_min, 
            scan_msg.angle_max, 
            len(ranges)
        )
        
        # Filter invalid ranges
        valid_mask = (ranges >= self.params.lidar_range_min) & \
                     (ranges <= self.params.lidar_range_max) & \
                     np.isfinite(ranges)
        
        if not np.any(valid_mask):
            return self._get_safe_output()
        
        valid_ranges = ranges[valid_mask]
        valid_angles = angles[valid_mask]
        
        # Convert to Cartesian coordinates
        x = valid_ranges * np.cos(valid_angles)
        y = valid_ranges * np.sin(valid_angles)
        points = np.column_stack([x, y])
        
        # ================================================================
        # Step 1: Detect gaps in the point cloud
        # ================================================================
        gap_info = self._detect_gaps(valid_ranges, valid_angles)
        
        if gap_info is None or len(gap_info) == 0:
            return self._get_safe_output()
        
        # Select best gap (widest within search range)
        best_gap = self._select_best_gap(gap_info)
        
        if best_gap is None:
            return self._get_safe_output()
        
        # ================================================================
        # Step 2: Compute clearance and direction
        # ================================================================
        clearance = best_gap['distance']
        direction = best_gap['angle']
        gap_width = best_gap['width']
        
        # Update current gap tracking
        self.current_gap_center = direction
        self.current_gap_width = gap_width
        
        # ================================================================
        # Step 3: Apply temporal smoothing (CRITICAL for continuous motion)
        # ================================================================
        smoothed_clearance = self._ema_smooth(
            clearance, 
            self.prev_clearance, 
            self.params.perception_ema_alpha
        )
        
        smoothed_direction = self._ema_smooth(
            direction, 
            self.prev_direction, 
            self.params.perception_ema_alpha
        )
        
        # Compute TTC (Time To Collision)
        ttc = self._compute_ttc(smoothed_clearance, dt)
        smoothed_ttc = self._ema_smooth(ttc, self.prev_ttc, 0.3)
        
        # Update previous values
        self.prev_clearance = smoothed_clearance
        self.prev_direction = smoothed_direction
        self.prev_ttc = smoothed_ttc
        
        # ================================================================
        # Step 4: Estimate corridor confidence
        # ================================================================
        corridor_confidence = self._estimate_corridor_confidence(
            points, 
            smoothed_direction, 
            gap_width
        )
        
        return {
            'clearance_m': smoothed_clearance,
            'direction_rad': smoothed_direction,
            'ttc_s': smoothed_ttc,
            'gap_width_m': gap_width,
            'corridor_confidence': corridor_confidence,
            'obstacle_points': points,
            'timestamp': scan_msg.header.stamp.sec + scan_msg.header.stamp.nanosec * 1e-9
        }
    
    def _detect_gaps(self, ranges: np.ndarray, angles: np.ndarray) -> list:
        """
        Detect gaps in the LiDAR data where the car can pass through.
        Returns list of gap dictionaries with: angle, distance, width
        """
        gaps = []
        
        # Sort by angle
        sort_idx = np.argsort(angles)
        sorted_ranges = ranges[sort_idx]
        sorted_angles = angles[sort_idx]
        
        # Find discontinuities (gaps)
        range_diff = np.diff(sorted_ranges)
        angle_diff = np.diff(sorted_angles)
        
        # A gap exists when there's a large jump in range values
        # or when the angular gap is large enough
        gap_threshold_angle = self.params.gap_min_width / max(np.median(sorted_ranges), 0.5)
        
        i = 0
        while i < len(sorted_ranges) - 1:
            # Check for angular gap
            if angle_diff[i] > gap_threshold_angle:
                # Found a gap
                gap_start_angle = sorted_angles[i]
                gap_end_angle = sorted_angles[i + 1]
                gap_center_angle = (gap_start_angle + gap_end_angle) / 2
                
                # Distance to gap (average of both sides)
                gap_distance = (sorted_ranges[i] + sorted_ranges[i + 1]) / 2
                
                # Gap width (arc length approximation)
                gap_width = gap_distance * (gap_end_angle - gap_start_angle)
                
                if gap_width >= self.params.gap_min_width:
                    gaps.append({
                        'angle': gap_center_angle,
                        'distance': gap_distance,
                        'width': gap_width,
                        'start_idx': i,
                        'end_idx': i + 1
                    })
            
            i += 1
        
        return gaps
    
    def _select_best_gap(self, gaps: list) -> Optional[Dict]:
        """
        Select the best gap to navigate through.
        Prioritizes: width, distance from center, safety margin
        """
        if len(gaps) == 0:
            return None
        
        # Filter gaps within search range
        valid_gaps = [
            g for g in gaps 
            if abs(g['angle']) <= self.params.gap_search_angle_range
        ]
        
        if len(valid_gaps) == 0:
            # Fallback: use any gap if none in search range
            valid_gaps = gaps
        
        # Score gaps: prefer wider gaps closer to center
        best_gap = None
        best_score = -float('inf')
        
        for gap in valid_gaps:
            # Score = width - penalty for angle deviation
            score = gap['width'] - 0.5 * abs(gap['angle'])
            
            if score > best_score:
                best_score = score
                best_gap = gap
        
        return best_gap
    
    def _compute_ttc(self, clearance: float, dt: float) -> float:
        """
        Compute Time To Collision based on clearance and estimated speed.
        """
        # Simple TTC estimate: assume current speed continues
        # In practice, you'd use actual speed from state estimator
        estimated_speed = 3.0  # Default assumption; will be refined
        
        if estimated_speed < 0.1:
            return 10.0  # Large TTC if nearly stopped
        
        ttc = clearance / estimated_speed
        return min(ttc, 10.0)  # Cap at 10 seconds
    
    def _ema_smooth(self, current: float, previous: float, alpha: float) -> float:
        """
        Exponential Moving Average smoothing.
        Alpha closer to 1 = more responsive, less smooth
        Alpha closer to 0 = more smooth, less responsive
        """
        return alpha * current + (1 - alpha) * previous
    
    def _estimate_corridor_confidence(self, points: np.ndarray, 
                                       direction: float, 
                                       gap_width: float) -> float:
        """
        Estimate confidence in the corridor detection.
        Higher confidence = more reliable perception output.
        """
        confidence = 0.5  # Base confidence
        
        # Increase confidence for wider gaps
        if gap_width > 0.8:
            confidence += 0.2
        elif gap_width > 0.5:
            confidence += 0.1
        
        # Increase confidence for more points (denser scan)
        if len(points) > 200:
            confidence += 0.15
        elif len(points) > 100:
            confidence += 0.1
        
        # Decrease confidence for extreme angles
        if abs(direction) > 0.5:
            confidence -= 0.1
        
        return np.clip(confidence, 0.0, 1.0)
    
    def _get_safe_output(self) -> Dict:
        """
        Return safe default output when perception fails.
        """
        return {
            'clearance_m': self.prev_clearance * 0.9,  # Conservative estimate
            'direction_rad': self.prev_direction,
            'ttc_s': self.prev_ttc * 0.8,
            'gap_width_m': 0.0,
            'corridor_confidence': 0.1,
            'obstacle_points': np.array([]),
            'timestamp': 0.0
        }
