#!/usr/bin/env python3
"""
Planning Module V2 - Curvature-Adaptive Trajectory Generation
Features:
- Adaptive lookahead based on curvature
- Smooth trajectory generation with clothoid-like properties
- Fallback behaviors for edge cases
"""

import numpy as np
from typing import Dict, List, Optional


class PlanningV2:
    def __init__(self, params):
        self.params = params
        
        # Trajectory buffer for smoothing
        self.prev_trajectory = None
        self.prev_direction = 0.0
        
    def plan(self, state: Dict, perception: Dict, mission_mode: str) -> Optional[List[np.ndarray]]:
        """
        Generate trajectory based on current state and perception.
        Returns list of [x, y] waypoints in local frame.
        """
        # ================================================================
        # Extract relevant information
        # ================================================================
        speed = state['speed']
        curvature = state['curvature']
        theta = state['theta']
        
        clearance = perception.get('clearance_m', 1.0)
        direction = perception.get('direction_rad', 0.0)
        confidence = perception.get('corridor_confidence', 0.5)
        
        # ================================================================
        # Step 1: Compute adaptive lookahead distance
        # ================================================================
        lookahead = self._compute_adaptive_lookahead(curvature, speed)
        
        # ================================================================
        # Step 2: Determine target direction
        # ================================================================
        # Blend perception direction with previous for smoothness
        smoothed_direction = 0.7 * direction + 0.3 * self.prev_direction
        self.prev_direction = smoothed_direction
        
        # Limit direction change rate
        max_direction_change = self.params.max_steering_angle * 0.5
        if abs(smoothed_direction - self.prev_direction) > max_direction_change:
            if smoothed_direction > self.prev_direction:
                smoothed_direction = self.prev_direction + max_direction_change
            else:
                smoothed_direction = self.prev_direction - max_direction_change
        
        # ================================================================
        # Step 3: Generate trajectory waypoints
        # ================================================================
        trajectory = self._generate_trajectory(
            lookahead=lookahead,
            direction=smoothed_direction,
            curvature=curvature,
            n_points=int(lookahead / self.params.trajectory_resolution)
        )
        
        if trajectory is None or len(trajectory) < 2:
            return self._get_straight_trajectory(lookahead=1.0)
        
        # ================================================================
        # Step 4: Validate trajectory against obstacles
        # ================================================================
        obstacle_points = perception.get('obstacle_points', np.array([]))
        
        if len(obstacle_points) > 0:
            is_safe = self._validate_trajectory(trajectory, obstacle_points)
            
            if not is_safe:
                # Try fallback: reduce speed and replan with conservative parameters
                fallback_trajectory = self._generate_fallback_trajectory(
                    perception, state
                )
                if fallback_trajectory is not None:
                    trajectory = fallback_trajectory
                else:
                    # Emergency: stop
                    return None
        
        # ================================================================
        # Step 5: Store trajectory for next iteration
        # ================================================================
        self.prev_trajectory = trajectory
        
        return trajectory
    
    def _compute_adaptive_lookahead(self, curvature: float, speed: float) -> float:
        """
        Compute lookahead distance based on curvature and speed.
        Reduces lookahead in curves for better tracking.
        """
        # Base lookahead
        lookahead = self.params.lookahead_base
        
        # Reduce lookahead for high curvature (tight turns)
        if abs(curvature) > self.params.curvature_threshold:
            # Linear interpolation between min and base
            curvature_ratio = min(abs(curvature) / self.params.max_trajectory_curvature, 1.0)
            lookahead = (self.params.lookahead_min + 
                        (self.params.lookahead_base - self.params.lookahead_min) * 
                        (1 - curvature_ratio))
        
        # Increase lookahead for high speed on straights
        if abs(curvature) < 0.1 and speed > 4.0:
            lookahead = min(lookahead * 1.2, self.params.lookahead_max)
        
        # Ensure minimum lookahead
        lookahead = max(lookahead, self.params.lookahead_min)
        
        return lookahead
    
    def _generate_trajectory(self, lookahead: float, direction: float, 
                            curvature: float, n_points: int) -> List[np.ndarray]:
        """
        Generate smooth trajectory using circular arc + straight line model.
        """
        if n_points < 2:
            n_points = 2
        
        trajectory = []
        
        # Compute turning radius from curvature
        if abs(curvature) > 0.01:
            radius = 1.0 / curvature
        else:
            radius = float('inf')
        
        # Generate waypoints along the arc/line
        for i in range(n_points):
            s = (i / (n_points - 1)) * lookahead  # arc length
            
            if abs(curvature) > 0.01:
                # Circular arc
                angle = s / radius
                x = radius * np.sin(angle)
                y = radius * (1 - np.cos(angle))
            else:
                # Straight line with direction offset
                x = s * np.cos(direction)
                y = s * np.sin(direction)
            
            trajectory.append(np.array([x, y]))
        
        return trajectory
    
    def _generate_fallback_trajectory(self, perception: Dict, 
                                      state: Dict) -> Optional[List[np.ndarray]]:
        """
        Generate conservative fallback trajectory when primary planning fails.
        """
        clearance = perception.get('clearance_m', 0.5)
        
        if clearance < self.params.min_clearance_emergency:
            return None  # No safe path
        
        # Generate short, conservative trajectory
        return self._get_straight_trajectory(lookahead=min(clearance * 0.5, 1.0))
    
    def _get_straight_trajectory(self, lookahead: float = 1.0) -> List[np.ndarray]:
        """Generate a simple straight trajectory."""
        n_points = max(int(lookahead / self.params.trajectory_resolution), 2)
        
        trajectory = []
        for i in range(n_points):
            s = (i / (n_points - 1)) * lookahead
            trajectory.append(np.array([s, 0.0]))
        
        return trajectory
    
    def _validate_trajectory(self, trajectory: List[np.ndarray], 
                            obstacle_points: np.ndarray) -> bool:
        """
        Check if trajectory collides with any obstacle points.
        Returns True if safe, False if collision detected.
        """
        safety_margin = 0.2  # meters
        
        for waypoint in trajectory:
            # Check distance to nearest obstacle
            if len(obstacle_points) > 0:
                distances = np.linalg.norm(obstacle_points - waypoint, axis=1)
                min_distance = np.min(distances)
                
                if min_distance < safety_margin:
                    return False
        
        return True
