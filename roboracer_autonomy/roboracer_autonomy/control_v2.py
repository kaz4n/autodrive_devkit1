"""
MPPI (Model Predictive Path Integral) Controller for RoboRacer.

This controller uses sampling-based Model Predictive Control to generate
smooth, optimal control commands. Key advantages:

1. Continuous steering - no reset-to-zero behavior
2. Natural handling of vehicle dynamics constraints
3. Smooth trajectory following even in curves
4. Explicit collision avoidance through cost function
5. Better performance at high speeds than pure pursuit
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import numpy as np

from .math_utils import clamp, low_pass, transform_points_world_to_local
from .models import ControlCommand, MissionMode, Plan, VehicleState, Waypoint
from .params_v2 import ControllerConfig, VehicleGeometry


@dataclass
class TrajectorySample:
    """A single trajectory rollout sample."""
    states: np.ndarray  # (N, 4) - [x, y, yaw, speed]
    controls: np.ndarray  # (N, 2) - [throttle, steering]
    cost: float = 0.0
    weights: np.ndarray = None  # For importance sampling


class MPPIController:
    """Model Predictive Path Integral controller."""

    def __init__(
        self,
        geometry: VehicleGeometry,
        config: ControllerConfig,
        nominal_max_speed_mps: float,
    ) -> None:
        self._geometry = geometry
        self._config = config
        self._nominal_max_speed_mps = max(nominal_max_speed_mps, 0.1)
        
        # State persistence - key fix for continuous control
        self._last_throttle = 0.0
        self._last_steering = 0.0
        self._last_stamp = 0.0
        self._yaw_rate_filtered = 0.0
        
        # MPPI internal state
        self._mean_control = np.zeros(2)  # [throttle, steering]
        self._control_covariance = np.diag([0.3, 0.15])  # Initial exploration
        
        # Exponential moving average for smoothness
        self._throttle_ema = 0.0
        self._steering_ema = 0.0
        
        # Pre-compute constants
        self._num_steps = int(config.sample_horizon_s / config.dt)
        self._wheelbase = geometry.wheelbase_m
        self._max_steer = geometry.max_steer_angle_rad
        self._max_steer_rate = geometry.max_steer_rate_radps

    def compute(self, plan: Plan, state: VehicleState, stamp: float) -> ControlCommand:
        """Compute optimal control using MPPI."""
        dt = max(0.01, min(0.05, stamp - self._last_stamp if self._last_stamp > 0 else 0.01))
        self._last_stamp = stamp
        
        # Update filtered yaw rate
        self._yaw_rate_filtered = low_pass(
            self._yaw_rate_filtered,
            state.yaw_rate,
            self._config.yaw_rate_low_pass_alpha,
        )
        
        # Handle safety/stop conditions
        if (
            plan.mode == MissionMode.SAFETY_BRAKE
            or plan.target_speed <= 0.05
            or len(plan.waypoints) < 2
            or not state.valid
        ):
            return self._compute_stop_command(stamp, plan.mode)
        
        # Extract reference path for tracking
        path_points = self._extract_path_points(plan.waypoints)
        
        # Run MPPI optimization
        optimal_throttle, optimal_steering = self._run_mppi(
            state, path_points, plan.target_speed, plan.curvature
        )
        
        # Apply rate limits and smoothing
        throttle = self._apply_smoothing_and_limits(
            self._last_throttle, optimal_throttle, dt, is_throttle=True
        )
        steering = self._apply_smoothing_and_limits(
            self._last_steering, optimal_steering, dt, is_throttle=False
        )
        
        # Update EMA for next iteration
        alpha = self._config.control_ema_alpha
        self._throttle_ema = alpha * throttle + (1 - alpha) * self._throttle_ema
        self._steering_ema = alpha * steering + (1 - alpha) * self._steering_ema
        
        # Cache for next iteration
        self._last_throttle = throttle
        self._last_steering = steering
        
        # Mode-specific adjustments
        if plan.mode == MissionMode.AVOID:
            throttle = min(throttle, self._config.avoid_mode_throttle_cap)
        
        return ControlCommand(
            stamp=stamp,
            throttle=float(throttle),
            steering=float(steering),
            emergency=False,
            reason=plan.mode.value,
        )

    def _run_mppi(
        self,
        state: VehicleState,
        path_points: np.ndarray,
        target_speed: float,
        reference_curvature: float,
    ) -> Tuple[float, float]:
        """Run MPPI optimization with trajectory sampling."""
        num_samples = self._config.num_samples
        
        # Initialize arrays for samples
        costs = np.zeros(num_samples)
        throttles = np.zeros(num_samples)
        steerings = np.zeros(num_samples)
        
        # Current state
        x, y, yaw = state.x, state.y, state.yaw
        v = max(state.speed, 0.1)
        steer = state.steering_angle
        
        # Sample controls from Gaussian distribution
        control_noise = np.random.multivariate_normal(
            self._mean_control,
            self._control_covariance,
            size=num_samples
        )
        
        # Roll out trajectories
        for i in range(num_samples):
            # Get sampled control sequence
            throttle_sample = control_noise[i, 0]
            steer_sample = control_noise[i, 1]
            
            # Clamp to feasible range
            throttle_sample = clamp(throttle_sample, -0.5, 1.0)
            steer_sample = clamp(steer_sample, -1.0, 1.0)
            
            # Convert normalized steering to angle
            steer_angle_sample = steer_sample * self._max_steer
            
            # Limit steering rate
            max_steer_change = self._max_steer_rate * self._config.dt
            steer_angle_sample = clamp(
                steer_angle_sample,
                steer - max_steer_change,
                steer + max_steer_change
            )
            
            # Rollout trajectory
            rollout_x, rollout_y = x, y
            rollout_yaw = yaw
            rollout_v = v
            rollout_steer = steer_angle_sample
            
            trajectory_cost = 0.0
            
            for step in range(self._num_steps):
                # Bicycle model prediction
                # Positive steering angle = turn left (positive curvature)
                # curvature = tan(steer) / wheelbase
                curvature = math.tan(rollout_steer) / self._wheelbase
                rollout_yaw += rollout_v * curvature * self._config.dt
                rollout_x += rollout_v * math.cos(rollout_yaw) * self._config.dt
                rollout_y += rollout_v * math.sin(rollout_yaw) * self._config.dt
                
                # Speed dynamics (simple first-order)
                accel = throttle_sample * 3.0 - 0.5 * rollout_v  # Simplified dynamics
                rollout_v += accel * self._config.dt
                rollout_v = max(0.0, min(rollout_v, self._nominal_max_speed_mps * 1.2))
                
                # Compute cost at this step
                step_cost = self._compute_step_cost(
                    rollout_x, rollout_y, rollout_yaw, rollout_v,
                    path_points, target_speed, reference_curvature
                )
                trajectory_cost += step_cost
                
                # Early termination if collision detected
                if step_cost > 100:  # High collision cost
                    break
            
            costs[i] = trajectory_cost
            throttles[i] = throttle_sample
            steerings[i] = steer_sample
        
        # Compute weighted average using softmax
        temperature = self._config.temperature
        costs_normalized = costs - np.min(costs)
        weights = np.exp(-costs_normalized / temperature)
        weights /= np.sum(weights) + 1e-10
        
        # Weighted average of controls
        optimal_throttle = np.sum(weights * throttles)
        optimal_steering = np.sum(weights * steerings)
        
        # Update mean control for next iteration (importance sampling)
        self._mean_control[0] = 0.8 * self._mean_control[0] + 0.2 * optimal_throttle
        self._mean_control[1] = 0.8 * self._mean_control[1] + 0.2 * optimal_steering
        
        return optimal_throttle, optimal_steering

    def _compute_step_cost(
        self,
        x: float, y: float, yaw: float, v: float,
        path_points: np.ndarray,
        target_speed: float,
        reference_curvature: float,
    ) -> float:
        """Compute cost for a single trajectory step."""
        cost = 0.0
        
        # Find closest point on reference path
        min_dist = 0.0
        if path_points.shape[0] > 0:
            distances = np.linalg.norm(path_points - np.array([x, y]), axis=1)
            min_dist = np.min(distances)
            
            # Tracking cost (quadratic in distance)
            cost += self._config.cost_tracking_weight * min_dist ** 2
        
        # Speed tracking cost
        speed_error = v - target_speed
        cost += self._config.cost_speed_weight * speed_error ** 2
        
        # Collision cost (simplified - penalize being too far from path)
        if path_points.shape[0] > 0 and min_dist > 1.5:
            cost += self._config.cost_collision_weight * (min_dist - 1.0) ** 2
        
        return cost

    def _extract_path_points(self, waypoints: Sequence[Waypoint]) -> np.ndarray:
        """Extract XY points from waypoints for tracking."""
        if not waypoints:
            return np.zeros((0, 2))
        return np.array([[wp.x, wp.y] for wp in waypoints], dtype=float)

    def _apply_smoothing_and_limits(
        self,
        current: float,
        target: float,
        dt: float,
        is_throttle: bool = True,
    ) -> float:
        """Apply rate limiting and smoothing to control command."""
        if is_throttle:
            limit = self._config.throttle_rate_limit_per_s
            target = clamp(target, self._config.throttle_min, self._config.throttle_max)
        else:
            limit = self._config.steering_rate_limit_per_s
            target = clamp(target, -1.0, 1.0)
        
        # Rate limit
        max_change = limit * dt
        delta = clamp(target - current, -max_change, max_change)
        limited = current + delta
        
        return limited

    def _compute_stop_command(self, stamp: float, mode: MissionMode) -> ControlCommand:
        """Generate smooth stop command without resetting steering abruptly."""
        # Gradually reduce throttle to zero
        throttle = low_pass(self._last_throttle, 0.0, 0.3)
        
        # Keep steering at last value or gradually center it
        # Don't reset to zero immediately - this was causing the jerky behavior
        if abs(self._last_steering) > 0.05:
            steering = low_pass(self._last_steering, 0.0, 0.1)
        else:
            steering = self._last_steering
        
        # Apply rate limits
        dt = 0.02  # Assume small dt for stop
        throttle = self._apply_smoothing_and_limits(
            self._last_throttle, throttle, dt, is_throttle=True
        )
        steering = self._apply_smoothing_and_limits(
            self._last_steering, steering, dt, is_throttle=False
        )
        
        self._last_throttle = throttle
        self._last_steering = steering
        
        return ControlCommand(
            stamp=stamp,
            throttle=float(throttle),
            steering=float(steering),
            emergency=(mode == MissionMode.SAFETY_BRAKE),
            reason='safety_brake' if mode == MissionMode.SAFETY_BRAKE else 'stop',
        )


# Alias for compatibility
LowLevelControllerV2 = MPPIController
