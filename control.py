#!/usr/bin/env python3
"""
Control Module V2 - MPPI (Model Predictive Path Integral) Controller
Features:
- Sampling-based optimal control with smooth output
- Exponential smoothing on control commands (CRITICAL for preventing steering reset)
- Bicycle model dynamics for realistic trajectory prediction
"""

import numpy as np
from typing import Dict, List, Optional


class ControlV2:
    def __init__(self, params):
        self.params = params
        
        # Previous control inputs for smoothing
        self.prev_throttle = 0.0
        self.prev_steering = 0.0
        
        # Control history for MPPI initialization
        self.control_sequence = np.zeros((params.mppi_horizon, 2))
        
    def compute_control(self, state: Dict, trajectory: List[np.ndarray], 
                       perception: Dict, dt: float) -> Dict:
        """
        Compute optimal control using MPPI algorithm.
        Returns throttle and steering commands.
        """
        # ================================================================
        # Extract state
        # ================================================================
        x = state['x']
        y = state['y']
        theta = state['theta']
        speed = state['speed']
        curvature = state['curvature']
        
        # ================================================================
        # Step 1: Sample trajectory rollouts using bicycle model
        # ================================================================
        sample_costs = []
        sample_trajectories = []
        
        for i in range(self.params.mppi_samples):
            # Sample control sequence
            control_seq = self._sample_control_sequence(speed, curvature)
            
            # Rollout trajectory using bicycle model
            rollout = self._bicycle_model_rollout(
                x=0.0, y=0.0, theta=0.0, v=speed,  # Local frame
                controls=control_seq,
                dt=dt
            )
            
            # Compute cost for this rollout
            cost = self._compute_cost(
                rollout=rollout,
                reference_trajectory=trajectory,
                controls=control_seq,
                perception=perception
            )
            
            sample_costs.append(cost)
            sample_trajectories.append(control_seq)
        
        # ================================================================
        # Step 2: Compute weighted average of control sequences (MPPI)
        # ================================================================
        sample_costs = np.array(sample_costs)
        
        # Normalize costs and compute weights
        min_cost = np.min(sample_costs)
        exp_costs = np.exp(-self.params.mppi_lambda * (sample_costs - min_cost))
        weights = exp_costs / (np.sum(exp_costs) + 1e-6)
        
        # Weighted average of control sequences
        optimal_control_seq = np.zeros_like(self.control_sequence)
        for i, ctrl_seq in enumerate(sample_trajectories):
            optimal_control_seq += weights[i] * ctrl_seq
        
        # Use first control from sequence
        optimal_throttle = optimal_control_seq[0, 0]
        optimal_steering = optimal_control_seq[0, 1]
        
        # ================================================================
        # Step 3: Apply exponential smoothing (CRITICAL for continuous motion)
        # ================================================================
        smoothed_throttle = self._ema_smooth(
            optimal_throttle, 
            self.prev_throttle, 
            self.params.mppi_alpha
        )
        
        smoothed_steering = self._ema_smooth(
            optimal_steering, 
            self.prev_steering, 
            self.params.mppi_alpha
        )
        
        # Update previous controls
        self.prev_throttle = smoothed_throttle
        self.prev_steering = smoothed_steering
        
        # Update control sequence for next iteration
        self.control_sequence = optimal_control_seq
        
        # ================================================================
        # Step 4: Apply rate limiting
        # ================================================================
        max_throttle_change = self.params.max_acceleration * dt
        max_steering_change = self.params.max_steering_rate * dt
        
        # Limit throttle change
        throttle_delta = np.clip(
            smoothed_throttle - self.prev_throttle,
            -max_throttle_change,
            max_throttle_change
        )
        final_throttle = self.prev_throttle + throttle_delta
        
        # Limit steering change (CRITICAL for preventing reset behavior)
        steering_delta = np.clip(
            smoothed_steering - self.prev_steering,
            -max_steering_change,
            max_steering_change
        )
        final_steering = self.prev_steering + steering_delta
        
        return {
            'throttle': float(final_throttle),
            'steering': float(final_steering),
            'raw_throttle': float(optimal_throttle),
            'raw_steering': float(optimal_steering)
        }
    
    def _sample_control_sequence(self, current_speed: float, 
                                 current_curvature: float) -> np.ndarray:
        """
        Sample a control sequence with noise around nominal values.
        """
        control_seq = np.zeros((self.params.mppi_horizon, 2))
        
        # Nominal controls based on current state
        nominal_throttle = 0.3 if current_speed < self.params.speed_straight else 0.1
        nominal_steering = current_curvature * self.params.wheelbase
        
        for t in range(self.params.mppi_horizon):
            # Add Gaussian noise
            throttle_noise = np.random.normal(0, 0.15)
            steering_noise = np.random.normal(0, 0.08)
            
            # Decay noise over horizon (more certainty in near future)
            decay = np.exp(-t / 5.0)
            
            control_seq[t, 0] = np.clip(
                nominal_throttle + throttle_noise * decay,
                -0.3, 1.0
            )
            control_seq[t, 1] = np.clip(
                nominal_steering + steering_noise * decay,
                -self.params.max_steering_angle,
                self.params.max_steering_angle
            )
        
        return control_seq
    
    def _bicycle_model_rollout(self, x: float, y: float, theta: float, 
                               v: float, controls: np.ndarray, 
                               dt: float) -> List[np.ndarray]:
        """
        Simulate vehicle dynamics using bicycle model.
        Returns list of [x, y, theta, v] states.
        """
        trajectory = []
        
        for t in range(len(controls)):
            throttle = controls[t, 0]
            steering = controls[t, 1]
            
            # Bicycle model kinematics
            # Acceleration from throttle
            accel = throttle * self.params.max_acceleration
            
            # Update velocity
            v_new = v + accel * dt
            v_new = np.clip(v_new, 0.0, self.params.max_speed)
            
            # Turning rate: ω = v * tan(δ) / L
            if abs(steering) > 1e-6:
                omega = v * np.tan(steering) / self.params.wheelbase
            else:
                omega = 0.0
            
            # Update position and orientation
            x_new = x + v * np.cos(theta) * dt
            y_new = y + v * np.sin(theta) * dt
            theta_new = theta + omega * dt
            v_new = v_new
            
            trajectory.append(np.array([x_new, y_new, theta_new, v_new]))
            
            # Update for next iteration
            x, y, theta, v = x_new, y_new, theta_new, v_new
        
        return trajectory
    
    def _compute_cost(self, rollout: List[np.ndarray], 
                     reference_trajectory: List[np.ndarray],
                     controls: np.ndarray,
                     perception: Dict) -> float:
        """
        Compute cost function for MPPI optimization.
        """
        cost = 0.0
        
        # -------------------------------------------------------------
        # Tracking cost: deviation from reference trajectory
        # -------------------------------------------------------------
        tracking_cost = 0.0
        for i, state in enumerate(rollout):
            if i < len(reference_trajectory):
                ref_point = reference_trajectory[i]
                error = np.linalg.norm(state[:2] - ref_point)
                tracking_cost += error ** 2
        
        tracking_cost *= self.params.cost_weight_tracking
        
        # -------------------------------------------------------------
        # Obstacle avoidance cost
        # -------------------------------------------------------------
        obstacle_points = perception.get('obstacle_points', np.array([]))
        obstacle_cost = 0.0
        
        if len(obstacle_points) > 0:
            for state in rollout:
                pos = state[:2]
                distances = np.linalg.norm(obstacle_points - pos, axis=1)
                min_dist = np.min(distances)
                
                # Penalty for being close to obstacles
                if min_dist < 0.5:
                    obstacle_cost += (0.5 - min_dist) ** 2
        
        obstacle_cost *= self.params.cost_weight_obstacle
        
        # -------------------------------------------------------------
        # Control smoothness cost
        # -------------------------------------------------------------
        smoothness_cost = 0.0
        for t in range(1, len(controls)):
            delta_throttle = (controls[t, 0] - controls[t-1, 0]) ** 2
            delta_steering = (controls[t, 1] - controls[t-1, 1]) ** 2
            smoothness_cost += delta_throttle + delta_steering
        
        smoothness_cost *= self.params.cost_weight_smoothness
        
        # -------------------------------------------------------------
        # Speed maintenance cost (prefer higher speeds when safe)
        # -------------------------------------------------------------
        speed_cost = 0.0
        for state in rollout:
            v = state[3]
            # Penalize low speeds
            if v < self.params.speed_straight:
                speed_cost += (self.params.speed_straight - v) ** 2
        
        speed_cost *= self.params.cost_weight_speed
        
        # Total cost
        cost = tracking_cost + obstacle_cost + smoothness_cost + speed_cost
        
        return cost
    
    def _ema_smooth(self, current: float, previous: float, alpha: float) -> float:
        """
        Exponential Moving Average smoothing.
        This is CRITICAL for preventing the steering reset behavior.
        """
        return alpha * current + (1 - alpha) * previous
