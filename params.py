#!/usr/bin/env python3
"""
RoboRacer Parameters - Optimized for V2 Stack
Tuned for continuous motion, smooth steering, and robust curve handling.
"""

import numpy as np


class Params:
    def __init__(self):
        # =====================================================================
        # Vehicle Parameters (1:10 scale)
        # =====================================================================
        self.wheelbase = 0.35  # meters (typical 1:10 RC car)
        self.max_steering_angle = 0.5  # radians (~28 degrees)
        self.max_speed = 8.0  # m/s
        self.min_speed = 0.5  # m/s
        self.max_acceleration = 3.0  # m/s²
        self.max_deceleration = 5.0  # m/s² (braking)
        self.max_steering_rate = 2.0  # rad/s
        
        # =====================================================================
        # Control Parameters
        # =====================================================================
        self.control_hz = 100.0  # Hz - higher frequency for smoother control
        self.dt = 1.0 / self.control_hz
        
        # MPPI Controller Parameters
        self.mppi_horizon = 20  # time steps
        self.mppi_samples = 500  # number of trajectory samples
        self.mppi_lambda = 0.5  # temperature parameter for weighting
        self.mppi_alpha = 0.35  # EMA smoothing factor for controls (CRITICAL for smooth steering)
        
        # Cost function weights
        self.cost_weight_tracking = 5.0  # trajectory tracking
        self.cost_weight_obstacle = 50.0  # obstacle avoidance
        self.cost_weight_smoothness = 2.0  # control smoothness
        self.cost_weight_speed = 1.0  # speed maintenance
        
        # =====================================================================
        # Perception Parameters
        # =====================================================================
        self.lidar_range_min = 0.1  # meters
        self.lidar_range_max = 10.0  # meters
        self.lidar_angle_min = -np.pi / 2  # -90 degrees
        self.lidar_angle_max = np.pi / 2  # +90 degrees
        
        # Gap detection
        self.gap_min_width = 0.4  # meters (car width + margin)
        self.gap_search_angle_range = np.deg2rad(60)  # search ±30° from center
        
        # Temporal smoothing (CRITICAL for reducing jitter)
        self.perception_ema_alpha = 0.35  # EMA factor for clearance/direction
        self.direction_history_size = 5  # frames for direction smoothing
        
        # Corridor estimation
        self.corridor_samples = 100  # points to sample for corridor fitting
        self.corridor_ransac_iterations = 50
        self.corridor_inlier_threshold = 0.15  # meters
        
        # =====================================================================
        # Planning Parameters
        # =====================================================================
        # Adaptive lookahead based on curvature
        self.lookahead_base = 1.5  # meters (straight sections)
        self.lookahead_max = 3.0  # meters (high speed straight)
        self.lookahead_min = 0.8  # meters (tight curves)
        self.curvature_threshold = 0.5  # 1/m - above this, reduce lookahead
        
        # Trajectory generation
        self.trajectory_resolution = 0.1  # meters between waypoints
        self.max_trajectory_curvature = 2.0  # 1/m (respect vehicle limits)
        
        # Speed planning
        self.speed_straight = 6.0  # m/s
        self.speed_curve_medium = 4.0  # m/s
        self.speed_curve_tight = 2.5  # m/s
        self.speed_curve_threshold_medium = 0.3  # 1/m
        self.speed_curve_threshold_tight = 0.6  # 1/m
        
        # =====================================================================
        # State Estimation Parameters
        # =====================================================================
        self.state_ema_alpha = 0.2  # EMA for speed/curvature estimates
        self.imu_bias_decay = 0.98  # decay factor for bias estimation
        self.outlier_threshold_std = 3.0  # standard deviations for outlier rejection
        
        # =====================================================================
        # Mission / State Machine Parameters
        # =====================================================================
        self.sensor_timeout_ms = 500  # ms - timeout for each sensor
        self.mode_transition_delay = 0.5  # seconds before mode change
        self.emergency_deceleration = -4.0  # m/s²
        
        # Startup behavior
        self.startup_settle_time = 2.0  # seconds to wait after initialization
        self.ready_speed_threshold = 0.2  # m/s - below this, consider stopped
        
        # =====================================================================
        # Safety Parameters
        # =====================================================================
        self.min_clearance_emergency = 0.3  # meters - trigger emergency if below
        self.min_ttc_emergency = 0.5  # seconds - time to collision threshold
        self.max_lateral_accel = 8.0  # m/s² - limit for comfort/stability
        
        # Fallback behaviors
        self.fallback_speed = 2.0  # m/s when uncertain
        self.fallback_clearance_target = 0.6  # meters to maintain
        
        # =====================================================================
        # Debug / Logging
        # =====================================================================
        self.debug_enabled = True
        self.log_perception_details = False
        self.log_control_details = False
