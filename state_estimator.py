#!/usr/bin/env python3
"""
State Estimator V2 - Sensor Fusion with Adaptive Weights
Combines odometry, IMU, and perception for robust state estimation.
"""

import numpy as np
from typing import Dict, Optional
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


class StateEstimatorV2:
    def __init__(self, params):
        self.params = params
        
        # State variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.speed = 0.0
        self.angular_rate = 0.0
        self.curvature = 0.0
        
        # IMU bias estimates
        self.gyro_bias = 0.0
        self.accel_bias = np.array([0.0, 0.0, 0.0])
        
        # EMA smoothing
        self.speed_ema = 0.0
        self.curvature_ema = 0.0
        self.prev_speed = 0.0
        self.prev_curvature = 0.0
        
        # Covariance estimates (for adaptive weighting)
        self.speed_variance = 0.1
        self.position_variance = 0.1
        
    def update(self, odom: Optional[Odometry], 
               imu: Optional[Imu], 
               perception: Dict, 
               dt: float) -> Dict:
        """
        Fuse sensor data to estimate vehicle state.
        Returns state dictionary with position, velocity, and curvature.
        """
        # ================================================================
        # Step 1: Process odometry
        # ================================================================
        if odom is not None:
            # Extract pose and twist from odometry
            pose = odom.pose.pose
            twist = odom.twist.twist
            
            # Position
            self.x = pose.position.x
            self.y = pose.position.y
            
            # Orientation (quaternion to euler)
            q = pose.orientation
            self.theta = self._quaternion_to_yaw(q)
            
            # Linear speed
            linear_speed = np.sqrt(
                twist.linear.x**2 + twist.linear.y**2
            )
            
            # Angular rate from odometry
            odom_angular_rate = twist.angular.z
            
            # Apply EMA smoothing to speed
            self.speed_ema = self._ema_smooth(
                linear_speed, 
                self.speed_ema, 
                self.params.state_ema_alpha
            )
            self.speed = self.speed_ema
            
        # ================================================================
        # Step 2: Process IMU
        # ================================================================
        if imu is not None:
            # Gyroscope (angular rate)
            gyro_z = imu.angular_velocity.z
            
            # Remove bias estimate
            corrected_gyro = gyro_z - self.gyro_bias
            
            # Update angular rate (prefer IMU over odometry for dynamics)
            self.angular_rate = corrected_gyro
            
            # Update bias estimate slowly
            self.gyro_bias = (self.gyro_bias * self.params.imu_bias_decay + 
                             (1 - self.params.imu_bias_decay) * gyro_z)
            
            # Compute curvature: κ = ω / v
            if abs(self.speed) > 0.1:
                instantaneous_curvature = self.angular_rate / self.speed
            else:
                instantaneous_curvature = 0.0
            
            # Apply EMA smoothing to curvature
            self.curvature_ema = self._ema_smooth(
                instantaneous_curvature,
                self.curvature_ema,
                self.params.state_ema_alpha
            )
            self.curvature = self.curvature_ema
        
        # ================================================================
        # Step 3: Fallback if sensors unavailable
        # ================================================================
        if odom is None and imu is None:
            # Decay speed estimate
            self.speed = self.speed * 0.95
            self.curvature = 0.0
        
        # ================================================================
        # Step 4: Validate estimates against physical limits
        # ================================================================
        self.speed = np.clip(self.speed, 0.0, self.params.max_speed)
        self.curvature = np.clip(
            self.curvature, 
            -self.params.max_trajectory_curvature,
            self.params.max_trajectory_curvature
        )
        
        # ================================================================
        # Step 5: Return state dictionary
        # ================================================================
        return {
            'x': self.x,
            'y': self.y,
            'theta': self.theta,
            'speed': self.speed,
            'angular_rate': self.angular_rate,
            'curvature': self.curvature,
            'gyro_bias': self.gyro_bias,
            'position_covariance': self.position_variance,
            'speed_covariance': self.speed_variance
        }
    
    def _quaternion_to_yaw(self, q) -> float:
        """Convert quaternion to yaw angle."""
        # yaw (z-axis rotation)
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        return yaw
    
    def _ema_smooth(self, current: float, previous: float, alpha: float) -> float:
        """Exponential Moving Average smoothing."""
        if previous == 0.0 and current != 0.0:
            return current
        return alpha * current + (1 - alpha) * previous
