#!/usr/bin/env python3
"""
RoboRacer Autonomy Stack v2 - Main Node
Optimized for continuous motion, better curve handling, and robust sensor integration.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.executors import MultiThreadedExecutor
from rclpy.timer import Timer

from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Bool, Float32, String
from roboracer_msgs.msg import CarCommand, Diagnostics, SystemState

import numpy as np
from collections import deque
import time
import threading

# Import v2 modules
from params import Params
from perception_v2 import PerceptionV2
from state_estimator_v2 import StateEstimatorV2
from planning_v2 import PlanningV2
from control_v2 import ControlV2
from mission_v2 import MissionV2


class AutonomyNodeV2(Node):
    def __init__(self):
        super().__init__('roboracer_autonomy')
        
        self.params = Params()
        
        # =====================================================================
        # CRITICAL FIX: Robust QoS Settings for Sensor Data
        # =====================================================================
        # Use BEST_EFFORT reliability to match typical LiDAR/IMU publishers
        # Use larger depth to prevent message loss during processing
        self.sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # =====================================================================
        # Subscribers with robust QoS
        # =====================================================================
        self.lidar_sub = self.create_subscription(
            LaserScan, 
            '/scan', 
            self.lidar_callback, 
            self.sensor_qos
        )
        
        self.imu_sub = self.create_subscription(
            Imu, 
            '/imu/data', 
            self.imu_callback, 
            self.sensor_qos
        )
        
        self.odom_sub = self.create_subscription(
            Odometry, 
            '/odom', 
            self.odom_callback, 
            self.sensor_qos
        )
        
        # =====================================================================
        # Publishers
        # =====================================================================
        self.cmd_pub = self.create_publisher(CarCommand, '/car_cmd', 10)
        self.state_pub = self.create_publisher(SystemState, '/autonomy/state', 10)
        self.diagnostics_pub = self.create_publisher(Diagnostics, '/autonomy/diagnostics', 10)
        self.debug_info_pub = self.create_publisher(String, '/autonomy/debug', 10)
        
        # =====================================================================
        # Initialize Modules
        # =====================================================================
        self.perception = PerceptionV2(self.params)
        self.state_estimator = StateEstimatorV2(self.params)
        self.planner = PlanningV2(self.params)
        self.controller = ControlV2(self.params)
        self.mission = MissionV2(self.params)
        
        # =====================================================================
        # State Variables
        # =====================================================================
        self.latest_lidar = None
        self.latest_imu = None
        self.latest_odom = None
        
        self.lidar_timestamp = 0.0
        self.imu_timestamp = 0.0
        self.odom_timestamp = 0.0
        
        self.current_state = SystemState()
        self.current_state.mode = String.DATA_IDLE
        self.current_state.emergency = True
        self.current_state.emergency_reason = "initializing"
        
        # Timing
        self.last_control_time = self.get_clock().now()
        self.control_period = 1.0 / self.params.control_hz
        
        # =====================================================================
        # Control Loop Timer
        # =====================================================================
        self.control_timer = self.create_timer(
            self.control_period,
            self.control_loop
        )
        
        # Diagnostics timer (slower)
        self.diagnostics_timer = self.create_timer(
            1.0,
            self.publish_diagnostics
        )
        
        self.get_logger().info("RoboRacer Autonomy V2 initialized")
        self.get_logger().info(f"Control frequency: {self.params.control_hz} Hz")
        self.get_logger().info("Using MPPI controller with temporal smoothing")
    
    # ========================================================================
    # Callbacks
    # ========================================================================
    
    def lidar_callback(self, msg: LaserScan):
        self.latest_lidar = msg
        self.lidar_timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
    def imu_callback(self, msg: Imu):
        self.latest_imu = msg
        self.imu_timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
    def odom_callback(self, msg: Odometry):
        self.latest_odom = msg
        self.odom_timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    
    # ========================================================================
    # Main Control Loop
    # ========================================================================
    
    def control_loop(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_control_time).nanoseconds * 1e-9
        self.last_control_time = current_time
        
        try:
            # -------------------------------------------------------------
            # Step 1: Check sensor availability with individual timeouts
            # -------------------------------------------------------------
            now_sec = current_time.seconds_nanoseconds()[0]
            
            lidar_age_ms = (now_sec - self.lidar_timestamp) * 1000 if self.lidar_timestamp > 0 else float('inf')
            imu_age_ms = (now_sec - self.imu_timestamp) * 1000 if self.imu_timestamp > 0 else float('inf')
            odom_age_ms = (now_sec - self.odom_timestamp) * 1000 if self.odom_timestamp > 0 else float('inf')
            
            sensors_ok = True
            emergency_reason = ""
            
            if lidar_age_ms > self.params.sensor_timeout_ms:
                sensors_ok = False
                emergency_reason = f"lidar_timeout ({lidar_age_ms:.0f}ms)"
            elif imu_age_ms > self.params.sensor_timeout_ms:
                sensors_ok = False
                emergency_reason = f"imu_timeout ({imu_age_ms:.0f}ms)"
            elif odom_age_ms > self.params.sensor_timeout_ms:
                sensors_ok = False
                emergency_reason = f"odom_timeout ({odom_age_ms:.0f}ms)"
            
            # -------------------------------------------------------------
            # Step 2: Update mission state machine
            # -------------------------------------------------------------
            mission_mode = self.mission.update(
                sensors_ok=sensors_ok,
                has_lidar=self.latest_lidar is not None,
                has_imu=self.latest_imu is not None,
                has_odom=self.latest_odom is not None
            )
            
            self.current_state.mode = mission_mode
            
            # -------------------------------------------------------------
            # Step 3: If not in autonomous mode, send safe command
            # -------------------------------------------------------------
            if mission_mode not in [String.DATA_AUTONOMOUS, String.DATA_TRACKING]:
                self.send_safe_command(mission_mode)
                self.current_state.emergency = (mission_mode == String.DATA_EMERGENCY)
                self.current_state.emergency_reason = emergency_reason if not sensors_ok else "manual_mode"
                return
            
            # -------------------------------------------------------------
            # Step 4: Process perception (with temporal smoothing)
            # -------------------------------------------------------------
            if self.latest_lidar is None:
                self.current_state.emergency = True
                self.current_state.emergency_reason = "no_lidar_data"
                self.send_safe_command(String.DATA_EMERGENCY)
                return
            
            perception_output = self.perception.process_scan(
                self.latest_lidar,
                dt
            )
            
            # -------------------------------------------------------------
            # Step 5: Update state estimate
            # -------------------------------------------------------------
            state_estimate = self.state_estimator.update(
                odom=self.latest_odom,
                imu=self.latest_imu,
                perception=perception_output,
                dt=dt
            )
            
            # -------------------------------------------------------------
            # Step 6: Plan trajectory
            # -------------------------------------------------------------
            trajectory = self.planner.plan(
                state=state_estimate,
                perception=perception_output,
                mission_mode=mission_mode
            )
            
            if trajectory is None or len(trajectory) < 2:
                self.get_logger().warn("Planning failed - using fallback")
                self.current_state.emergency = True
                self.current_state.emergency_reason = "planning_failure"
                self.send_safe_command(String.DATA_EMERGENCY)
                return
            
            # -------------------------------------------------------------
            # Step 7: Compute control with MPPI (continuous output)
            # -------------------------------------------------------------
            control_output = self.controller.compute_control(
                state=state_estimate,
                trajectory=trajectory,
                perception=perception_output,
                dt=dt
            )
            
            # -------------------------------------------------------------
            # Step 8: Send command
            # -------------------------------------------------------------
            cmd = CarCommand()
            cmd.header.stamp = current_time.to_msg()
            cmd.throttle = float(np.clip(control_output['throttle'], -0.3, 1.0))
            cmd.steering = float(np.clip(control_output['steering'], -0.5, 0.5))
            
            self.cmd_pub.publish(cmd)
            
            # Update state
            self.current_state.emergency = False
            self.current_state.emergency_reason = ""
            self.current_state.speed_cmd = cmd.throttle
            self.current_state.steering_cmd = cmd.steering
            self.current_state.clearance = perception_output.get('clearance_m', float('inf'))
            self.current_state.corridor_confidence = perception_output.get('corridor_confidence', 0.0)
            
            # Publish state
            self.state_pub.publish(self.current_state)
            
            # Debug info
            if self.params.debug_enabled:
                debug_msg = String()
                debug_msg.data = f"v={state_estimate['speed']:.2f} m/s | κ={state_estimate['curvature']:.3f} | clear={perception_output.get('clearance_m', 99.9):.2f}m | ttc={perception_output.get('ttc_s', 99.9):.2f}s"
                self.debug_info_pub.publish(debug_msg)
                
        except Exception as e:
            self.get_logger().error(f"Control loop error: {str(e)}")
            self.current_state.emergency = True
            self.current_state.emergency_reason = f"exception: {str(e)}"
            self.send_safe_command(String.DATA_EMERGENCY)
    
    # ========================================================================
    # Helper Methods
    # ========================================================================
    
    def send_safe_command(self, mode: str):
        """Send a safe command based on current mode."""
        cmd = CarCommand()
        cmd.header.stamp = self.get_clock().now().to_msg()
        
        if mode == String.DATA_EMERGENCY:
            cmd.throttle = -0.3  # Gentle brake
            cmd.steering = 0.0
        elif mode == String.DATA_IDLE:
            cmd.throttle = 0.0
            cmd.steering = 0.0
        else:
            cmd.throttle = 0.0
            cmd.steering = 0.0
        
        self.cmd_pub.publish(cmd)
        self.current_state.speed_cmd = cmd.throttle
        self.current_state.steering_cmd = cmd.steering
    
    def publish_diagnostics(self):
        """Publish detailed diagnostics for debugging."""
        now_sec = self.get_clock().now().seconds_nanoseconds()[0]
        
        lidar_age_ms = (now_sec - self.lidar_timestamp) * 1000 if self.lidar_timestamp > 0 else float('inf')
        imu_age_ms = (now_sec - self.imu_timestamp) * 1000 if self.imu_timestamp > 0 else float('inf')
        odom_age_ms = (now_sec - self.odom_timestamp) * 1000 if self.odom_timestamp > 0 else float('inf')
        
        diag = Diagnostics()
        diag.header.stamp = self.get_clock().now().to_msg()
        diag.emergency = self.current_state.emergency
        diag.reason = self.current_state.emergency_reason
        diag.throttle_cmd = self.current_state.speed_cmd
        diag.steering_cmd = self.current_state.steering_cmd
        diag.lidar_age_ms = float(lidar_age_ms)
        diag.imu_age_ms = float(imu_age_ms)
        diag.encoders_age_ms = float(odom_age_ms)
        diag.corridor_conf = self.current_state.corridor_confidence
        diag.clearance_m = self.current_state.clearance
        diag.ttc_s = self.current_state.clearance / max(self.current_state.speed_cmd, 0.1) if self.current_state.clearance < float('inf') else float('inf')
        
        self.diagnostics_pub.publish(diag)
        
        # Also log periodically
        self.get_logger().info(
            f"DIAGNOSTICS: emergency={diag.emergency} | reason={diag.reason} | "
            f"throttle={diag.throttle_cmd:.3f} | steering={diag.steering_cmd:.3f} | "
            f"lidar_age_ms={diag.lidar_age_ms:.1f} | imu_age_ms={diag.imu_age_ms:.1f} | "
            f"encoders_age_ms={diag.encoders_age_ms:.1f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = AutonomyNodeV2()
    
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
