from __future__ import annotations

import json
from typing import Optional

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Imu, JointState, LaserScan
from std_msgs.msg import Float32, String

from .free_space_mpc import FreeSpaceMPC
from .math_utils import quaternion_to_yaw
from .models import ControlCommand, SensorHeartbeat, TrackBoundaries, VehicleState
from .params import StackConfig
from .perception import LidarTrackExtractor
from .state_estimator import WheelOdometryEstimator


class RoboRacerAutonomyNode(Node):
    def __init__(self) -> None:
        super().__init__('roboracer_autonomy')
        self.declare_parameter('max_speed_mps', 10.0)
        self.declare_parameter('control_hz', 15.0)
        self.declare_parameter('external_pose_topic', '')
        self.declare_parameter('use_external_pose', False)
        self.declare_parameter('vehicle_prefix', '/autodrive/roboracer_1')

        self._config = StackConfig()
        self._config.mpc.max_speed_mps = float(self.get_parameter('max_speed_mps').value)
        self._config.localization.external_pose_topic = str(self.get_parameter('external_pose_topic').value)
        self._config.localization.use_external_pose_if_available = bool(self.get_parameter('use_external_pose').value)
        self._control_hz = max(1.0, float(self.get_parameter('control_hz').value))
        self._prefix = str(self.get_parameter('vehicle_prefix').value)

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._estimator = WheelOdometryEstimator(self._config.vehicle, self._config.localization)
        self._perception = LidarTrackExtractor(self._config.lidar)
        self._mpc = FreeSpaceMPC(self._config.vehicle, self._config.mpc)

        now = self._now()
        self._heartbeats = SensorHeartbeat(
            lidar_stamp=now,
            imu_stamp=now,
            left_encoder_stamp=now,
            right_encoder_stamp=now,
            steering_stamp=now,
            external_pose_stamp=now,
        )
        self._latest_lidar = TrackBoundaries()
        self._latest_external_pose: Optional[VehicleState] = None
        self._last_command = ControlCommand()
        self._solver_failure_count = 0
        self._log_cycle_counter = 0

        self._pub_throttle = self.create_publisher(Float32, f'{self._prefix}/throttle_command', qos)
        self._pub_steering = self.create_publisher(Float32, f'{self._prefix}/steering_command', qos)
        self._pub_pose = self.create_publisher(Odometry, '/roboracer_autonomy/pose_estimate', qos)
        self._pub_wheel_odom = self.create_publisher(Odometry, '/roboracer_autonomy/wheel_odom', qos)
        self._pub_track_id = self.create_publisher(String, '/roboracer_autonomy/track_id', qos)
        self._pub_map_status = self.create_publisher(String, '/roboracer_autonomy/map_status', qos)
        self._pub_controller_status = self.create_publisher(String, '/roboracer_autonomy/controller_status', qos)
        self._pub_reference_source = self.create_publisher(String, '/roboracer_autonomy/reference_source', qos)
        self._pub_target_speed = self.create_publisher(Float32, '/roboracer_autonomy/target_speed', qos)

        self.create_subscription(LaserScan, f'{self._prefix}/lidar', self._on_lidar, qos)
        self.create_subscription(Imu, f'{self._prefix}/imu', self._on_imu, qos)
        self.create_subscription(JointState, f'{self._prefix}/left_encoder', self._on_left_encoder, qos)
        self.create_subscription(JointState, f'{self._prefix}/right_encoder', self._on_right_encoder, qos)
        self.create_subscription(Float32, f'{self._prefix}/steering', self._on_steering_feedback, qos)
        if self._config.localization.external_pose_topic:
            self.create_subscription(Odometry, self._config.localization.external_pose_topic, self._on_external_pose, qos)

        self.create_timer(1.0 / self._control_hz, self._control_loop)
        self.get_logger().info(
            'Free-space MPC stack ready. '
            f'control_hz={self._control_hz:.1f}, '
            f'max_speed_mps={self._config.mpc.max_speed_mps:.2f}, '
            f'external_pose_topic={self._config.localization.external_pose_topic or "<none>"}'
        )

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1.0e-9

    @staticmethod
    def _stamp_to_seconds(msg_stamp) -> float:
        return float(msg_stamp.sec) + float(msg_stamp.nanosec) * 1.0e-9

    def _on_lidar(self, msg: LaserScan) -> None:
        stamp = self._stamp_to_seconds(msg.header.stamp) or self._now()
        speed = self._estimator.state.speed
        self._latest_lidar = self._perception.process(
            np.asarray(msg.ranges, dtype=float),
            angle_min=float(msg.angle_min),
            angle_increment=float(msg.angle_increment),
            speed_mps=float(speed),
            stamp=stamp,
        )
        self._heartbeats.lidar_stamp = stamp

    def _on_imu(self, msg: Imu) -> None:
        stamp = self._stamp_to_seconds(msg.header.stamp) or self._now()
        self._estimator.update_imu(
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
            msg.angular_velocity.z,
            msg.linear_acceleration.x,
            stamp,
        )
        self._heartbeats.imu_stamp = stamp

    def _on_left_encoder(self, msg: JointState) -> None:
        if not msg.position:
            return
        stamp = self._stamp_to_seconds(msg.header.stamp) or self._now()
        self._estimator.update_left_encoder(float(msg.position[0]), stamp)
        self._heartbeats.left_encoder_stamp = stamp

    def _on_right_encoder(self, msg: JointState) -> None:
        if not msg.position:
            return
        stamp = self._stamp_to_seconds(msg.header.stamp) or self._now()
        self._estimator.update_right_encoder(float(msg.position[0]), stamp)
        self._heartbeats.right_encoder_stamp = stamp

    def _on_steering_feedback(self, msg: Float32) -> None:
        stamp = self._now()
        self._estimator.update_steering(float(msg.data), stamp)
        self._heartbeats.steering_stamp = stamp

    def _on_external_pose(self, msg: Odometry) -> None:
        stamp = self._stamp_to_seconds(msg.header.stamp) or self._now()
        yaw = quaternion_to_yaw(
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        )
        self._latest_external_pose = VehicleState(
            stamp=stamp,
            x=float(msg.pose.pose.position.x),
            y=float(msg.pose.pose.position.y),
            yaw=float(yaw),
            speed=float(msg.twist.twist.linear.x),
            yaw_rate=float(msg.twist.twist.angular.z),
            steering_angle=float(self._estimator.state.steering_angle),
            steering_normalized=float(self._estimator.state.steering_normalized),
            linear_accel_x=float(self._estimator.state.linear_accel_x),
            valid=True,
            confidence=0.95,
            source='external_pose',
        )
        self._heartbeats.external_pose_stamp = stamp

    def _select_pose(self, now: float, wheel_odom: VehicleState) -> VehicleState:
        ext = self._latest_external_pose
        if ext is not None and self._config.localization.use_external_pose_if_available and now - ext.stamp <= self._config.localization.external_pose_timeout_s:
            return VehicleState(
                stamp=ext.stamp,
                x=ext.x,
                y=ext.y,
                yaw=ext.yaw,
                speed=wheel_odom.speed if abs(ext.speed) < 1.0e-6 else ext.speed,
                yaw_rate=wheel_odom.yaw_rate if abs(ext.yaw_rate) < 1.0e-6 else ext.yaw_rate,
                steering_angle=wheel_odom.steering_angle,
                steering_normalized=wheel_odom.steering_normalized,
                linear_accel_x=wheel_odom.linear_accel_x,
                valid=True,
                confidence=max(wheel_odom.confidence, ext.confidence),
                source=ext.source,
            )
        return wheel_odom

    def _control_loop(self) -> None:
        now = self._now()
        wheel_odom = self._estimator.predict(now)
        state = self._select_pose(now, wheel_odom)

        if not state.valid:
            command = self._immediate_brake(now, reason='state_unavailable')
            self._last_command = command
            self._publish_all(wheel_odom, state, command)
            self._log_diagnostics(now)
            return

        safety_cmd = self._precheck(now)
        if safety_cmd is not None:
            self._last_command = safety_cmd
            self._publish_all(wheel_odom, state, safety_cmd)
            self._log_diagnostics(now)
            return

        command = self._mpc.solve(
            state,
            self._latest_lidar.left_boundary,
            self._latest_lidar.right_boundary,
            stamp=now,
        )
        command = self._postcheck_solver_failures(command, now)
        self._last_command = command
        self._publish_all(wheel_odom, state, command)
        self._log_diagnostics(now)

    def _log_diagnostics(self, now: float) -> None:
        self._log_cycle_counter += 1
        log_interval = max(1, int(self._control_hz))
        if self._log_cycle_counter % log_interval != 0:
            return
        
        lidar_age = now - self._heartbeats.lidar_stamp
        imu_age = now - self._heartbeats.imu_stamp
        le_age = now - self._heartbeats.left_encoder_stamp
        re_age = now - self._heartbeats.right_encoder_stamp
        
        self.get_logger().info(
            f'DIAGNOSTICS: emergency={self._last_command.emergency} | '
            f'reason={self._last_command.reason} | '
            f'throttle={self._last_command.throttle:.3f} | '
            f'steering={self._last_command.steering:.3f} | '
            f'lidar_age_ms={lidar_age*1000:.1f} | '
            f'imu_age_ms={imu_age*1000:.1f} | '
            f'encoders_age_ms={max(le_age,re_age)*1000:.1f} | '
            f'corridor_conf={self._latest_lidar.confidence:.2f} | '
            f'clearance_m={self._latest_lidar.forward_clearance:.2f} | '
            f'ttc_s={self._latest_lidar.ttc:.2f}'
        )

    def _precheck(self, now: float) -> Optional[ControlCommand]:
        if now - self._heartbeats.lidar_stamp > self._config.mpc.stale_lidar_timeout_s:
            return self._immediate_brake(now, reason='stale_lidar')
        if self._latest_lidar.forward_clearance < self._config.mpc.emergency_clearance_m:
            return self._immediate_brake(now, reason='hard_clearance')
        if self._latest_lidar.ttc < self._config.mpc.emergency_ttc_s:
            return self._immediate_brake(now, reason='hard_ttc')
        if not self._latest_lidar.has_corridor():
            return self._immediate_brake(now, reason='empty_corridor')
        return None

    def _postcheck_solver_failures(self, command: ControlCommand, now: float) -> ControlCommand:
        solver_failure = command.emergency and command.reason in {
            'solver_failure',
            'solver_unavailable',
            'invalid_corridor',
            'weak_corridor',
        }
        if not solver_failure:
            self._solver_failure_count = 0
            return command

        self._solver_failure_count += 1
        command.metadata['consecutive_solver_failures'] = float(self._solver_failure_count)
        command.steering = self._last_command.steering
        command.target_speed = 0.0
        if self._solver_failure_count >= self._config.mpc.fallback_hold_cycles:
            command.throttle = self._config.mpc.fallback_brake_command
        else:
            command.throttle = 0.0
        command.stamp = now
        return command

    def _immediate_brake(self, stamp: float, reason: str) -> ControlCommand:
        self._solver_failure_count = 0
        return ControlCommand(
            stamp=stamp,
            throttle=self._config.mpc.fallback_brake_command,
            steering=self._last_command.steering,
            target_speed=0.0,
            emergency=True,
            reason=reason,
        )

    def _publish_all(self, wheel_odom: VehicleState, state: VehicleState, command: ControlCommand) -> None:
        throttle_msg = Float32(); throttle_msg.data = float(command.throttle)
        steering_msg = Float32(); steering_msg.data = float(command.steering)
        self._pub_throttle.publish(throttle_msg)
        self._pub_steering.publish(steering_msg)

        pose_frame = 'map' if state.source != 'wheel_odom' else 'roboracer_local'
        self._pub_pose.publish(self._make_odom_msg(state, frame_id=pose_frame))
        self._pub_wheel_odom.publish(self._make_odom_msg(wheel_odom, frame_id='roboracer_local'))

        track_msg = String(); track_msg.data = ''
        self._pub_track_id.publish(track_msg)

        map_status = {
            'map_mode': 'disabled',
            'reference': 'free_space_mpc_live_corridor',
            'live_confidence': float(self._latest_lidar.confidence),
        }
        map_status_msg = String(); map_status_msg.data = json.dumps(map_status, separators=(',', ':'), sort_keys=True)
        self._pub_map_status.publish(map_status_msg)

        solver = self._mpc.last_solver_debug
        ctrl_status = {
            'controller': 'free_space_mpc',
            'emergency': bool(command.emergency),
            'reason': command.reason,
            'solve_success': bool(solver.success),
            'solve_time_ms': float(solver.solve_time_ms),
            'solve_iterations': int(solver.iterations),
            'target_speed_mps': float(command.target_speed),
            'throttle_cmd': float(command.throttle),
            'steering_cmd': float(command.steering),
            'forward_clearance_m': float(self._latest_lidar.forward_clearance),
            'ttc_s': float(self._latest_lidar.ttc),
            'consecutive_solver_failures': int(self._solver_failure_count),
            'reference': 'free_space_mpc_live_corridor',
        }
        ctrl_status_msg = String(); ctrl_status_msg.data = json.dumps(ctrl_status, separators=(',', ':'), sort_keys=True)
        self._pub_controller_status.publish(ctrl_status_msg)

        ref_msg = String(); ref_msg.data = 'free_space_mpc_live_corridor'
        self._pub_reference_source.publish(ref_msg)
        target_speed_msg = Float32(); target_speed_msg.data = float(command.target_speed)
        self._pub_target_speed.publish(target_speed_msg)

    def _make_odom_msg(self, state: VehicleState, frame_id: str) -> Odometry:
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = frame_id
        odom.child_frame_id = f'{frame_id}/base_link'
        odom.pose.pose.position.x = float(state.x)
        odom.pose.pose.position.y = float(state.y)
        odom.pose.pose.orientation.z = float(np.sin(0.5 * state.yaw))
        odom.pose.pose.orientation.w = float(np.cos(0.5 * state.yaw))
        odom.twist.twist.linear.x = float(state.speed)
        odom.twist.twist.angular.z = float(state.yaw_rate)
        return odom


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RoboRacerAutonomyNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
