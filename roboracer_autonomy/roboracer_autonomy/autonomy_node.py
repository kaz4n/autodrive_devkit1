from __future__ import annotations

import numpy as np
import rclpy
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image, Imu, JointState, LaserScan
from std_msgs.msg import Float32, String

from .control import LowLevelController
from .mission import MissionManager
from .models import CameraObservation, LidarObservation, MissionMode, Plan, SensorHeartbeat
from .params import StackConfig
from .perception import CameraPerception, LidarPerception
from .planning import ReactivePlanner
from .state_estimator import SimpleStateEstimator


class RoboRacerAutonomyNode(Node):
    """Reactive autonomy stack for the RoboRacer competition.

    The node uses only runtime-legal topics: LiDAR, front camera, IMU, wheel encoders and
    steering feedback. Restricted IPS/odometry topics are intentionally
    ignored, in line with the technical guide.
    """

    def __init__(self) -> None:
        super().__init__('roboracer_autonomy')
        self.declare_parameter('use_camera', True)
        self.declare_parameter('max_speed_mps', 15.0)
        self.declare_parameter('control_hz', 80.0)

        self._config = StackConfig()
        self._config.camera.enabled = bool(self.get_parameter('use_camera').value)
        self._config.planner.max_speed_mps = float(self.get_parameter('max_speed_mps').value)
        self._config.mission.gap_enter_angle_rad = self._config.planner.gap_activation_angle_rad
        self._config.mission.gap_exit_angle_rad = 0.65 * self._config.mission.gap_enter_angle_rad
        self._control_hz = max(1.0, float(self.get_parameter('control_hz').value))

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._bridge = CvBridge()
        self._estimator = SimpleStateEstimator(self._config.vehicle)
        self._lidar = LidarPerception(self._config.lidar)
        self._camera = CameraPerception(self._config.camera)
        self._planner = ReactivePlanner(self._config.planner)
        self._controller = LowLevelController(
            self._config.vehicle,
            self._config.controller,
            nominal_max_speed_mps=self._config.planner.max_speed_mps,
        )
        self._mission = MissionManager(self._config.mission)

        self._heartbeats = SensorHeartbeat()
        self._latest_lidar = LidarObservation()
        self._latest_camera = CameraObservation()
        self._latest_plan = Plan()
        self._last_mode = MissionMode.BOOTSTRAP
        self._last_camera_processing_stamp = 0.0

        prefix = '/autodrive/roboracer_1'
        self._pub_throttle_cmd = self.create_publisher(Float32, f'{prefix}/throttle_command', qos)
        self._pub_steering_cmd = self.create_publisher(Float32, f'{prefix}/steering_command', qos)
        self._pub_state_estimate = self.create_publisher(Odometry, '/roboracer_autonomy/state_estimate', qos)
        self._pub_mode = self.create_publisher(String, '/roboracer_autonomy/mission_mode', qos)
        self._pub_target_speed = self.create_publisher(Float32, '/roboracer_autonomy/target_speed', qos)

        self.create_subscription(LaserScan, f'{prefix}/lidar', self._on_lidar, qos)
        self.create_subscription(Imu, f'{prefix}/imu', self._on_imu, qos)
        self.create_subscription(JointState, f'{prefix}/left_encoder', self._on_left_encoder, qos)
        self.create_subscription(JointState, f'{prefix}/right_encoder', self._on_right_encoder, qos)
        self.create_subscription(Float32, f'{prefix}/steering', self._on_steering_feedback, qos)
        if self._config.camera.enabled:
            self.create_subscription(Image, f'{prefix}/front_camera', self._on_front_camera, qos)

        self.create_timer(1.0 / self._control_hz, self._control_loop)
        self.get_logger().info(
            f'RoboRacer autonomy stack ready. use_camera={self._config.camera.enabled}, '
            f'max_speed_mps={self._config.planner.max_speed_mps:.2f}, '
            f'control_hz={self._control_hz:.1f}'
        )

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    @staticmethod
    def _stamp_to_seconds(msg_stamp) -> float:
        return float(msg_stamp.sec) + float(msg_stamp.nanosec) * 1e-9

    def _on_lidar(self, msg: LaserScan) -> None:
        stamp = self._stamp_to_seconds(msg.header.stamp) or self._now()
        speed = self._estimator.state.speed
        self._latest_lidar = self._lidar.process(
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

    def _on_front_camera(self, msg: Image) -> None:
        stamp = self._stamp_to_seconds(msg.header.stamp) or self._now()
        if stamp - self._last_camera_processing_stamp < 0.08:
            self._heartbeats.camera_stamp = stamp
            return
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        except Exception as exc:  # pragma: no cover - depends on runtime ROS transport.
            self.get_logger().warning(f'Camera conversion failed: {exc}')
            return
        self._latest_camera = self._camera.process(np.asarray(frame), stamp)
        self._last_camera_processing_stamp = stamp
        self._heartbeats.camera_stamp = stamp

    def _control_loop(self) -> None:
        now = self._now()
        state = self._estimator.predict(now)
        mode = self._mission.decide(now, state, self._latest_lidar, self._latest_camera, self._heartbeats)

        if mode == MissionMode.BOOTSTRAP:
            self._latest_plan = Plan(stamp=now, mode=mode, target_speed=0.0)
            self._publish_control(0.0, 0.0)
            self._publish_debug(state, mode)
            return

        self._latest_plan = self._planner.plan(state, self._latest_lidar, self._latest_camera, mode, now)
        command = self._controller.compute(self._latest_plan, state, now)
        self._publish_control(command.throttle, command.steering)
        self._publish_debug(state, mode)

    def _publish_control(self, throttle: float, steering: float) -> None:
        throttle_msg = Float32()
        throttle_msg.data = float(throttle)
        steering_msg = Float32()
        steering_msg.data = float(steering)
        self._pub_throttle_cmd.publish(throttle_msg)
        self._pub_steering_cmd.publish(steering_msg)

    def _publish_debug(self, state, mode: MissionMode) -> None:
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'roboracer_local'
        odom.child_frame_id = 'roboracer_local/base_link'
        odom.pose.pose.position.x = float(state.x)
        odom.pose.pose.position.y = float(state.y)
        odom.pose.pose.orientation.z = float(np.sin(0.5 * state.yaw))
        odom.pose.pose.orientation.w = float(np.cos(0.5 * state.yaw))
        odom.twist.twist.linear.x = float(state.speed)
        odom.twist.twist.angular.z = float(state.yaw_rate)
        self._pub_state_estimate.publish(odom)

        mode_msg = String()
        mode_msg.data = mode.value
        self._pub_mode.publish(mode_msg)

        speed_msg = Float32()
        speed_msg.data = float(self._latest_plan.target_speed)
        self._pub_target_speed.publish(speed_msg)

        if mode != self._last_mode:
            self.get_logger().info(
                f'Mode transition: {self._last_mode.value} -> {mode.value} | '
                f'clearance={self._latest_lidar.forward_clearance:.2f} m | '
                f'ttc={self._latest_lidar.ttc:.2f} s | '
                f'target_speed={self._latest_plan.target_speed:.2f} m/s'
            )
            self._last_mode = mode


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
