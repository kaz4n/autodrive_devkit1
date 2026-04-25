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
from .models import CameraObservation, LidarObservation, MissionMode, Plan, SensorHeartbeat, VehicleState
from .params import StackConfig
from .perception import CameraPerception, LidarPerception
from .planning import ReactivePlanner
from .state_estimator import SimpleStateEstimator
from .math_utils import quaternion_to_yaw


class RoboRacerAutonomyNode(Node):
    """Path-centric RoboRacer autonomy stack.

    Runtime inputs remain competition-legal: LiDAR, front camera, IMU, wheel encoders, and
    steering feedback. A higher-level localizer can be connected through an optional custom pose
    topic if it itself is built from legal sensors only.
    """

    def __init__(self) -> None:
        super().__init__('roboracer_autonomy')
        self.declare_parameter('use_camera', True)
        self.declare_parameter('max_speed_mps', 100.0)
        self.declare_parameter('control_hz', 800.0)
        self.declare_parameter('raceline_csv_path', '')
        self.declare_parameter('external_pose_topic', '')

        self._config = StackConfig()
        self._config.camera.enabled = bool(self.get_parameter('use_camera').value)
        self._config.planner.max_speed_mps = float(self.get_parameter('max_speed_mps').value)
        self._config.planner.raceline_csv_path = str(self.get_parameter('raceline_csv_path').value)
        self._config.localization.external_pose_topic = str(self.get_parameter('external_pose_topic').value)
        self._control_hz = max(1.0, float(self.get_parameter('control_hz').value))

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._bridge = CvBridge()
        self._estimator = SimpleStateEstimator(self._config.vehicle, self._config.localization)
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
        self._latest_external_pose: VehicleState | None = None
        self._last_mode = MissionMode.BOOTSTRAP
        self._last_camera_processing_stamp = 0.0

        prefix = '/autodrive/roboracer_1'
        self._pub_throttle_cmd = self.create_publisher(Float32, f'{prefix}/throttle_command', qos)
        self._pub_steering_cmd = self.create_publisher(Float32, f'{prefix}/steering_command', qos)
        self._pub_pose_estimate = self.create_publisher(Odometry, '/roboracer_autonomy/pose_estimate', qos)
        self._pub_wheel_odom = self.create_publisher(Odometry, '/roboracer_autonomy/wheel_odom', qos)
        self._pub_mode = self.create_publisher(String, '/roboracer_autonomy/mission_mode', qos)
        self._pub_target_speed = self.create_publisher(Float32, '/roboracer_autonomy/target_speed', qos)
        self._pub_reference_source = self.create_publisher(String, '/roboracer_autonomy/reference_source', qos)

        self.create_subscription(LaserScan, f'{prefix}/lidar', self._on_lidar, qos)
        self.create_subscription(Imu, f'{prefix}/imu', self._on_imu, qos)
        self.create_subscription(JointState, f'{prefix}/left_encoder', self._on_left_encoder, qos)
        self.create_subscription(JointState, f'{prefix}/right_encoder', self._on_right_encoder, qos)
        self.create_subscription(Float32, f'{prefix}/steering', self._on_steering_feedback, qos)
        if self._config.camera.enabled:
            self.create_subscription(Image, f'{prefix}/front_camera', self._on_front_camera, qos)
        if self._config.localization.external_pose_topic:
            self.create_subscription(
                Odometry,
                self._config.localization.external_pose_topic,
                self._on_external_pose,
                qos,
            )

        self.create_timer(1.0 / self._control_hz, self._control_loop)
        self.get_logger().info(
            'RoboRacer autonomy stack ready. '
            f'use_camera={self._config.camera.enabled}, '
            f'max_speed_mps={self._config.planner.max_speed_mps:.2f}, '
            f'control_hz={self._control_hz:.1f}, '
            f'raceline_loaded={self._planner.has_raceline}, '
            f'external_pose_topic={self._config.localization.external_pose_topic or "<none>"}'
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
        if stamp - self._last_camera_processing_stamp < self._config.camera.process_period_s:
            self._heartbeats.camera_stamp = stamp
            return
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        except Exception as exc:  # pragma: no cover
            self.get_logger().warning(f'Camera conversion failed: {exc}')
            return
        self._latest_camera = self._camera.process(np.asarray(frame), stamp)
        self._last_camera_processing_stamp = stamp
        self._heartbeats.camera_stamp = stamp

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

    def _select_state(self, now: float, wheel_odom: VehicleState) -> VehicleState:
        ext = self._latest_external_pose
        if (
            ext is not None
            and self._config.localization.use_external_pose_if_available
            and now - ext.stamp <= self._config.localization.external_pose_timeout_s
        ):
            fused = VehicleState(
                stamp=ext.stamp,
                x=ext.x,
                y=ext.y,
                yaw=ext.yaw,
                speed=wheel_odom.speed if abs(ext.speed) < 1e-6 else ext.speed,
                yaw_rate=wheel_odom.yaw_rate if abs(ext.yaw_rate) < 1e-6 else ext.yaw_rate,
                steering_angle=wheel_odom.steering_angle,
                steering_normalized=wheel_odom.steering_normalized,
                linear_accel_x=wheel_odom.linear_accel_x,
                valid=True,
                confidence=ext.confidence,
                source=ext.source,
                covariance=ext.covariance,
            )
            return fused
        return wheel_odom

    def _control_loop(self) -> None:
        now = self._now()
        wheel_odom = self._estimator.predict(now)
        state = self._select_state(now, wheel_odom)
        mode = self._mission.decide(now, state, self._latest_lidar, self._latest_camera, self._heartbeats)
        self._latest_plan = self._planner.plan(state, self._latest_lidar, self._latest_camera, mode, now)
        command = self._controller.compute(self._latest_plan, state, now)
        self._publish_control(command.throttle, command.steering)
        self._publish_debug(wheel_odom, state, mode)

    def _publish_control(self, throttle: float, steering: float) -> None:
        throttle_msg = Float32()
        throttle_msg.data = float(throttle)
        steering_msg = Float32()
        steering_msg.data = float(steering)
        self._pub_throttle_cmd.publish(throttle_msg)
        self._pub_steering_cmd.publish(steering_msg)

    def _publish_debug(self, wheel_odom: VehicleState, state: VehicleState, mode: MissionMode) -> None:
        self._pub_pose_estimate.publish(self._make_odom_msg(state, frame_id='map' if state.source != 'wheel_odom' else 'roboracer_local'))
        self._pub_wheel_odom.publish(self._make_odom_msg(wheel_odom, frame_id='roboracer_local'))

        mode_msg = String()
        mode_msg.data = mode.value
        self._pub_mode.publish(mode_msg)

        speed_msg = Float32()
        speed_msg.data = float(self._latest_plan.target_speed)
        self._pub_target_speed.publish(speed_msg)

        ref_msg = String()
        ref_msg.data = self._latest_plan.reference_source
        self._pub_reference_source.publish(ref_msg)

        if mode != self._last_mode:
            self.get_logger().info(
                f'Mode transition: {self._last_mode.value} -> {mode.value} | '
                f'clearance={self._latest_lidar.forward_clearance:.2f} m | '
                f'ttc={self._latest_lidar.ttc:.2f} s | '
                f'target_speed={self._latest_plan.target_speed:.2f} m/s | '
                f'reference={self._latest_plan.reference_source} | '
                f'pose_source={state.source}'
            )
            self._last_mode = mode

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
