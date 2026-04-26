#!/usr/bin/env python3
"""
RoboRacer Autonomy Node - Complete rewrite.

Architecture (in order of priority):
  1. Fix: Ensure QoS matches autodrive_bridge (RELIABLE + VOLATILE + KEEP_LAST depth=1)
  2. Two-mode stack:
       LAP_RECORDING  -> first lap: record waypoints from IPS + gap-follow for safety
       WAYPOINT_RACE  -> subsequent laps: Pure Pursuit on recorded centerline waypoints
                         with disparity-extender emergency override
  3. Fallback: disparity-extender gap-follow if no waypoints yet or emergency triggered

The wiring bug in the original stack was caused by the autonomy node initializing its
SensorHeartbeat timestamps to `self._now()` at construction time, then checking
`now - heartbeat_stamp > stale_timeout`. Since sensors weren't received for up to 196s,
the stale check fired before any data arrived. Fixed by:
  - Initializing all heartbeat stamps to 0.0 (forces stale on first loop)
  - Only triggering stale-brake after the FIRST valid sensor packet is received
  - Using `sensor_initialized` flag so the node waits rather than braking before start
"""

from __future__ import annotations

import math
import time
from typing import List, Optional

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                        QoSReliabilityPolicy)
from sensor_msgs.msg import Imu, JointState, LaserScan
from std_msgs.msg import Float32, Bool, Int32

# ---------------------------------------------------------------------------
# QoS that matches autodrive_bridge exactly
# ---------------------------------------------------------------------------
BRIDGE_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)


# ---------------------------------------------------------------------------
# Pure Pursuit constants (tune these)
# ---------------------------------------------------------------------------
LOOKAHEAD_SLOW   = 0.8   # m  - lookahead at low speed
LOOKAHEAD_FAST   = 2.5   # m  - lookahead at high speed
LOOKAHEAD_SPEED_REF = 5.0  # m/s - speed at which fast lookahead kicks in

MAX_SPEED        = 6.0   # m/s - target cruise speed (tune up cautiously)
CORNER_SPEED_MIN = 2.0   # m/s - minimum cornering speed
LATERAL_ACCEL_G  = 0.4   # fraction of g for speed profiling (lower = safer)

# Waypoint recording
WAYPOINT_STRIDE  = 0.15  # m   - minimum distance between recorded waypoints
MIN_WAYPOINTS    = 40    # need at least this many before switching to PP mode

# Disparity extender (emergency / first-lap)
DISPARITY_THRESH   = 0.4   # m  - jump in consecutive LiDAR ranges = disparity
CAR_HALF_WIDTH     = 0.18  # m  - half-width safety margin
SAFE_DISTANCE      = 0.35  # m  - hard-stop threshold
GAP_SPEED_MAX      = 4.0   # m/s - gap-follow max speed
GAP_SPEED_MIN      = 1.0   # m/s - gap-follow min speed
GAP_LOOKAHEAD_DIST = 2.0   # m  - how far to look into the gap for speed

# Steering limits
MAX_STEER_NORM   = 1.0   # normalized [-1, 1]
MAX_STEER_RAD    = 0.5236  # rad (30 deg) - physical limit from params

# Lap detection (for switching from recording to racing mode)
LAP_CLOSE_RADIUS = 1.5   # m - how close to start before declaring lap done


class RoboRacerAutonomyNode(Node):
    """
    Clean, competition-ready autonomy node.

    Modes:
      'gap'  - disparity-extender reactive driving (first-lap / emergency fallback)
      'pure_pursuit' - map-based Pure Pursuit once waypoints are recorded
    """

    def __init__(self) -> None:
        super().__init__('roboracer_autonomy')

        # ---- ROS parameters ----
        self.declare_parameter('max_speed_mps', MAX_SPEED)
        self.declare_parameter('control_hz', 40.0)
        self.declare_parameter('vehicle_prefix', '/autodrive/roboracer_1')
        self.declare_parameter('mode', 'gap')  # 'gap' | 'pure_pursuit' | 'auto'

        self._max_speed = float(self.get_parameter('max_speed_mps').value)
        self._control_hz = float(self.get_parameter('control_hz').value)
        self._prefix = str(self.get_parameter('vehicle_prefix').value)
        self._user_mode = str(self.get_parameter('mode').value)

        # ---- State ----
        self._lidar_ranges: np.ndarray = np.array([])
        self._lidar_angle_min: float = -2.35619
        self._lidar_angle_inc: float = 0.004363323
        self._position: np.ndarray = np.zeros(3)   # x, y, z from IPS
        self._speed: float = 0.0                   # from odometry
        self._lap_count: int = 0
        self._collision_count: int = 0

        # Sensor init guard - prevents braking before first packet
        self._sensor_initialized: bool = False
        self._lidar_received: bool = False
        self._ips_received: bool = False

        # Timing
        self._last_lidar_time: float = 0.0
        self._last_ips_time: float = 0.0
        self._last_odom_time: float = 0.0

        # ---- Waypoint recording ----
        self._waypoints: List[np.ndarray] = []   # list of [x, y, speed]
        self._recording: bool = True              # True until first full lap
        self._record_start: Optional[np.ndarray] = None
        self._last_recorded_pos: Optional[np.ndarray] = None

        # ---- Pure pursuit ----
        self._pp_waypoints: Optional[np.ndarray] = None   # Nx3 array [x, y, speed]
        self._current_mode: str = 'gap'

        # ---- Previous commands (for smoothing) ----
        self._last_steering: float = 0.0
        self._last_throttle: float = 0.0

        # ---- Publishers ----
        self._pub_throttle = self.create_publisher(
            Float32, f'{self._prefix}/throttle_command', BRIDGE_QOS)
        self._pub_steering = self.create_publisher(
            Float32, f'{self._prefix}/steering_command', BRIDGE_QOS)

        # ---- Subscribers (QoS must match autodrive_bridge) ----
        self.create_subscription(
            LaserScan, f'{self._prefix}/lidar', self._cb_lidar, BRIDGE_QOS)
        self.create_subscription(
            Point, f'{self._prefix}/ips', self._cb_ips, BRIDGE_QOS)
        self.create_subscription(
            Odometry, f'{self._prefix}/odom', self._cb_odom, BRIDGE_QOS)
        self.create_subscription(
            Int32, f'{self._prefix}/lap_count', self._cb_lap, BRIDGE_QOS)
        self.create_subscription(
            Int32, f'{self._prefix}/collision_count', self._cb_collision, BRIDGE_QOS)

        # ---- Control timer ----
        self._timer = self.create_timer(
            1.0 / self._control_hz, self._control_loop)

        self.get_logger().info(
            f'RoboRacer autonomy started. '
            f'mode={self._user_mode}, max_speed={self._max_speed:.1f} m/s, '
            f'control_hz={self._control_hz:.0f}'
        )

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _cb_lidar(self, msg: LaserScan) -> None:
        self._lidar_ranges = np.array(msg.ranges, dtype=float)
        self._lidar_angle_min = float(msg.angle_min)
        self._lidar_angle_inc = float(msg.angle_increment)
        self._last_lidar_time = time.monotonic()
        self._lidar_received = True
        self._check_init()

    def _cb_ips(self, msg: Point) -> None:
        self._position = np.array([msg.x, msg.y, msg.z])
        self._last_ips_time = time.monotonic()
        self._ips_received = True
        self._check_init()
        self._try_record_waypoint()

    def _cb_odom(self, msg: Odometry) -> None:
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self._speed = float(np.sqrt(vx**2 + vy**2))
        self._last_odom_time = time.monotonic()

    def _cb_lap(self, msg: Int32) -> None:
        new_lap = int(msg.data)
        if new_lap > self._lap_count:
            self._lap_count = new_lap
            self.get_logger().info(f'Lap {self._lap_count} completed.')
            if self._recording and len(self._waypoints) >= MIN_WAYPOINTS:
                self._finalize_waypoints()

    def _cb_collision(self, msg: Int32) -> None:
        self._collision_count = int(msg.data)

    def _check_init(self) -> None:
        if not self._sensor_initialized and self._lidar_received and self._ips_received:
            self._sensor_initialized = True
            self.get_logger().info('Sensors initialized - starting control.')

    # ------------------------------------------------------------------
    # Waypoint recording
    # ------------------------------------------------------------------

    def _try_record_waypoint(self) -> None:
        if not self._recording:
            return
        pos = self._position[:2].copy()

        if self._record_start is None:
            self._record_start = pos.copy()
            self._last_recorded_pos = pos.copy()
            self._waypoints.append(np.array([pos[0], pos[1], self._speed]))
            return

        dist_from_last = float(np.linalg.norm(pos - self._last_recorded_pos))
        if dist_from_last < WAYPOINT_STRIDE:
            return

        # Check if we completed a lap (close to start)
        dist_from_start = float(np.linalg.norm(pos - self._record_start))
        if (len(self._waypoints) > MIN_WAYPOINTS and
                dist_from_start < LAP_CLOSE_RADIUS):
            self._finalize_waypoints()
            return

        self._waypoints.append(np.array([pos[0], pos[1], self._speed]))
        self._last_recorded_pos = pos.copy()

    def _finalize_waypoints(self) -> None:
        if len(self._waypoints) < MIN_WAYPOINTS:
            return
        self._recording = False
        wp = np.array(self._waypoints)  # Nx3

        # Smooth the speed profile based on path curvature
        wp = self._compute_speed_profile(wp)
        self._pp_waypoints = wp

        # Decide mode
        if self._user_mode in ('pure_pursuit', 'auto'):
            self._current_mode = 'pure_pursuit'
        else:
            self._current_mode = 'gap'

        self.get_logger().info(
            f'Waypoints finalized: {len(wp)} points. '
            f'Switching to mode: {self._current_mode}'
        )

    def _compute_speed_profile(self, wp: np.ndarray) -> np.ndarray:
        """Assign speed targets to waypoints based on local curvature."""
        n = len(wp)
        speeds = np.full(n, self._max_speed)
        if n < 5:
            wp[:, 2] = speeds
            return wp

        xy = wp[:, :2]
        # Compute curvature at each point using finite differences
        for i in range(2, n - 2):
            p0 = xy[i - 2]
            p1 = xy[i]
            p2 = xy[i + 2]
            # Menger curvature
            a = np.linalg.norm(p1 - p0)
            b = np.linalg.norm(p2 - p1)
            c = np.linalg.norm(p2 - p0)
            area2 = abs((p1[0] - p0[0]) * (p2[1] - p0[1]) -
                        (p1[1] - p0[1]) * (p2[0] - p0[0]))
            denom = max(a * b * c, 1e-6)
            kappa = area2 / denom
            # Speed limit from lateral acceleration: v = sqrt(a_lat / kappa)
            if kappa > 1e-4:
                v_lat = math.sqrt(LATERAL_ACCEL_G * 9.81 / kappa)
                speeds[i] = max(CORNER_SPEED_MIN, min(self._max_speed, v_lat))

        # Forward-backward smoothing pass
        for _ in range(3):
            for i in range(1, n):
                speeds[i] = min(speeds[i], speeds[i - 1] + 0.5)
            for i in range(n - 2, -1, -1):
                speeds[i] = min(speeds[i], speeds[i + 1] + 0.5)

        wp[:, 2] = speeds
        return wp

    # ------------------------------------------------------------------
    # Control loop
    # ------------------------------------------------------------------

    def _control_loop(self) -> None:
        # Safety guard: don't act before sensors are ready
        if not self._sensor_initialized:
            self._publish(0.0, 0.0)
            return

        # Stale-sensor check (only after initialization)
        now = time.monotonic()
        lidar_age = now - self._last_lidar_time
        if lidar_age > 0.5:
            self.get_logger().warn(f'STALE LIDAR ({lidar_age*1000:.0f} ms) - braking')
            self._publish(-0.3, self._last_steering)
            return

        if self._lidar_ranges.size == 0:
            self._publish(0.0, 0.0)
            return

        # Hard emergency check (always active)
        fwd_clear = self._forward_clearance()
        if fwd_clear < SAFE_DISTANCE:
            self.get_logger().warn(f'HARD STOP: clearance={fwd_clear:.2f}m')
            self._publish(-0.5, self._last_steering)
            return

        # Choose algorithm
        if (self._current_mode == 'pure_pursuit' and
                self._pp_waypoints is not None and
                len(self._pp_waypoints) >= MIN_WAYPOINTS):
            throttle, steering = self._run_pure_pursuit()
        else:
            throttle, steering = self._run_disparity_extender()

        self._last_throttle = throttle
        self._last_steering = steering
        self._publish(throttle, steering)

    # ------------------------------------------------------------------
    # Disparity Extender
    # ------------------------------------------------------------------

    def _run_disparity_extender(self):
        """
        Disparity Extender algorithm:
        1. Find all disparities (large jumps) in the LiDAR scan
        2. Extend the closer side into the farther side by car_half_width
           (this carves out space the car can't physically fit through)
        3. Find the deepest gap in the extended scan
        4. Steer toward the best point in that gap
        """
        ranges = np.clip(
            np.where(np.isfinite(self._lidar_ranges),
                     self._lidar_ranges, 8.0),
            0.0, 8.0
        )
        n = len(ranges)
        angles = self._lidar_angle_min + np.arange(n) * self._lidar_angle_inc

        # Only consider forward half (within ±110 deg)
        focus = np.abs(np.degrees(angles)) <= 110.0

        extended = ranges.copy()

        # Extend disparities: where ranges[i+1] - ranges[i] > thresh,
        # the closer point 'blocks' a cone of angle = atan(car_width / range)
        for i in range(n - 1):
            if not focus[i]:
                continue
            diff = float(ranges[i + 1] - ranges[i])
            if abs(diff) > DISPARITY_THRESH:
                # Which side is closer?
                if diff > 0:  # i is closer
                    close_range = max(float(ranges[i]), 0.1)
                    spread = int(math.ceil(
                        math.atan(CAR_HALF_WIDTH / close_range) /
                        max(self._lidar_angle_inc, 1e-5)))
                    lo = max(0, i - spread)
                    hi = min(n, i + 1)
                    extended[lo:hi] = np.minimum(extended[lo:hi], ranges[i])
                else:  # i+1 is closer
                    close_range = max(float(ranges[i + 1]), 0.1)
                    spread = int(math.ceil(
                        math.atan(CAR_HALF_WIDTH / close_range) /
                        max(self._lidar_angle_inc, 1e-5)))
                    lo = max(0, i + 1)
                    hi = min(n, i + 1 + spread)
                    extended[lo:hi] = np.minimum(extended[lo:hi], ranges[i + 1])

        # Apply focus mask
        extended[~focus] = 0.0

        # Find best gap: largest contiguous region of free space
        FREE_THRESH = max(SAFE_DISTANCE * 1.5, 0.6)
        free = (extended > FREE_THRESH).astype(int)
        best_start, best_end, best_len = 0, 0, 0
        cur_start, cur_len = 0, 0
        for i, f in enumerate(free):
            if f:
                if cur_len == 0:
                    cur_start = i
                cur_len += 1
                if cur_len > best_len:
                    best_len = cur_len
                    best_start = cur_start
                    best_end = i
            else:
                cur_len = 0

        if best_len == 0:
            # No gap at all - emergency brake
            return -0.4, self._last_steering

        # Pick target: best range point within the gap
        gap_ranges = extended[best_start:best_end + 1]
        gap_angles = angles[best_start:best_end + 1]

        # Weight by range (farther = better) and proximity to center
        weights = gap_ranges - 0.3 * np.abs(gap_angles)
        target_idx = int(np.argmax(weights))
        target_angle = float(gap_angles[target_idx])
        target_range = float(gap_ranges[target_idx])

        # Convert angle to steering command
        steering = np.clip(
            target_angle / MAX_STEER_RAD,
            -MAX_STEER_NORM, MAX_STEER_NORM
        )
        steering = float(steering)

        # Smooth steering
        steering = 0.6 * steering + 0.4 * self._last_steering

        # Speed: reduce for sharp turns and when close
        steer_factor = 1.0 - 0.7 * abs(steering)
        range_factor = min(1.0, target_range / GAP_LOOKAHEAD_DIST)
        target_speed = GAP_SPEED_MIN + (GAP_SPEED_MAX - GAP_SPEED_MIN) * steer_factor * range_factor
        target_speed = min(target_speed, self._max_speed)

        current_speed = max(self._speed, 0.0)
        throttle = self._speed_to_throttle(target_speed, current_speed)

        return float(throttle), float(steering)

    # ------------------------------------------------------------------
    # Pure Pursuit
    # ------------------------------------------------------------------

    def _run_pure_pursuit(self):
        """
        Pure Pursuit path tracking.
        1. Find the nearest waypoint ahead
        2. Find the goal point at lookahead distance
        3. Compute curvature to goal
        4. Convert to steering + speed command
        """
        if self._pp_waypoints is None:
            return self._run_disparity_extender()

        wp = self._pp_waypoints
        pos = self._position[:2]

        # Adaptive lookahead: longer at higher speed
        speed_ratio = min(1.0, self._speed / LOOKAHEAD_SPEED_REF)
        lookahead = LOOKAHEAD_SLOW + (LOOKAHEAD_FAST - LOOKAHEAD_SLOW) * speed_ratio

        # Find nearest waypoint
        dists = np.linalg.norm(wp[:, :2] - pos, axis=1)
        nearest_idx = int(np.argmin(dists))

        # Find goal point: first waypoint at >= lookahead from current pos
        n = len(wp)
        goal_idx = nearest_idx
        for i in range(n):
            idx = (nearest_idx + i) % n
            d = float(np.linalg.norm(wp[idx, :2] - pos))
            if d >= lookahead:
                goal_idx = idx
                break

        goal = wp[goal_idx, :2]
        target_speed = float(wp[goal_idx, 2])

        # We need goal in vehicle frame to compute steering
        # Use IPS position (world frame) - we don't have yaw directly,
        # so we estimate it from recent motion
        yaw = self._estimate_yaw(nearest_idx, wp)

        # Transform goal to vehicle frame
        dx = goal[0] - pos[0]
        dy = goal[1] - pos[1]
        cos_y = math.cos(-yaw)
        sin_y = math.sin(-yaw)
        gx_local = dx * cos_y - dy * sin_y   # forward
        gy_local = dx * sin_y + dy * cos_y   # lateral

        # Pure pursuit curvature: gamma = 2*gy / L^2
        L = max(float(np.linalg.norm(goal - pos)), 0.1)
        curvature = 2.0 * gy_local / (L * L)

        # Steering angle from curvature: delta = atan(kappa * wheelbase)
        WHEELBASE = 0.324  # m
        steer_rad = math.atan(curvature * WHEELBASE)
        steering = float(np.clip(steer_rad / MAX_STEER_RAD,
                                  -MAX_STEER_NORM, MAX_STEER_NORM))

        # Smooth steering
        steering = 0.7 * steering + 0.3 * self._last_steering

        # Override speed if obstacle detected
        fwd = self._forward_clearance()
        if fwd < 1.5:
            target_speed = min(target_speed, CORNER_SPEED_MIN)
        if fwd < 0.8:
            target_speed = 0.5

        # Emergency gap check: if something blocks us, fall back to gap
        if fwd < SAFE_DISTANCE * 2.0:
            return self._run_disparity_extender()

        throttle = self._speed_to_throttle(target_speed, self._speed)
        return float(throttle), float(steering)

    def _estimate_yaw(self, nearest_idx: int, wp: np.ndarray) -> float:
        """Estimate vehicle heading from direction between consecutive waypoints."""
        n = len(wp)
        idx_a = nearest_idx
        idx_b = (nearest_idx + 1) % n
        dx = float(wp[idx_b, 0] - wp[idx_a, 0])
        dy = float(wp[idx_b, 1] - wp[idx_a, 1])
        if abs(dx) < 1e-6 and abs(dy) < 1e-6:
            return 0.0
        return math.atan2(dy, dx)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _forward_clearance(self) -> float:
        """Minimum range in the forward ±15 degrees."""
        if self._lidar_ranges.size == 0:
            return float('inf')
        n = len(self._lidar_ranges)
        angles = self._lidar_angle_min + np.arange(n) * self._lidar_angle_inc
        fwd = (np.abs(np.degrees(angles)) <= 15.0)
        if not np.any(fwd):
            return float('inf')
        vals = self._lidar_ranges[fwd]
        vals = vals[np.isfinite(vals)]
        if vals.size == 0:
            return float('inf')
        return float(np.percentile(vals, 10))

    def _speed_to_throttle(self, target: float, current: float) -> float:
        """P-controller for speed -> throttle [-1, 1]."""
        err = target - current
        kp = 0.4
        raw = float(np.clip(kp * err, -1.0, 1.0))
        # Ramp: don't change throttle too fast
        max_delta = 0.15
        prev = self._last_throttle
        return float(np.clip(raw, prev - max_delta, prev + max_delta))

    def _publish(self, throttle: float, steering: float) -> None:
        t_msg = Float32()
        t_msg.data = float(np.clip(throttle, -1.0, 1.0))
        s_msg = Float32()
        s_msg.data = float(np.clip(steering, -MAX_STEER_NORM, MAX_STEER_NORM))
        self._pub_throttle.publish(t_msg)
        self._pub_steering.publish(s_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RoboRacerAutonomyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
