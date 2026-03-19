from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Optional

from .math_utils import clamp, low_pass, quaternion_to_yaw, unwrap_delta, wrap_to_pi
from .models import VehicleState
from .params import VehicleGeometry


@dataclass
class _EncoderSample:
    stamp: float = 0.0
    angle: float = 0.0
    valid: bool = False


class SimpleStateEstimator:
    """Competition-legal dead-reckoning estimator.

    The rulebook exposes IMU and encoder topics as legal runtime inputs, while IPS/odometry are
    restricted to debugging and offline use. This estimator therefore fuses IMU yaw with rear-wheel
    encoder speed and steering feedback only.
    """

    def __init__(self, geometry: VehicleGeometry) -> None:
        self._geometry = geometry
        self._state = VehicleState()
        self._last_predict_stamp: float = 0.0
        self._last_imu_yaw_stamp: float = 0.0
        self._last_left = _EncoderSample()
        self._last_right = _EncoderSample()
        self._speed_initialized = False

    @property
    def state(self) -> VehicleState:
        return self._state

    def update_steering(self, steering_angle_rad: float, stamp: float) -> VehicleState:
        self._state.stamp = max(self._state.stamp, stamp)
        self._state.steering_angle = clamp(
            steering_angle_rad,
            -self._geometry.max_steer_angle_rad,
            self._geometry.max_steer_angle_rad,
        )
        return self._state

    def update_imu(
        self,
        orientation_x: float,
        orientation_y: float,
        orientation_z: float,
        orientation_w: float,
        yaw_rate: float,
        linear_accel_x: float,
        stamp: float,
    ) -> VehicleState:
        yaw = quaternion_to_yaw(orientation_x, orientation_y, orientation_z, orientation_w)
        if not self._state.valid:
            self._state.yaw = yaw
        else:
            # IMU yaw is trusted more than integrated yaw. Low-pass to reject transient noise.
            self._state.yaw = wrap_to_pi(low_pass(self._state.yaw, yaw, 0.75))
        self._state.yaw_rate = yaw_rate
        self._state.linear_accel_x = linear_accel_x
        self._state.stamp = max(self._state.stamp, stamp)
        self._state.valid = True
        self._last_imu_yaw_stamp = stamp
        return self._state

    def update_left_encoder(self, angle_rad: float, stamp: float) -> VehicleState:
        left_speed = self._compute_wheel_speed(self._last_left, angle_rad, stamp)
        if left_speed is not None:
            self._update_speed_from_wheels(left_speed, None, stamp)
        self._last_left = _EncoderSample(stamp=stamp, angle=angle_rad, valid=True)
        return self._state

    def update_right_encoder(self, angle_rad: float, stamp: float) -> VehicleState:
        right_speed = self._compute_wheel_speed(self._last_right, angle_rad, stamp)
        if right_speed is not None:
            self._update_speed_from_wheels(None, right_speed, stamp)
        self._last_right = _EncoderSample(stamp=stamp, angle=angle_rad, valid=True)
        return self._state

    def predict(self, stamp: float) -> VehicleState:
        if not self._state.valid:
            return self._state
        if self._last_predict_stamp <= 0.0:
            self._last_predict_stamp = stamp
            self._state.stamp = stamp
            return self._state

        dt = max(0.0, stamp - self._last_predict_stamp)
        if dt <= 1e-4:
            self._state.stamp = stamp
            return self._state

        yaw = self._state.yaw
        speed = self._state.speed
        self._state.x += speed * dt * math.cos(yaw)
        self._state.y += speed * dt * math.sin(yaw)
        self._state.yaw = wrap_to_pi(yaw + self._state.yaw_rate * dt * 0.15)
        self._state.stamp = stamp
        self._last_predict_stamp = stamp
        return self._state

    def _compute_wheel_speed(
        self,
        previous: _EncoderSample,
        angle_rad: float,
        stamp: float,
    ) -> Optional[float]:
        if not previous.valid:
            return None
        dt = stamp - previous.stamp
        if dt <= 1e-4:
            return None
        delta_angle = unwrap_delta(angle_rad, previous.angle)
        angular_speed = delta_angle / dt
        return angular_speed * self._geometry.wheel_radius_m

    def _update_speed_from_wheels(
        self,
        left_speed: Optional[float],
        right_speed: Optional[float],
        stamp: float,
    ) -> None:
        speeds = []
        if left_speed is not None:
            speeds.append(left_speed)
        if right_speed is not None:
            speeds.append(right_speed)
        if not speeds:
            return

        measurement = sum(speeds) / float(len(speeds))
        if not self._speed_initialized:
            self._state.speed = measurement
            self._speed_initialized = True
        else:
            self._state.speed = low_pass(self._state.speed, measurement, 0.45)
        self._state.stamp = max(self._state.stamp, stamp)
        self._state.valid = True
