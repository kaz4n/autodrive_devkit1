from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Optional

from .math_utils import clamp, low_pass, quaternion_to_yaw, unwrap_delta, wrap_to_pi
from .models import VehicleState
from .params import LocalizationConfig, VehicleGeometry


@dataclass
class _EncoderSample:
    stamp: float = 0.0
    angle: float = 0.0
    valid: bool = False


class WheelOdometryEstimator:
    """Competition-legal wheel-odometry prior.

    This estimator uses only IMU orientation / yaw-rate, wheel encoders, and steering feedback.
    It is intended to provide a fast odometry prior for scan matching or particle-filter based
    localization, while still being useful as a fallback pose source when no external localizer is
    available.
    """

    def __init__(
        self,
        geometry: VehicleGeometry,
        config: Optional[LocalizationConfig] = None,
    ) -> None:
        self._geometry = geometry
        self._config = config or LocalizationConfig()
        self._state = VehicleState()
        self._last_predict_stamp: float = 0.0
        self._last_imu_stamp: float = 0.0
        self._last_speed_stamp: float = 0.0
        self._last_left = _EncoderSample()
        self._last_right = _EncoderSample()
        self._speed_initialized = False
        self._raw_model_yaw: Optional[float] = None

    @property
    def state(self) -> VehicleState:
        return self._state

    def update_steering(self, steering_normalized: float, stamp: float) -> VehicleState:
        steering_normalized = clamp(steering_normalized, -1.0, 1.0)
        self._state.steering_normalized = steering_normalized
        self._state.steering_angle = steering_normalized * self._geometry.max_steer_angle_rad
        self._state.stamp = max(self._state.stamp, stamp)
        self._state.valid = True
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
        measured_yaw = quaternion_to_yaw(orientation_x, orientation_y, orientation_z, orientation_w)
        if not self._state.valid:
            self._state.yaw = measured_yaw
            self._raw_model_yaw = measured_yaw
        else:
            self._state.yaw = wrap_to_pi(
                low_pass(self._state.yaw, measured_yaw, self._config.yaw_fusion_alpha)
            )
            if self._raw_model_yaw is None:
                self._raw_model_yaw = self._state.yaw
        self._state.yaw_rate = float(yaw_rate)
        self._state.linear_accel_x = float(linear_accel_x)
        self._state.stamp = max(self._state.stamp, stamp)
        self._state.valid = True
        self._last_imu_stamp = stamp
        self._refresh_confidence(stamp)
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
            self._refresh_confidence(stamp)
            return self._state

        dt = max(0.0, stamp - self._last_predict_stamp)
        if dt <= 1e-4:
            self._state.stamp = stamp
            self._refresh_confidence(stamp)
            return self._state

        speed = float(self._state.speed)
        steer = float(self._state.steering_angle)
        curvature = math.tan(steer) / max(self._geometry.wheelbase_m, 1e-6)
        model_yaw = wrap_to_pi(self._state.yaw + speed * curvature * dt)
        self._raw_model_yaw = model_yaw

        self._state.x += speed * dt * math.cos(self._state.yaw)
        self._state.y += speed * dt * math.sin(self._state.yaw)

        if stamp - self._last_imu_stamp > 0.08:
            self._state.yaw = model_yaw
        else:
            self._state.yaw = wrap_to_pi(
                low_pass(model_yaw, self._state.yaw, self._config.yaw_fusion_alpha)
            )

        self._state.stamp = stamp
        self._state.source = 'wheel_odom'
        self._state.valid = True
        self._last_predict_stamp = stamp
        self._refresh_confidence(stamp)
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
        measurements = []
        if left_speed is not None:
            measurements.append(float(left_speed))
        if right_speed is not None:
            measurements.append(float(right_speed))
        if not measurements:
            return

        measurement = sum(measurements) / float(len(measurements))
        if not self._speed_initialized:
            self._state.speed = measurement
            self._speed_initialized = True
        else:
            self._state.speed = low_pass(
                self._state.speed,
                measurement,
                self._config.speed_fusion_alpha,
            )
        self._last_speed_stamp = stamp
        self._state.stamp = max(self._state.stamp, stamp)
        self._state.valid = True
        self._refresh_confidence(stamp)

    def _refresh_confidence(self, stamp: float) -> None:
        age = max(0.0, stamp - max(self._last_speed_stamp, self._last_imu_stamp, self._state.stamp))
        decay = clamp(1.0 - age / max(self._config.odom_pose_confidence_decay_s, 1e-3), 0.0, 1.0)
        imu_bonus = 0.25 if self._last_imu_stamp > 0.0 else 0.0
        speed_bonus = 0.35 if self._last_speed_stamp > 0.0 else 0.0
        steering_bonus = 0.20 if abs(self._state.steering_normalized) <= 1.0 else 0.0
        self._state.confidence = clamp((0.20 + imu_bonus + speed_bonus + steering_bonus) * decay, 0.0, 0.95)
        self._state.source = 'wheel_odom'


SimpleStateEstimator = WheelOdometryEstimator
