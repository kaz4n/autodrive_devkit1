from __future__ import annotations

import math
from dataclasses import dataclass

from .math_utils import clamp
from .models import ControlCommand, MissionMode, Plan, VehicleState
from .params import ControllerConfig, VehicleGeometry


@dataclass
class PIDState:
    integral: float = 0.0
    previous_error: float = 0.0
    previous_stamp: float = 0.0


class LowLevelController:
    def __init__(self, geometry: VehicleGeometry, config: ControllerConfig, nominal_max_speed_mps: float) -> None:
        self._geometry = geometry
        self._config = config
        self._nominal_max_speed_mps = max(nominal_max_speed_mps, 0.1)
        self._pid = PIDState()
        self._last_throttle = 0.0
        self._last_steering = 0.0
        self._last_stamp = 0.0

    def compute(self, plan: Plan, state: VehicleState, stamp: float) -> ControlCommand:
        dt = stamp - self._last_stamp if self._last_stamp > 0.0 else 0.05
        dt = max(0.01, min(0.20, dt))

        desired_steer_angle = math.atan(self._geometry.wheelbase_m * plan.curvature)
        normalized_steer = desired_steer_angle / max(self._geometry.max_steer_angle_rad, 1e-6)
        normalized_steer -= self._config.steer_yaw_rate_damping * state.yaw_rate
        normalized_steer = self._rate_limit(
            self._last_steering,
            clamp(normalized_steer, -1.0, 1.0),
            self._config.steering_rate_limit_per_s,
            dt,
        )

        if plan.mode == MissionMode.SAFETY_BRAKE or plan.target_speed <= 0.05:
            throttle = 0.0
            command = ControlCommand(
                stamp=stamp,
                throttle=0.0,
                steering=normalized_steer,
                emergency=(plan.mode == MissionMode.SAFETY_BRAKE),
                reason='safety_brake' if plan.mode == MissionMode.SAFETY_BRAKE else 'stop',
            )
            self._cache(throttle, normalized_steer, stamp)
            return command

        speed_error = plan.target_speed - state.speed
        throttle_ff = self._config.throttle_feedforward_gain * (plan.target_speed / self._nominal_max_speed_mps)
        pid_output = self._pid_step(speed_error, stamp)
        throttle_target = throttle_ff + pid_output
        if speed_error < -0.35:
            throttle_target = min(throttle_target, 0.0)
        throttle = self._rate_limit(
            self._last_throttle,
            clamp(throttle_target, 0.0, 1.0),
            self._config.throttle_rate_limit_per_s,
            dt,
        )
        if plan.mode == MissionMode.GAP_AVOID:
            throttle = min(throttle, 0.45)

        command = ControlCommand(
            stamp=stamp,
            throttle=throttle,
            steering=normalized_steer,
            emergency=False,
            reason=plan.mode.value,
        )
        self._cache(throttle, normalized_steer, stamp)
        return command

    def _pid_step(self, error: float, stamp: float) -> float:
        if self._pid.previous_stamp <= 0.0:
            self._pid.previous_stamp = stamp
            self._pid.previous_error = error
            return self._config.throttle_kp * error
        dt = max(0.01, min(0.20, stamp - self._pid.previous_stamp))
        self._pid.integral += error * dt
        self._pid.integral = clamp(self._pid.integral, -2.0, 2.0)
        derivative = (error - self._pid.previous_error) / dt
        output = (
            self._config.throttle_kp * error
            + self._config.throttle_ki * self._pid.integral
            + self._config.throttle_kd * derivative
        )
        self._pid.previous_error = error
        self._pid.previous_stamp = stamp
        return output

    def _rate_limit(self, current: float, target: float, limit_per_second: float, dt: float) -> float:
        max_step = abs(limit_per_second) * dt
        delta = clamp(target - current, -max_step, max_step)
        return current + delta

    def _cache(self, throttle: float, steering: float, stamp: float) -> None:
        self._last_throttle = throttle
        self._last_steering = steering
        self._last_stamp = stamp
