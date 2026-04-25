from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence, Tuple

import numpy as np

from .math_utils import clamp, low_pass, nearest_point_index, transform_points_world_to_local
from .models import ControlCommand, MissionMode, Plan, VehicleState, Waypoint
from .params import ControllerConfig, VehicleGeometry


@dataclass
class PIDState:
    integral: float = 0.0
    previous_error: float = 0.0
    previous_stamp: float = 0.0


class AdaptivePurePursuitController:
    def __init__(
        self,
        geometry: VehicleGeometry,
        config: ControllerConfig,
        nominal_max_speed_mps: float,
    ) -> None:
        self._geometry = geometry
        self._config = config
        self._nominal_max_speed_mps = max(nominal_max_speed_mps, 0.1)
        self._pid = PIDState()
        self._last_throttle = 0.0
        self._last_steering = 0.0
        self._last_stamp = 0.0
        self._last_mode = MissionMode.BOOTSTRAP
        self._yaw_rate_filtered = 0.0

    def compute(self, plan: Plan, state: VehicleState, stamp: float) -> ControlCommand:
        dt = stamp - self._last_stamp if self._last_stamp > 0.0 else 0.05
        dt = max(0.01, min(0.20, dt))
        self._yaw_rate_filtered = low_pass(
            self._yaw_rate_filtered,
            state.yaw_rate,
            self._config.yaw_rate_low_pass_alpha,
        )

        if plan.mode != self._last_mode:
            self._reset_pid(stamp)
            self._last_mode = plan.mode

        if plan.mode == MissionMode.SAFETY_BRAKE or len(plan.waypoints) < 2 or not state.valid:
            # Hold the current steering in stop phases so the rack does not snap back to center
            # between short braking/recovery transitions.
            steering_hold = float(self._last_steering)
            self._cache(0.0, steering_hold, stamp)
            reason = 'safety_brake' if plan.mode == MissionMode.SAFETY_BRAKE else 'stop'
            return ControlCommand(
                stamp=stamp,
                throttle=0.0,
                steering=steering_hold,
                emergency=(plan.mode == MissionMode.SAFETY_BRAKE),
                reason=reason,
            )

        if plan.target_speed <= 0.05:
            steering_hold = float(self._last_steering)
            self._cache(0.0, steering_hold, stamp)
            return ControlCommand(
                stamp=stamp,
                throttle=0.0,
                steering=steering_hold,
                emergency=False,
                reason='coast_stop',
            )

        goal_xy, goal_speed, ref_curvature = self._select_goal(plan.waypoints, state, plan.lookahead)
        local_goal = transform_points_world_to_local(goal_xy.reshape(1, 2), state)[0]
        ld = max(float(np.hypot(local_goal[0], local_goal[1])), max(plan.lookahead, 0.20))
        pp_curvature = 2.0 * local_goal[1] / max(ld * ld, 1e-6)
        commanded_curvature = 0.70 * pp_curvature + 0.30 * ref_curvature

        desired_steer_angle = math.atan(self._geometry.wheelbase_m * commanded_curvature)
        desired_steer_angle = clamp(
            desired_steer_angle,
            -self._geometry.max_steer_angle_rad,
            self._geometry.max_steer_angle_rad,
        )
        normalized_ff = self._config.steering_feedforward_gain * (
            desired_steer_angle / max(self._geometry.max_steer_angle_rad, 1e-6)
        )
        normalized_error = (desired_steer_angle - state.steering_angle) / max(self._geometry.max_steer_angle_rad, 1e-6)
        steering_target = (
            normalized_ff
            + self._config.steering_kp * normalized_error
            - self._config.steering_kd * self._yaw_rate_filtered
            - self._config.steer_yaw_rate_damping * self._yaw_rate_filtered
        )
        normalized_steer = self._rate_limit(
            self._last_steering,
            clamp(steering_target, -1.0, 1.0),
            self._config.steering_rate_limit_per_s,
            dt,
        )

        speed_target = max(plan.target_speed, min(goal_speed, self._nominal_max_speed_mps))
        speed_error = speed_target - state.speed
        throttle_ff = self._config.throttle_feedforward_gain * (speed_target / self._nominal_max_speed_mps)
        pid_output = self._pid_step(speed_error, stamp)
        throttle_target = throttle_ff + pid_output
        if speed_error < -0.25:
            throttle_target = min(throttle_target, 0.0)
        throttle = self._rate_limit(
            self._last_throttle,
            clamp(throttle_target, self._config.throttle_min, self._config.throttle_max),
            self._config.throttle_rate_limit_per_s,
            dt,
        )
        if plan.mode == MissionMode.AVOID:
            throttle = min(throttle, self._config.avoid_mode_throttle_cap)

        command = ControlCommand(
            stamp=stamp,
            throttle=float(throttle),
            steering=float(normalized_steer),
            emergency=False,
            reason=plan.mode.value,
        )
        self._cache(throttle, normalized_steer, stamp)
        return command

    def _select_goal(
        self,
        waypoints: Sequence[Waypoint],
        state: VehicleState,
        lookahead: float,
    ) -> Tuple[np.ndarray, float, float]:
        points = np.asarray([[wp.x, wp.y] for wp in waypoints], dtype=float)
        idx = nearest_point_index(points, np.asarray([state.x, state.y], dtype=float))
        remaining = 0.0
        for cursor in range(idx, len(waypoints) - 1):
            p0 = points[cursor]
            p1 = points[cursor + 1]
            seg = float(np.linalg.norm(p1 - p0))
            if remaining + seg >= lookahead:
                ratio = (lookahead - remaining) / max(seg, 1e-6)
                goal = p0 + ratio * (p1 - p0)
                speed_window = waypoints[cursor : min(cursor + 5, len(waypoints))]
                ref_speed = min(wp.target_speed for wp in speed_window)
                ref_curvature = (1.0 - ratio) * waypoints[cursor].curvature + ratio * waypoints[cursor + 1].curvature
                return goal, float(ref_speed), float(ref_curvature)
            remaining += seg

        last = waypoints[-1]
        return np.asarray([last.x, last.y], dtype=float), float(last.target_speed), float(last.curvature)

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

    def _reset_pid(self, stamp: float) -> None:
        self._pid.integral = 0.0
        self._pid.previous_error = 0.0
        self._pid.previous_stamp = stamp


LowLevelController = AdaptivePurePursuitController
