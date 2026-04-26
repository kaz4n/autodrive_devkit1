from __future__ import annotations

import math
import time
from typing import Optional, Tuple

import numpy as np

try:
    from scipy.optimize import minimize
except Exception:  # pragma: no cover
    minimize = None

from .math_utils import (
    clamp,
    curvature_from_profile,
    finite_difference_heading,
    interpolate_path_value,
    moving_average_1d,
    wrap_to_pi,
)
from .models import ControlCommand, SolverDebug, VehicleState
from .params import MPCConfig, VehicleGeometry


class FreeSpaceMPC:
    """Corridor-based nonlinear MPC solved in the current vehicle frame.

    The solver does not rely on any pre-computed map, lap recording, or global raceline.
    At every cycle it rebuilds a local corridor from live left/right boundary points and
    optimizes throttle/steering-rate commands over a short horizon.
    """

    def __init__(self, geometry: VehicleGeometry, config: MPCConfig) -> None:
        self._geometry = geometry
        self._config = config
        self._prev_solution = np.zeros((2 * max(2, int(config.control_knots)),), dtype=float)
        self._last_throttle = 0.0
        self._last_steering = 0.0
        self._last_solver_debug = SolverDebug()
        self._width_guess = float(config.corridor_width_guess_m)

    @property
    def last_solver_debug(self) -> SolverDebug:
        return self._last_solver_debug

    def solve(
        self,
        state: VehicleState,
        left_boundary: np.ndarray,
        right_boundary: np.ndarray,
        *,
        stamp: float,
    ) -> ControlCommand:
        left = self._as_points(left_boundary)
        right = self._as_points(right_boundary)
        if left.shape[0] < 2 and right.shape[0] < 2:
            return self._failure_command(stamp, reason='empty_corridor')

        profile = self._build_corridor_profile(state, left, right)
        if profile is None:
            return self._failure_command(stamp, reason='invalid_corridor')
        x_grid, left_y, right_y, center_y, heading_ref, curvature_ref, speed_ref = profile
        finite_width = np.isfinite(left_y) & np.isfinite(right_y) & ((left_y - right_y) > 2.0 * self._config.corridor_margin_m)
        if np.count_nonzero(finite_width) < self._config.min_corridor_points:
            return self._failure_command(stamp, reason='weak_corridor')

        state0 = np.asarray([
            0.0,
            0.0,
            0.0,
            max(self._config.min_speed_mps, float(state.speed)),
            float(state.steering_angle),
        ], dtype=float)
        t0 = time.perf_counter()

        if minimize is None:
            return self._failure_command(stamp, reason='solver_unavailable')

        knots = max(2, int(self._config.control_knots))
        z0 = self._warm_start_guess(knots)
        bounds = [(-self._config.max_brake_mps2, self._config.max_accel_mps2)] * knots
        bounds += [(-self._geometry.max_steer_rate_radps, self._geometry.max_steer_rate_radps)] * knots

        def objective(z_flat: np.ndarray) -> float:
            accel_seq, steer_rate_seq = self._expand_controls(z_flat)
            return self._rollout_cost(
                state0,
                accel_seq,
                steer_rate_seq,
                x_grid,
                left_y,
                right_y,
                center_y,
                heading_ref,
                curvature_ref,
                speed_ref,
            )

        try:
            result = minimize(
                objective,
                z0,
                method='L-BFGS-B',
                bounds=bounds,
                options={
                    'maxiter': int(self._config.solver_max_iter),
                    'ftol': float(self._config.solver_ftol),
                },
            )
        except Exception:
            result = None

        solve_ms = 1.0e3 * (time.perf_counter() - t0)
        if result is None or not bool(result.success):
            return self._failure_command(stamp, reason='solver_failure', solve_ms=solve_ms)

        self._prev_solution = np.asarray(result.x, dtype=float)
        accel_seq, steer_rate_seq = self._expand_controls(self._prev_solution)
        predicted_path, predicted_states = self._simulate_sequence(state0, accel_seq, steer_rate_seq)

        target_delta = float(
            np.clip(
                predicted_states[1, 4] if predicted_states.shape[0] > 1 else state0[4],
                -self._geometry.max_steer_angle_rad,
                self._geometry.max_steer_angle_rad,
            )
        )
        steering_cmd = float(np.clip(target_delta / max(self._geometry.max_steer_angle_rad, 1.0e-6), -1.0, 1.0))
        throttle_cmd = self._accel_to_throttle(float(accel_seq[0]))
        target_speed = float(np.clip(speed_ref[min(1, speed_ref.size - 1)] if speed_ref.size else state.speed, 0.0, self._config.max_speed_mps))

        self._last_throttle = throttle_cmd
        self._last_steering = steering_cmd
        self._last_solver_debug = SolverDebug(
            success=True,
            cost=float(result.fun),
            solve_time_ms=float(solve_ms),
            iterations=int(getattr(result, 'nit', 0)),
            target_speed=target_speed,
            progress_m=float(predicted_path[-1, 0]) if predicted_path.size else 0.0,
            predicted_path=predicted_path,
        )
        return ControlCommand(
            stamp=stamp,
            throttle=throttle_cmd,
            steering=steering_cmd,
            target_speed=target_speed,
            emergency=False,
            reason='',
            metadata={
                'solve_time_ms': float(solve_ms),
                'solver_iterations': float(getattr(result, 'nit', 0)),
            },
        )

    def _failure_command(self, stamp: float, reason: str, solve_ms: float = 0.0) -> ControlCommand:
        self._last_solver_debug = SolverDebug(
            success=False,
            cost=float('inf'),
            solve_time_ms=float(solve_ms),
            iterations=0,
            target_speed=0.0,
            progress_m=0.0,
            predicted_path=np.zeros((0, 2), dtype=float),
        )
        return ControlCommand(
            stamp=stamp,
            throttle=0.0,
            steering=self._last_steering,
            target_speed=0.0,
            emergency=True,
            reason=reason,
            metadata={'solve_time_ms': float(solve_ms)},
        )

    def _as_points(self, points: np.ndarray) -> np.ndarray:
        arr = np.asarray(points, dtype=float)
        if arr.size == 0:
            return np.zeros((0, 2), dtype=float)
        arr = arr.reshape(-1, 2)
        valid = np.isfinite(arr[:, 0]) & np.isfinite(arr[:, 1])
        arr = arr[valid]
        if arr.shape[0] == 0:
            return np.zeros((0, 2), dtype=float)
        order = np.argsort(arr[:, 0])
        return arr[order]

    def _warm_start_guess(self, knots: int) -> np.ndarray:
        if self._prev_solution.size != 2 * knots:
            return np.zeros((2 * knots,), dtype=float)
        prev = self._prev_solution.copy()
        if knots >= 2:
            prev[:knots - 1] = prev[1:knots]
            prev[knots - 1] = prev[knots - 2]
            prev[knots:2 * knots - 1] = prev[knots + 1:2 * knots]
            prev[2 * knots - 1] = prev[2 * knots - 2]
        return prev

    def _expand_controls(self, z_flat: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        knots = max(2, int(self._config.control_knots))
        z = np.asarray(z_flat, dtype=float).reshape(-1)
        accel_knots = z[:knots]
        steer_knots = z[knots:2 * knots]
        knot_pos = np.linspace(0.0, self._config.horizon_steps - 1, knots)
        idx = np.arange(self._config.horizon_steps, dtype=float)
        accel_seq = np.interp(idx, knot_pos, accel_knots)
        steer_rate_seq = np.interp(idx, knot_pos, steer_knots)
        return accel_seq.astype(float), steer_rate_seq.astype(float)

    def _build_corridor_profile(
        self,
        state: VehicleState,
        left_boundary: np.ndarray,
        right_boundary: np.ndarray,
    ) -> Optional[Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]]:
        horizon_m = max(4.0, min(12.0, state.speed * self._config.dt * self._config.horizon_steps * 1.5 + 2.0))
        x_grid = np.linspace(0.25, horizon_m, int(self._config.corridor_samples))
        left_y = interpolate_path_value(left_boundary, x_grid)
        right_y = interpolate_path_value(right_boundary, x_grid)

        valid_width = np.isfinite(left_y) & np.isfinite(right_y)
        if np.any(valid_width):
            widths = left_y[valid_width] - right_y[valid_width]
            widths = widths[(widths >= self._config.min_track_width_m) & (widths <= self._config.max_track_width_m)]
            if widths.size > 0:
                measured_width = float(np.median(widths))
                self._width_guess = clamp(0.70 * self._width_guess + 0.30 * measured_width, self._config.min_track_width_m, self._config.max_track_width_m)
            else:
                self._width_guess = clamp(self._width_guess, self._config.min_track_width_m, self._config.max_track_width_m)
        else:
            self._width_guess = clamp(self._width_guess, self._config.min_track_width_m, self._config.max_track_width_m)

        for idx in range(x_grid.size):
            has_left = np.isfinite(left_y[idx])
            has_right = np.isfinite(right_y[idx])
            if has_left and has_right:
                width = float(left_y[idx] - right_y[idx])
                if width < self._config.min_track_width_m or width > self._config.max_track_width_m:
                    center = 0.5 * (left_y[idx] + right_y[idx])
                    left_y[idx] = center + 0.5 * self._width_guess
                    right_y[idx] = center - 0.5 * self._width_guess
                else:
                    self._width_guess = clamp(0.80 * self._width_guess + 0.20 * width, self._config.min_track_width_m, self._config.max_track_width_m)
            elif has_left and not has_right:
                right_y[idx] = left_y[idx] - self._width_guess
            elif has_right and not has_left:
                left_y[idx] = right_y[idx] + self._width_guess

        if np.count_nonzero(np.isfinite(left_y) | np.isfinite(right_y)) < self._config.min_corridor_points:
            return None

        left_y = self._nan_fill(left_y)
        right_y = self._nan_fill(right_y)
        center_y = 0.5 * (left_y + right_y)
        center_y = moving_average_1d(center_y, 5)
        width = np.clip(left_y - right_y, self._config.min_track_width_m, self._config.max_track_width_m)
        left_y = center_y + 0.5 * width
        right_y = center_y - 0.5 * width

        heading_ref = finite_difference_heading(center_y, x_grid)
        curvature_ref = curvature_from_profile(center_y, x_grid)
        curvature_mag = np.maximum(np.abs(curvature_ref), 0.02)
        speed_ref = np.sqrt(np.maximum(0.5, self._config.lateral_accel_limit_mps2 / curvature_mag))
        width_scale = np.clip((width - 2.0 * self._config.corridor_margin_m) / max(self._width_guess, 1.0e-3), 0.35, 1.00)
        speed_ref *= np.sqrt(width_scale)
        speed_ref = np.clip(speed_ref, self._config.min_speed_mps, self._config.max_speed_mps)
        return x_grid, left_y, right_y, center_y, heading_ref, curvature_ref, speed_ref

    def _nan_fill(self, values: np.ndarray) -> np.ndarray:
        arr = np.asarray(values, dtype=float).copy()
        finite = np.isfinite(arr)
        if np.all(finite):
            return arr
        if not np.any(finite):
            return np.zeros_like(arr)
        idx = np.where(finite)[0]
        arr[~finite] = np.interp(np.where(~finite)[0], idx, arr[finite])
        return arr

    def _simulate_sequence(self, state0: np.ndarray, accel_seq: np.ndarray, steer_rate_seq: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        dt = self._config.dt
        x, y, psi, v, delta = [float(v_i) for v_i in state0]
        path = []
        states = [state0.copy()]
        for k in range(self._config.horizon_steps):
            a = float(accel_seq[k])
            delta_rate = float(steer_rate_seq[k])
            x += dt * v * math.cos(psi)
            y += dt * v * math.sin(psi)
            psi = wrap_to_pi(psi + dt * v * math.tan(delta) / max(self._geometry.wheelbase_m, 1.0e-6))
            v = float(np.clip(v + dt * a, self._config.min_speed_mps, self._config.max_speed_mps))
            delta = float(np.clip(delta + dt * delta_rate, -self._geometry.max_steer_angle_rad, self._geometry.max_steer_angle_rad))
            path.append((x, y))
            states.append(np.asarray([x, y, psi, v, delta], dtype=float))
        return np.asarray(path, dtype=float), np.asarray(states, dtype=float)

    def _rollout_cost(
        self,
        state0: np.ndarray,
        accel_seq: np.ndarray,
        steer_rate_seq: np.ndarray,
        x_grid: np.ndarray,
        left_y: np.ndarray,
        right_y: np.ndarray,
        center_y: np.ndarray,
        heading_ref: np.ndarray,
        curvature_ref: np.ndarray,
        speed_ref: np.ndarray,
    ) -> float:
        _, states = self._simulate_sequence(state0, accel_seq, steer_rate_seq)
        if states.shape[0] <= 1:
            return 1.0e9
        cost = 0.0
        prev_a = 0.0
        prev_d = 0.0
        for k in range(self._config.horizon_steps):
            x = float(states[k + 1, 0])
            y = float(states[k + 1, 1])
            psi = float(states[k + 1, 2])
            v = float(states[k + 1, 3])
            delta = float(states[k + 1, 4])
            a = float(accel_seq[k])
            steer_rate = float(steer_rate_seq[k])

            y_mid = float(np.interp(x, x_grid, center_y, left=center_y[0], right=center_y[-1]))
            psi_ref = float(np.interp(x, x_grid, heading_ref, left=heading_ref[0], right=heading_ref[-1]))
            v_ref = float(np.interp(x, x_grid, speed_ref, left=speed_ref[0], right=speed_ref[-1]))
            y_left = float(np.interp(x, x_grid, left_y, left=left_y[0], right=left_y[-1]))
            y_right = float(np.interp(x, x_grid, right_y, left=right_y[0], right=right_y[-1]))
            width = max(y_left - y_right, 1.0e-3)

            left_violation = max(0.0, y - (y_left - self._config.corridor_margin_m))
            right_violation = max(0.0, (y_right + self._config.corridor_margin_m) - y)
            narrow_penalty = max(0.0, 2.0 * self._config.corridor_margin_m - width)

            cost += self._config.w_lat * (y - y_mid) ** 2
            cost += self._config.w_heading * wrap_to_pi(psi - psi_ref) ** 2
            cost += self._config.w_speed * (v - v_ref) ** 2
            cost += self._config.w_accel * a * a
            cost += self._config.w_steer_rate * steer_rate * steer_rate
            cost += self._config.w_steer_abs * delta * delta
            cost += self._config.w_boundary * (left_violation ** 2 + right_violation ** 2 + narrow_penalty ** 2)
            cost += self._config.w_input_smooth * ((a - prev_a) ** 2 + 0.5 * (steer_rate - prev_d) ** 2)
            prev_a = a
            prev_d = steer_rate

        x_terminal = float(states[-1, 0])
        y_terminal = float(states[-1, 1])
        psi_terminal = float(states[-1, 2])
        y_mid_terminal = float(np.interp(x_terminal, x_grid, center_y, left=center_y[0], right=center_y[-1]))
        psi_ref_terminal = float(np.interp(x_terminal, x_grid, heading_ref, left=heading_ref[0], right=heading_ref[-1]))
        cost += self._config.w_terminal_lat * (y_terminal - y_mid_terminal) ** 2
        cost += 0.5 * self._config.w_heading * wrap_to_pi(psi_terminal - psi_ref_terminal) ** 2
        cost -= self._config.w_progress * x_terminal
        return float(cost)

    def _accel_to_throttle(self, accel: float) -> float:
        if accel >= 0.0:
            return float(np.clip(accel / max(self._config.max_accel_mps2, 1.0e-6), 0.0, 1.0))
        return float(np.clip(accel / max(self._config.max_brake_mps2, 1.0e-6), -1.0, 0.0))
