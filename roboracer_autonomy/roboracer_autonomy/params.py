from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class VehicleGeometry:
    wheelbase_m: float = 0.324
    track_width_m: float = 0.236
    wheel_radius_m: float = 0.059
    car_length_m: float = 0.500
    car_width_m: float = 0.270
    max_steer_angle_rad: float = 0.5236


@dataclass
class LidarConfig:
    focus_half_angle_deg: float = 110.0
    bubble_radius_m: float = 0.35
    min_free_distance_m: float = 0.20
    forward_sector_deg: float = 12.0
    stop_distance_m: float = 0.55
    hard_stop_distance_m: float = 0.35
    hard_ttc_s: float = 0.45
    caution_ttc_s: float = 0.90
    leak_fill_max_bins: int = 6
    smoothing_kernel: int = 5
    range_max_clip_m: float = 8.0
    lane_width_slowdown_m: float = 1.1
    lane_width_confident_m: float = 1.4


@dataclass
class CameraConfig:
    enabled: bool = True
    roi_vertical_start: float = 0.38
    canny_low: int = 50
    canny_high: int = 140
    hough_threshold: int = 20
    hough_min_line_length: int = 18
    hough_max_line_gap: int = 10
    min_slope_abs: float = 0.20
    heading_scale_rad: float = 0.65
    max_fused_weight: float = 0.25


@dataclass
class PlannerConfig:
    max_speed_mps: float = 4.0
    gap_mode_speed_mps: float = 2.4
    straight_speed_mps: float = 4.6
    min_speed_mps: float = 0.0
    nominal_lookahead_m: float = 1.10
    min_lookahead_m: float = 0.75
    max_lookahead_m: float = 2.20
    lookahead_speed_gain: float = 0.18
    lateral_accel_limit_mps2: float = 4.5
    clearance_speed_gain: float = 1.25 #was 1.25
    steering_bias_limit_rad: float = 0.85
    gap_activation_angle_rad: float = 0.30
    camera_confidence_threshold: float = 0.25


@dataclass
class ControllerConfig:
    throttle_kp: float = 0.22
    throttle_ki: float = 0.05
    throttle_kd: float = 0.02
    throttle_feedforward_gain: float = 0.18
    steer_yaw_rate_damping: float = 0.08
    throttle_rate_limit_per_s: float = 1.4
    steering_rate_limit_per_s: float = 4.0


@dataclass
class MissionConfig:
    sensor_timeout_s: float = 0.35 #was 0.35
    steering_timeout_s: float = 0.70
    camera_timeout_s: float = 0.45
    bootstrap_time_s: float = 0.2 #was 0.2
    safety_clearance_enter_m: float = 0.35
    safety_clearance_exit_m: float = 0.55
    safety_ttc_enter_s: float = 0.45
    safety_ttc_exit_s: float = 0.80
    safety_brake_hold_s: float = 0.30
    stale_cycles_before_brake: int = 5
    gap_enter_angle_rad: float = 0.32
    gap_exit_angle_rad: float = 0.20
    gap_enter_consecutive_scans: int = 4
    gap_exit_consecutive_scans: int = 8
    recovery_enabled: bool = False
    recovery_reverse_throttle: float = -0.15
    recovery_duration_s: float = 0.50


@dataclass
class StackConfig:
    vehicle: VehicleGeometry = field(default_factory=VehicleGeometry)
    lidar: LidarConfig = field(default_factory=LidarConfig)
    camera: CameraConfig = field(default_factory=CameraConfig)
    planner: PlannerConfig = field(default_factory=PlannerConfig)
    controller: ControllerConfig = field(default_factory=ControllerConfig)
    mission: MissionConfig = field(default_factory=MissionConfig)
