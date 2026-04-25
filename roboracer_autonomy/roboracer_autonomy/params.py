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
    max_steer_rate_radps: float = 3.2


@dataclass
class LocalizationConfig:
    external_pose_topic: str = ''
    external_pose_timeout_s: float = 0.25
    use_external_pose_if_available: bool = True
    odom_pose_confidence_decay_s: float = 1.0
    yaw_fusion_alpha: float = 0.65
    speed_fusion_alpha: float = 0.45


@dataclass
class LidarConfig:
    focus_half_angle_deg: float = 110.0
    forward_sector_deg: float = 12.0
    range_min_clip_m: float = 0.06
    range_max_clip_m: float = 8.0
    smoothing_kernel: int = 5
    leak_fill_max_bins: int = 6
    boundary_lookahead_m: float = 6.0
    x_bin_size_m: float = 0.15
    centerline_smoothing_window: int = 7
    side_outlier_jump_m: float = 0.40
    min_boundary_points_per_side: int = 5
    nominal_track_width_m: float = 1.80
    min_track_width_m: float = 0.90
    max_track_width_m: float = 3.50
    stop_distance_m: float = 0.60
    hard_stop_distance_m: float = 0.35
    caution_ttc_s: float = 0.90
    hard_ttc_s: float = 0.45
    bubble_radius_m: float = 0.35
    min_free_distance_m: float = 0.20
    gap_continuity_weight: float = 0.30


@dataclass
class CameraConfig:
    enabled: bool = True
    process_period_s: float = 0.08
    roi_vertical_start: float = 0.38
    canny_low: int = 50
    canny_high: int = 140
    hough_threshold: int = 20
    hough_min_line_length: int = 18
    hough_max_line_gap: int = 10
    min_slope_abs: float = 0.20
    heading_scale_rad: float = 0.65
    max_fused_weight: float = 0.12


@dataclass
class PlannerConfig:
    raceline_csv_path: str = ''
    raceline_closed_loop: bool = True
    require_external_pose_for_raceline: bool = False
    local_horizon_m: float = 8.0
    local_horizon_points: int = 35
    max_speed_mps: float = 8.0
    avoid_mode_speed_mps: float = 4.0
    localize_mode_speed_mps: float = 1.5
    min_speed_mps: float = 0.0
    nominal_lookahead_m: float = 1.20
    min_lookahead_m: float = 0.80
    max_lookahead_m: float = 3.00
    lookahead_speed_gain: float = 0.18
    lateral_accel_limit_mps2: float = 6.0
    max_brake_decel_mps2: float = 5.5
    max_accel_mps2: float = 4.0
    clearance_speed_gain: float = 1.30
    narrow_width_slowdown_m: float = 1.15
    min_track_confidence: float = 0.28
    min_pose_confidence: float = 0.20
    min_path_points: int = 8
    corridor_validation_error_m: float = 0.70
    camera_confidence_threshold: float = 0.25
    camera_stale_after_s: float = 0.15
    camera_stale_full_decay_s: float = 0.45
    camera_center_offset_gain: float = 0.15


@dataclass
class ControllerConfig:
    steering_kp: float = 0.90
    steering_kd: float = 0.10
    steering_feedforward_gain: float = 1.00
    steer_yaw_rate_damping: float = 0.08
    throttle_kp: float = 0.25
    throttle_ki: float = 0.06
    throttle_kd: float = 0.015
    throttle_feedforward_gain: float = 0.16
    throttle_min: float = -0.30
    throttle_max: float = 1.00
    throttle_rate_limit_per_s: float = 2.00
    steering_rate_limit_per_s: float = 5.80
    yaw_rate_low_pass_alpha: float = 0.30
    avoid_mode_throttle_cap: float = 0.55


@dataclass
class MissionConfig:
    sensor_timeout_s: float = 0.35
    steering_timeout_s: float = 0.60
    camera_timeout_s: float = 0.45
    bootstrap_time_s: float = 0.20
    safety_clearance_enter_m: float = 0.35
    safety_clearance_exit_m: float = 0.60
    safety_ttc_enter_s: float = 0.45
    safety_ttc_exit_s: float = 0.85
    safety_brake_hold_s: float = 0.30
    stale_cycles_before_brake: int = 5
    min_pose_confidence: float = 0.15
    min_track_confidence: float = 0.25
    min_centerline_points: int = 6
    avoid_enter_consecutive_scans: int = 3
    avoid_exit_consecutive_scans: int = 6
    recovery_enabled: bool = False
    recovery_reverse_throttle: float = -0.15
    recovery_duration_s: float = 0.50


@dataclass
class StackConfig:
    vehicle: VehicleGeometry = field(default_factory=VehicleGeometry)
    localization: LocalizationConfig = field(default_factory=LocalizationConfig)
    lidar: LidarConfig = field(default_factory=LidarConfig)
    camera: CameraConfig = field(default_factory=CameraConfig)
    planner: PlannerConfig = field(default_factory=PlannerConfig)
    controller: ControllerConfig = field(default_factory=ControllerConfig)
    mission: MissionConfig = field(default_factory=MissionConfig)
