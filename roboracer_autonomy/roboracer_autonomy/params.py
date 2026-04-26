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
    external_pose_timeout_s: float = 0.20
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
    boundary_lookahead_m: float = 8.0
    x_bin_size_m: float = 0.15
    centerline_smoothing_window: int = 5
    side_outlier_jump_m: float = 0.40
    min_boundary_points_per_side: int = 5
    nominal_track_width_m: float = 1.80
    min_track_width_m: float = 0.90
    max_track_width_m: float = 3.50
    stop_distance_m: float = 0.55
    hard_stop_distance_m: float = 0.28
    caution_ttc_s: float = 0.90
    hard_ttc_s: float = 0.35
    bubble_radius_m: float = 0.35
    min_free_distance_m: float = 0.20
    gap_continuity_weight: float = 0.30


@dataclass
class MappingConfig:
    """Deprecated placeholder kept only for legacy imports.

    The nominal stack intentionally does not depend on saved maps or manual mapping laps.
    """

    enabled: bool = False
    maps_root: str = ''
    track_name_override: str = ''
    auto_track_selection: bool = False
    auto_create_tracks: bool = False


@dataclass
class MPCConfig:
    dt: float = 0.10
    horizon_steps: int = 20
    control_knots: int = 7
    corridor_samples: int = 60
    corridor_width_guess_m: float = 1.80
    min_track_width_m: float = 0.90
    max_track_width_m: float = 3.50
    max_speed_mps: float = 10.0
    max_accel_mps2: float = 4.0
    max_brake_mps2: float = 6.0
    lateral_accel_limit_mps2: float = 5.5
    corridor_margin_m: float = 0.15
    w_lat: float = 10.0
    w_heading: float = 5.0
    w_speed: float = 1.50
    w_progress: float = 2.00
    w_accel: float = 0.50
    w_steer_rate: float = 1.00
    w_steer_abs: float = 0.15
    w_boundary: float = 100.0
    w_terminal_lat: float = 10.0
    w_input_smooth: float = 0.75
    solver_max_iter: int = 60
    solver_ftol: float = 1.0e-3
    stale_lidar_timeout_s: float = 0.15
    emergency_clearance_m: float = 0.25
    emergency_ttc_s: float = 0.18
    min_corridor_points: int = 6
    min_speed_mps: float = 0.0
    fallback_brake_command: float = -0.30
    fallback_hold_cycles: int = 3


@dataclass
class StackConfig:
    vehicle: VehicleGeometry = field(default_factory=VehicleGeometry)
    localization: LocalizationConfig = field(default_factory=LocalizationConfig)
    lidar: LidarConfig = field(default_factory=LidarConfig)
    mapping: MappingConfig = field(default_factory=MappingConfig)
    mpc: MPCConfig = field(default_factory=MPCConfig)
