"""
Enhanced parameters for RoboRacer Autonomy Stack v2.

Key improvements:
- More conservative defaults for stability
- MPPI-specific tuning parameters
- Better sensor timeout handling
"""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class VehicleGeometry:
    """Vehicle kinematic parameters."""
    wheelbase_m: float = 0.324
    track_width_m: float = 0.236
    wheel_radius_m: float = 0.059
    car_length_m: float = 0.500
    car_width_m: float = 0.270
    max_steer_angle_rad: float = 0.5236  # ~30 degrees
    max_steer_rate_radps: float = 8.0  # Increased for smoother tracking
    mass_kg: float = 5.0  # For dynamic models
    friction_coefficient: float = 0.5  # Tire-road friction


@dataclass
class LocalizationConfig:
    """State estimation and localization parameters."""
    external_pose_topic: str = ''
    external_pose_timeout_s: float = 0.30  # Slightly more lenient
    use_external_pose_if_available: bool = True
    odom_pose_confidence_decay_s: float = 1.5  # Slower decay
    yaw_fusion_alpha: float = 0.55  # More balanced fusion
    speed_fusion_alpha: float = 0.40  # Smoother speed estimate
    position_noise_std: float = 0.05  # For Kalman filtering
    yaw_noise_std: float = 0.02
    speed_noise_std: float = 0.1


@dataclass
class LidarConfig:
    """LiDAR perception parameters."""
    # Basic processing
    focus_half_angle_deg: float = 100.0  # Slightly narrower focus
    forward_sector_deg: float = 15.0
    range_min_clip_m: float = 0.08
    range_max_clip_m: float = 10.0
    
    # Boundary extraction
    smoothing_kernel: int = 27  # More smoothing for stability
    leak_fill_max_bins: int = 12
    boundary_lookahead_m: float = 8.0
    x_bin_size_m: float = 0.12  # Finer binning
    centerline_smoothing_window: int = 14  # More temporal smoothing
    side_outlier_jump_m: float = 0.35
    
    # Track model
    min_boundary_points_per_side: int = 10
    nominal_track_width_m: float = 1.80
    min_track_width_m: float = 0.80
    max_track_width_m: float = 4
    
    # Safety thresholds
    stop_distance_m: float = 0.50
    hard_stop_distance_m: float = 0.30
    caution_ttc_s: float = 1.0
    hard_ttc_s: float = 0.5
    
    # Gap following fallback
    bubble_radius_m: float = 0.40
    min_free_distance_m: float = 0.25
    gap_continuity_weight: float = 0.40  # Higher weight for continuity
    
    # Temporal filtering
    temporal_alpha: float = 0.30  # For smoothing across frames
    curvature_lookahead_m: float = 3.0


@dataclass
class CameraConfig:
    """Camera perception parameters."""
    enabled: bool = True
    process_period_s: float = 0.10  # Slower, more stable processing
    roi_vertical_start: float = 0.40
    canny_low: int = 60
    canny_high: int = 150
    hough_threshold: int = 25
    hough_min_line_length: int = 20
    hough_max_line_gap: int = 12
    min_slope_abs: float = 0.25
    heading_scale_rad: float = 0.60
    max_fused_weight: float = 0.10  # Lower weight for camera
    confidence_decay_rate: float = 0.15


@dataclass
class PlannerConfig:
    """Trajectory planning parameters."""
    # Raceline
    raceline_csv_path: str = ''
    raceline_closed_loop: bool = True
    require_external_pose_for_raceline: bool = False
    
    # Planning horizon
    local_horizon_m: float = 10.0
    local_horizon_points: int = 40
    min_path_points: int = 6
    
    # Speed limits
    max_speed_mps: float = 6.0  # Conservative default
    avoid_mode_speed_mps: float = 2.0
    localize_mode_speed_mps: float = 1.2
    min_speed_mps: float = 0.3  # Don't go too slow
    
    # Lookahead scheduling
    nominal_lookahead_m: float = 1.50
    min_lookahead_m: float = 0.90
    max_lookahead_m: float = 3.50
    lookahead_speed_gain: float = 0.15
    
    # Dynamic constraints
    lateral_accel_limit_mps2: float = 5.0  # More conservative
    max_brake_decel_mps2: float = 6.0
    max_accel_mps2: float = 3.5
    
    # Speed modulation
    clearance_speed_gain: float = 1.50
    narrow_width_slowdown_m: float = 1.00
    curvature_speed_reduction: float = 1.7  # Additional reduction in high curvature
    
    # Validation
    min_track_confidence: float = 0.25
    min_pose_confidence: float = 0.18
    corridor_validation_error_m: float = 0.80
    
    # Camera fusion
    camera_confidence_threshold: float = 0.30
    camera_stale_after_s: float = 0.20
    camera_stale_full_decay_s: float = 0.50
    camera_center_offset_gain: float = 0.12


@dataclass
class ControllerConfig:
    """MPPI controller parameters."""
    # MPPI sampling
    num_samples: int = 1000  # Number of trajectory samples
    sample_horizon_s: float = 3  # How far to roll out trajectories
    dt: float = 0.02  # Time step for rollouts
    
    # Cost function weights
    cost_tracking_weight: float = 9.0  # Path tracking importance
    cost_smoothness_weight: float = 2.0  # Control smoothness
    cost_collision_weight: float = 10.0  # Collision avoidance
    cost_speed_weight: float = 1.5  # Speed tracking
    
    # Temperature for softmax weighting
    temperature: float = 0.5
    
    # Control limits
    steering_kp: float = 1.9
    steering_kd: float = 0.15
    steering_feedforward_gain: float = 0.95
    steer_yaw_rate_damping: float = 0.10
    
    throttle_kp: float = 0.30
    throttle_ki: float = 0.08
    throttle_kd: float = 0.02
    throttle_feedforward_gain: float = 0.18
    
    throttle_min: float = -0.35
    throttle_max: float = 1.00
    throttle_rate_limit_per_s: float = 2.5
    steering_rate_limit_per_s: float = 6.0
    
    # Filtering
    yaw_rate_low_pass_alpha: float = 0.35
    avoid_mode_throttle_cap: float = 0.50
    
    # Exponential moving average for control smoothing
    control_ema_alpha: float = 0.25


@dataclass
class MissionConfig:
    """Mission management parameters."""
    # Sensor timeouts (more granular than v1)
    sensor_timeout_s: float = 0.40
    lidar_timeout_s: float = 0.35
    imu_timeout_s: float = 0.30
    encoder_timeout_s: float = 0.40
    steering_timeout_s: float = 0.70
    camera_timeout_s: float = 0.50
    
    # Mode transitions
    bootstrap_time_s: float = 0.30
    safety_clearance_enter_m: float = 0.40
    safety_clearance_exit_m: float = 0.70
    safety_ttc_enter_s: float = 0.50
    safety_ttc_exit_s: float = 1.00
    safety_brake_hold_s: float = 0.40
    
    # Staleness handling
    stale_cycles_before_brake: int = 6
    min_pose_confidence: float = 0.12
    min_track_confidence: float = 0.22
    min_centerline_points: int = 5
    
    # Avoid mode hysteresis
    avoid_enter_consecutive_scans: int = 4
    avoid_exit_consecutive_scans: int = 8
    
    # Recovery (disabled by default)
    recovery_enabled: bool = False
    recovery_reverse_throttle: float = -0.12
    recovery_duration_s: float = 0.60
    
    # Graceful degradation
    degraded_mode_speed_factor: float = 0.5


@dataclass
class StackConfigV2:
    """Complete stack configuration."""
    vehicle: VehicleGeometry = field(default_factory=VehicleGeometry)
    localization: LocalizationConfig = field(default_factory=LocalizationConfig)
    lidar: LidarConfig = field(default_factory=LidarConfig)
    camera: CameraConfig = field(default_factory=CameraConfig)
    planner: PlannerConfig = field(default_factory=PlannerConfig)
    controller: ControllerConfig = field(default_factory=ControllerConfig)
    mission: MissionConfig = field(default_factory=MissionConfig)
