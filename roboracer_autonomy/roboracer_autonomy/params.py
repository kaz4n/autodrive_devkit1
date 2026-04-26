from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class VehicleGeometry:
    wheelbase_m: float = 0.324  # Axle-to-axle distance; increasing makes turns wider/more stable, decreasing makes turns tighter/more twitchy.
    track_width_m: float = 0.236  # Left-right wheel spacing; increasing improves lateral stability, decreasing increases rollover/slip tendency.
    wheel_radius_m: float = 0.059  # Wheel radius for speed/kinematics conversion; increasing raises speed per wheel rotation, decreasing lowers it.
    car_length_m: float = 0.500  # Bumper-to-bumper length used by footprint checks; increasing is more conservative near obstacles, decreasing is less conservative.
    car_width_m: float = 0.270  # Vehicle width used by clearance logic; increasing reduces accepted gaps, decreasing allows tighter gaps.
    max_steer_angle_rad: float = 0.5236  # Physical steering limit; increasing allows tighter cornering commands, decreasing limits agility.


@dataclass
class LidarConfig:
    focus_half_angle_deg: float = 110.0  # Half-angle of lidar region of interest; increasing uses wider FOV, decreasing focuses more forward.
    bubble_radius_m: float = 0.35  # Safety bubble around obstacles; increasing keeps more distance, decreasing allows closer passes.
    min_free_distance_m: float = 0.20  # Minimum distance considered traversable; increasing is stricter on free space, decreasing is more permissive.
    forward_sector_deg: float = 12.0  # Center-forward sector width for frontal checks; increasing averages more rays, decreasing reacts to narrow center rays.
    stop_distance_m: float = 0.55  # Distance threshold for normal stop behavior; increasing stops earlier, decreasing stops later.
    hard_stop_distance_m: float = 0.35  # Immediate stop distance threshold; increasing triggers emergency stop earlier, decreasing allows closer approach.
    hard_ttc_s: float = 0.45  # Emergency time-to-collision threshold; increasing hard-stops sooner, decreasing hard-stops later.
    caution_ttc_s: float = 0.90  # Caution TTC threshold for slowdown; increasing enters caution sooner, decreasing stays aggressive longer.
    leak_fill_max_bins: int = 6  # Max lidar gap bins to bridge as continuous space; increasing fills larger holes, decreasing preserves discontinuities.
    smoothing_kernel: int = 5  # Lidar smoothing window size (bins); increasing smooths noise but blurs detail, decreasing keeps detail but more noise.
    range_max_clip_m: float = 8.0  # Max lidar distance considered; increasing uses farther points, decreasing prioritizes near-field data.
    lane_width_slowdown_m: float = 1.1  # Estimated free-lane width to begin slowing; increasing slows in wider corridors, decreasing waits for tighter spaces.
    lane_width_confident_m: float = 1.4  # Lane width considered comfortable; increasing demands more room, decreasing accepts narrower corridors.


@dataclass
class CameraConfig:
    enabled: bool = True  # Enables camera contribution to steering; True fuses vision, False relies on lidar-only behavior.
    roi_vertical_start: float = 0.38  # Top crop ratio for image ROI; increasing looks lower/nearer, decreasing includes more far-field view.
    canny_low: int = 50  # Lower Canny threshold; increasing ignores weak edges, decreasing detects more faint edges/noise.
    canny_high: int = 140  # Upper Canny threshold; increasing keeps only strong edges, decreasing accepts weaker edges.
    hough_threshold: int = 20  # Votes needed to accept a line; increasing yields fewer/cleaner lines, decreasing yields more/noisier lines.
    hough_min_line_length: int = 18  # Minimum detected segment length; increasing filters short clutter, decreasing keeps short features.
    hough_max_line_gap: int = 10  # Max gap to connect collinear segments; increasing merges broken lines, decreasing keeps segments separate.
    min_slope_abs: float = 0.20  # Minimum absolute slope for candidate lane lines; increasing rejects flatter lines, decreasing admits flatter lines.
    heading_scale_rad: float = 0.65  # Scale from visual heading error to steering bias; increasing strengthens camera steering effect, decreasing weakens it.
    max_fused_weight: float = 0.25  # Max camera weight in lidar-camera fusion; increasing trusts camera more, decreasing prioritizes lidar.


@dataclass
class PlannerConfig:
    max_speed_mps: float = 4.0  # Global planner speed cap; increasing allows faster motion, decreasing enforces lower top speed.
    gap_mode_speed_mps: float = 2.4  # Speed while gap-following/uncertain pathing; increasing is more aggressive in clutter, decreasing is safer.
    straight_speed_mps: float = 4.6  # Preferred speed on straight sections; increasing boosts straight-line pace, decreasing reduces it.
    min_speed_mps: float = 0.0  # Lower bound on commanded speed; increasing prevents very low-speed creep, decreasing allows slower commands.
    nominal_lookahead_m: float = 1.10  # Baseline path preview distance; increasing smooths but can cut corners, decreasing tracks tightly but can oscillate.
    min_lookahead_m: float = 0.75  # Minimum adaptive lookahead; increasing reduces responsiveness in tight turns, decreasing increases responsiveness.
    max_lookahead_m: float = 2.20  # Maximum adaptive lookahead; increasing smooths high-speed steering, decreasing keeps it more reactive.
    lookahead_speed_gain: float = 0.18  # How much lookahead grows with speed; increasing increases speed-dependent smoothing, decreasing flattens adaptation.
    lateral_accel_limit_mps2: float = 4.5  # Lateral acceleration safety cap; increasing permits harder cornering, decreasing forces more corner slowdown.
    clearance_speed_gain: float = 1.25  # Speed gain from obstacle clearance; increasing speeds up in open space, decreasing reduces clearance-based acceleration.
    steering_bias_limit_rad: float = 0.85  # Clamp on planner steering bias; increasing allows stronger steering corrections, decreasing limits authority.
    gap_activation_angle_rad: float = 0.30  # Angle error to switch into gap-handling logic; increasing switches less often, decreasing switches earlier.
    camera_confidence_threshold: float = 0.25  # Minimum confidence to trust camera input; increasing requires cleaner vision, decreasing accepts noisier vision.


@dataclass
class ControllerConfig:
    throttle_kp: float = 0.22  # Proportional throttle gain; increasing speeds error correction but may overshoot, decreasing is smoother but slower.
    throttle_ki: float = 0.05  # Integral throttle gain for steady-state error; increasing removes bias faster but can wind up, decreasing reduces windup risk.
    throttle_kd: float = 0.02  # Derivative throttle damping; increasing damps transients/noise sensitivity, decreasing gives sharper but less damped response.
    throttle_feedforward_gain: float = 0.18  # Feedforward throttle from target speed; increasing improves promptness but may over-command, decreasing relies more on PID feedback.
    steer_yaw_rate_damping: float = 0.08  # Steering damping using yaw-rate feedback; increasing reduces oscillation but can feel sluggish, decreasing is more agile but may wobble.
    throttle_rate_limit_per_s: float = 1.4  # Max throttle change per second; increasing allows snappier accel/brake transitions, decreasing smooths commands.
    steering_rate_limit_per_s: float = 4.0  # Max steering change rate; increasing turns wheel faster, decreasing softens steering dynamics.


@dataclass
class MissionConfig:
    sensor_timeout_s: float = 0.35  # Max allowed age for core sensor data; increasing tolerates stale data longer, decreasing fails safe sooner.
    camera_timeout_s: float = 0.45  # Max allowed age for camera data; increasing keeps vision active through delays, decreasing drops vision sooner.
    bootstrap_time_s: float = 0.20  # Startup hold before full autonomy; increasing gives sensors more settle time, decreasing starts control sooner.
    recovery_enabled: bool = False  # Enables automatic unstuck routine; True allows reverse recovery, False disables recovery maneuvers.
    recovery_reverse_throttle: float = -0.15  # Reverse throttle during recovery; more negative reverses harder, less negative reverses more gently.
    recovery_duration_s: float = 0.50  # Duration of recovery reverse action; increasing backs up longer, decreasing backs up briefly.


@dataclass
class StackConfig:
    vehicle: VehicleGeometry = field(default_factory=VehicleGeometry)  # Vehicle geometry bundle; tuning values above changes kinematics and clearance behavior.
    lidar: LidarConfig = field(default_factory=LidarConfig)  # Lidar perception bundle; tuning values above changes obstacle sensitivity and safety margins.
    camera: CameraConfig = field(default_factory=CameraConfig)  # Camera perception bundle; tuning values above changes vision feature detection and fusion strength.
    planner: PlannerConfig = field(default_factory=PlannerConfig)  # Planning bundle; tuning values above changes speed policy and steering target generation.
    controller: ControllerConfig = field(default_factory=ControllerConfig)  # Low-level control bundle; tuning values above changes throttle/steering responsiveness.
    mission: MissionConfig = field(default_factory=MissionConfig)  # Mission/safety bundle; tuning values above changes timeout handling and recovery behavior.
