"""RoboRacer free-space MPC stack.

Nominal autonomy path:
    wheel odometry -> LiDAR corridor extraction -> free-space MPC -> throttle/steering

The old discrete mission/planning/control pipeline has been retired from the nominal loop.
Compatibility shims remain only to avoid import breakage in older launch files.
"""

__all__ = [
    'RoboRacerAutonomyNode',
    'FreeSpaceMPC',
    'LidarTrackExtractor',
    'WheelOdometryEstimator',
]
