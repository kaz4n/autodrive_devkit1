"""Upgraded RoboRacer autonomy stack.

The stack is organized around a path-tracking pipeline instead of a purely reactive heading
blender:

wheel odometry prior -> optional external localization hook -> LiDAR track extraction ->
trajectory planning / raceline tracking -> adaptive pure pursuit controller -> follow-the-gap
fallback for blocked-path recovery.
"""
