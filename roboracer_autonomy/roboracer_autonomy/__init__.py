"""Reactive RoboRacer autonomy stack.

This package keeps the runtime stack competition-legal by avoiding restricted IPS and odometry
inputs during deployment. It combines encoder/IMU dead reckoning, LiDAR corridor and gap
perception, camera edge cues, a finite-state mission manager, reactive path planning and
pure-pursuit-style control with a speed PID.
"""
