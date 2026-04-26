#!/usr/bin/env python3
"""
Mission Manager V2 - State Machine with Granular Timeouts
Handles mode transitions, sensor monitoring, and safe fallbacks.
"""

from std_msgs.msg import String
import time
from typing import Optional


class MissionV2:
    def __init__(self, params):
        self.params = params
        
        # Current mode
        self.current_mode = String.DATA_IDLE
        self.previous_mode = String.DATA_IDLE
        
        # Timing
        self.start_time = time.time()
        self.mode_transition_start = None
        self.settled = False
        
        # Sensor status tracking
        self.lidar_available = False
        self.imu_available = False
        self.odom_available = False
        self.sensors_ok = False
        
    def update(self, sensors_ok: bool, has_lidar: bool, 
               has_imu: bool, has_odom: bool) -> str:
        """
        Update mission state machine based on sensor status.
        Returns current mode string.
        """
        # Update sensor status
        self.sensors_ok = sensors_ok
        self.lidar_available = has_lidar
        self.imu_available = has_imu
        self.odom_available = has_odom
        
        current_time = time.time()
        elapsed = current_time - self.start_time
        
        # ================================================================
        # Phase 1: Initialization (first few seconds)
        # ================================================================
        if elapsed < self.params.startup_settle_time:
            self.current_mode = String.DATA_INITIALIZING
            self.mode_transition_start = current_time
            return self.current_mode
        
        # ================================================================
        # Phase 2: Check for sensor failures -> Emergency
        # ================================================================
        if not sensors_ok:
            if self.current_mode != String.DATA_EMERGENCY:
                self.mode_transition_start = current_time
            self.current_mode = String.DATA_EMERGENCY
            return self.current_mode
        
        # ================================================================
        # Phase 3: Normal operation modes
        # ================================================================
        
        # If all sensors are available, we can go autonomous
        if has_lidar and has_imu and has_odom:
            # Transition to autonomous mode
            if self.current_mode in [String.DATA_IDLE, String.DATA_INITIALIZING]:
                if self.mode_transition_start is None:
                    self.mode_transition_start = current_time
                
                # Wait for transition delay before switching
                if current_time - self.mode_transition_start > self.params.mode_transition_delay:
                    self.previous_mode = self.current_mode
                    self.current_mode = String.DATA_AUTONOMOUS
                    self.mode_transition_start = None
            
            # Stay in autonomous mode
            elif self.current_mode == String.DATA_AUTONOMOUS:
                pass  # Remain in autonomous
            
            # Recover from emergency if sensors are back
            elif self.current_mode == String.DATA_EMERGENCY:
                if self.mode_transition_start is None:
                    self.mode_transition_start = current_time
                
                if current_time - self.mode_transition_start > self.params.mode_transition_delay:
                    self.previous_mode = self.current_mode
                    self.current_mode = String.DATA_AUTONOMOUS
                    self.mode_transition_start = None
        
        else:
            # Some sensors missing but not timed out yet
            # Stay in current mode or degrade gracefully
            if self.current_mode == String.DATA_AUTONOMOUS:
                # Can continue with reduced capability
                pass
        
        return self.current_mode
    
    def get_mode_description(self, mode: str) -> str:
        """Get human-readable description of mode."""
        descriptions = {
            String.DATA_IDLE: "Idle - waiting for command",
            String.DATA_INITIALIZING: "Initializing - settling sensors",
            String.DATA_AUTONOMOUS: "Autonomous - full self-driving",
            String.DATA_TRACKING: "Tracking - following reference",
            String.DATA_EMERGENCY: "Emergency - stopping safely",
            String.DATA_MANUAL: "Manual - operator control"
        }
        return descriptions.get(mode, f"Unknown mode: {mode}")
    
    def is_autonomous(self) -> bool:
        """Check if currently in autonomous mode."""
        return self.current_mode == String.DATA_AUTONOMOUS
    
    def is_emergency(self) -> bool:
        """Check if currently in emergency mode."""
        return self.current_mode == String.DATA_EMERGENCY
    
    def get_sensor_status(self) -> dict:
        """Get current sensor availability status."""
        return {
            'lidar': self.lidar_available,
            'imu': self.imu_available,
            'odom': self.odom_available,
            'all_ok': self.sensors_ok
        }
