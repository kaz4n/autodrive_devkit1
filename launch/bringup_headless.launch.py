# Copyright (c) 2026, Tinker Twins
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

################################################################################

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    max_speed_mps = LaunchConfiguration('max_speed_mps')
    control_hz = LaunchConfiguration('control_hz')
    maps_root = LaunchConfiguration('maps_root')
    track_name = LaunchConfiguration('track_name')
    external_pose_topic = LaunchConfiguration('external_pose_topic')
    use_external_pose = LaunchConfiguration('use_external_pose')
    vehicle_prefix = LaunchConfiguration('vehicle_prefix')

    return LaunchDescription([
        DeclareLaunchArgument('max_speed_mps', default_value='10.0'),
        DeclareLaunchArgument('control_hz', default_value='15.0'),
        DeclareLaunchArgument('maps_root', default_value='~/.roboracer_track_maps'),
        DeclareLaunchArgument('track_name', default_value=''),
        DeclareLaunchArgument('external_pose_topic', default_value=''),
        DeclareLaunchArgument('use_external_pose', default_value='true'),
        DeclareLaunchArgument('vehicle_prefix', default_value='/autodrive/roboracer_1'),
        Node(
            package='autodrive_roboracer',
            executable='autodrive_bridge',
            name='autodrive_bridge',
            emulate_tty=True,
            output='screen',
        ),
        Node(
            package='roboracer_autonomy',
            executable='autonomy_node',
            name='roboracer_autonomy',
            emulate_tty=True,
            output='screen',
            parameters=[{
                'max_speed_mps': ParameterValue(max_speed_mps, value_type=float),
                'control_hz': ParameterValue(control_hz, value_type=float),
                'maps_root': ParameterValue(maps_root, value_type=str),
                'track_name': ParameterValue(track_name, value_type=str),
                'external_pose_topic': ParameterValue(external_pose_topic, value_type=str),
                'use_external_pose': ParameterValue(use_external_pose, value_type=bool),
                'vehicle_prefix': ParameterValue(vehicle_prefix, value_type=str),
            }],
        ),
    ])