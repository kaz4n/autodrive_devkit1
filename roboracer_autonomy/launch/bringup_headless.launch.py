from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_camera = LaunchConfiguration('use_camera')
    max_speed_mps = LaunchConfiguration('max_speed_mps')
    control_hz = LaunchConfiguration('control_hz')

    return LaunchDescription([
        DeclareLaunchArgument('use_camera', default_value='true'),
        DeclareLaunchArgument('max_speed_mps', default_value='4.0'),
        DeclareLaunchArgument('control_hz', default_value='1000.0'),
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
                'use_camera': ParameterValue(use_camera, value_type=bool),
                'max_speed_mps': ParameterValue(max_speed_mps, value_type=float),
                'control_hz': ParameterValue(control_hz, value_type=float),
            }],
        ),
    ])
