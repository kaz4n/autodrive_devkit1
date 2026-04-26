from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    max_speed_mps = LaunchConfiguration('max_speed_mps')
    control_hz = LaunchConfiguration('control_hz')
    mode = LaunchConfiguration('mode')

    return LaunchDescription([
        DeclareLaunchArgument('max_speed_mps', default_value='6.0'),
        DeclareLaunchArgument('control_hz',    default_value='40.0'),
        DeclareLaunchArgument('mode',          default_value='auto'),

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
                'control_hz':    ParameterValue(control_hz,    value_type=float),
                'mode':          ParameterValue(mode,          value_type=str),
                'vehicle_prefix': '/autodrive/roboracer_1',
            }],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', [FindPackageShare('autodrive_roboracer'), '/rviz',
                               '/autodrive_roboracer.rviz']],
        ),
    ])
