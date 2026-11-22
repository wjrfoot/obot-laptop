from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    spray_args = ['cross_spray_sec', 'circle_spray_sec',
                  'triangle_spray_sec', 'square_spray_sec']
    params = {
        'output_dir': LaunchConfiguration('output_dir'),
        'odom_topic': LaunchConfiguration('odom_topic'),
        'joy_topic': LaunchConfiguration('joy_topic'),
        'distance_unit': LaunchConfiguration('distance_unit'),
    }
    for arg in spray_args:
        params[arg.replace('_spray_sec', '_duration_sec')] = ParameterValue(
            LaunchConfiguration(arg), value_type=float)

    return LaunchDescription([
        DeclareLaunchArgument('output_dir', default_value='~/odom_records'),
        DeclareLaunchArgument('odom_topic', default_value='/diff_drive_controller/odom'),
        DeclareLaunchArgument('joy_topic', default_value='/joy'),
        DeclareLaunchArgument('distance_unit', default_value='feet'),
        *[DeclareLaunchArgument(arg, default_value='0.0') for arg in spray_args],
        Node(
            package='obot',
            executable='odom_spray_record',
            name='odom_spray_record',
            output='screen',
            parameters=[params],
        ),
    ])
