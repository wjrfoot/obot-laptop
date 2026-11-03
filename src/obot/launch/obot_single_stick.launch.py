from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    defaults = {
        'joy_topic': '/joy',
        'joy_dev': '/dev/input/js0',
        'cmd_vel_out': '/cmd_vel_teleop',
        'linear_axis': '1',
        'angular_axis': '0',
        'invert_linear': 'false',
        'invert_angular': 'false',
        'deadband': '0.50',
        'proportional': 'false',
        'track_width': '0.486',
        'max_linear': '0.5',
        'low_speed_scale': '0.50',
        'fast_mode_button': '5',
        'enable_button': '4',
        'require_enable': 'true',
        'joy_timeout_sec': '0.5',
    }

    arguments = [
        DeclareLaunchArgument(key, default_value=value)
        for key, value in defaults.items()
    ]

    parameters = {
        key: LaunchConfiguration(key)
        for key in defaults
        if key != 'joy_dev'
    }

    return LaunchDescription(arguments + [
        Node(
            package='obot',
            executable='single_stick_mixer_node',
            name='single_stick_mixer_node',
            output='screen',
            parameters=[parameters],
        ),
    ])
