from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='8888',
                              description='UDP port the ESP32 spray controller connects to'),
        DeclareLaunchArgument('domain_id', default_value='5',
                              description='ROS_DOMAIN_ID; must match the firmware'),
        SetEnvironmentVariable('ROS_DOMAIN_ID', LaunchConfiguration('domain_id')),
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            output='screen',
            arguments=['udp4', '--port', LaunchConfiguration('port')],
        ),
    ])
