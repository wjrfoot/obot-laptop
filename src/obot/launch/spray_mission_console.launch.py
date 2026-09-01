from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='obot',
            executable='spray_mission_console',
            name='spray_mission_console',
            output='screen',
            parameters=[{
                'map_frame': 'map',
                'map_topic': '/map',
                'costmap_topic': '/global_costmap/costmap',
                'heatmap_topic': '/wifi/heatmap_markers',
                'initialpose_topic': '/initialpose',
                'amcl_pose_topic': '/amcl_pose',
                'spray_topic': '/spray/event',
                'valve_command_topic': '/spray/valve_command',
            }],
        ),
    ])
