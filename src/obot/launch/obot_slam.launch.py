"""Launch the filtered scan, asynchronous mapping, and optional RViz."""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    """Start mapping with the installed SLAM Toolbox lifecycle launch."""
    obot = Path(get_package_share_directory('obot'))
    slam = Path(get_package_share_directory('slam_toolbox'))
    gtk_path = os.environ.get('GTK_PATH', '')
    rviz_prefix = 'env -u GTK_PATH' if '/snap/' in gtk_path else ''
    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz', default_value='true', description='Start mapping RViz'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            str(obot / 'launch' / 'obot_scan_filter.launch.py'))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(slam / 'launch' / 'online_async_launch.py')),
            launch_arguments={
                'slam_params_file': str(obot / 'config' / 'obot_slam.yaml'),
                'use_sim_time': 'false',
                'autostart': 'true',
                'use_lifecycle_manager': 'false',
            }.items(),
        ),
        Node(
            package='rviz2', executable='rviz2', name='obot_slam_rviz',
            arguments=['-d', str(obot / 'rviz' / 'obot_slam.rviz')],
            prefix=rviz_prefix,
            condition=IfCondition(LaunchConfiguration('rviz')),
            output='screen',
        ),
    ])
