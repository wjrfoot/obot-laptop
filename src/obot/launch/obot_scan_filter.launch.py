"""Publish a scan with the robot's rear sector masked."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription

from launch_ros.actions import Node


def generate_launch_description():
    """Start the standard LaserScan filter chain."""
    config = Path(get_package_share_directory('obot'))
    config = config / 'config' / 'scan_filter.yaml'
    return LaunchDescription([
        Node(
            package='laser_filters',
            executable='scan_to_scan_filter_chain',
            name='obot_scan_filter',
            parameters=[str(config)],
            remappings=[
                ('scan', '/scan'), ('scan_filtered', '/scan_filtered')],
            output='screen',
        ),
    ])
