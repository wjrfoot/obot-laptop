"""Verify the mapping settings and RViz display configuration."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory

import yaml


ROOT = Path(__file__).resolve().parents[1]


def load_yaml(path):
    """Load a ROS parameter or RViz YAML file."""
    with path.open(encoding='utf-8') as stream:
        return yaml.safe_load(stream)


def test_slam_parameters_match_defaults_with_requested_overrides():
    """Keep installed asynchronous mapper defaults apart from our settings."""
    default_path = (
        Path(get_package_share_directory('slam_toolbox')) / 'config' /
        'mapper_params_online_async.yaml')
    expected = load_yaml(default_path)['slam_toolbox']['ros__parameters']
    expected.update({
        'mode': 'mapping',
        'scan_topic': '/scan_filtered',
        'map_frame': 'map',
        'odom_frame': 'odom',
        'base_frame': 'base_link',
        'use_sim_time': False,
        'resolution': 0.05,
        'max_laser_range': 12.0,
        'minimum_travel_distance': 0.10,
        'minimum_travel_heading': 0.10,
        'map_update_interval': 2.0,
        'do_loop_closing': True,
    })
    actual = load_yaml(ROOT / 'config' / 'obot_slam.yaml')
    assert actual['slam_toolbox']['ros__parameters'] == expected


def test_rviz_has_required_displays_and_topics():
    """Display the map, filtered scan, robot description, and TF in map."""
    config = load_yaml(ROOT / 'rviz' / 'obot_slam.rviz')
    manager = config['Visualization Manager']
    assert manager['Global Options']['Fixed Frame'] == 'map'
    displays = {display['Class']: display for display in manager['Displays']}
    assert displays['rviz_default_plugins/Map']['Topic']['Value'] == '/map'
    assert (
        displays['rviz_default_plugins/LaserScan']['Topic']['Value']
        == '/scan_filtered')
    robot = displays['rviz_default_plugins/RobotModel']
    assert robot['Description Source'] == 'Topic'
    assert robot['Description Topic']['Value'] == '/robot_description'
    for name in ('Map', 'LaserScan', 'RobotModel', 'TF'):
        assert displays['rviz_default_plugins/' + name]['Enabled']
