from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    pkg_share = get_package_share_directory("obot_bringup")

    default_map = os.path.join(
        os.path.expanduser("~"),
        "Workspaces",
        "obot_ws",
        "maps",
        "lab_test.yaml",
    )

    amcl_params = os.path.join(
        pkg_share,
        "config",
        "amcl.yaml",
    )

    map_arg = DeclareLaunchArgument(
        "map",
        default_value=default_map,
        description="Full path to occupancy map YAML file",
    )

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            {
                "yaml_filename": LaunchConfiguration("map"),
                "use_sim_time": False,
            }
        ],
    )

    amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=[amcl_params],
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[
            {
                "use_sim_time": False,
                "autostart": True,
                "node_names": ["map_server", "amcl"],
            }
        ],
    )

    return LaunchDescription([
        map_arg,
        map_server,
        amcl,
        lifecycle_manager,
    ])
