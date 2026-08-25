import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory("nc_bringup"),
        "config",
        "vision2.yaml",
    )

    return LaunchDescription(
        [
            Node(
                package="nc_vision",
                executable="vision2_node",
                name="vision2_node",
                output="screen",
                parameters=[config_path, {"use_sim_time": True}],
            )
        ]
    )
