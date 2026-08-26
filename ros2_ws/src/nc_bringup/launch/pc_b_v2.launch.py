"""Launch the opt-in NetClean V2 vision nodes without changing ROS APIs."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    config_dir = Path(get_package_share_directory("nc_bringup")) / "config"
    return LaunchDescription(
        [
            Node(
                package="nc_vision",
                executable="vision1_node_v2",
                # Keep the public ROS node name and parameter namespace stable.
                name="vision1_node",
                output="screen",
                parameters=[str(config_dir / "vision1_v2.yaml")],
            ),
            Node(
                package="nc_vision",
                executable="vision2_node_v2",
                # Robot/control integrations continue to see /vision2_node.
                name="vision2_node",
                output="screen",
                parameters=[str(config_dir / "vision2_v2.yaml")],
            ),
        ]
    )
