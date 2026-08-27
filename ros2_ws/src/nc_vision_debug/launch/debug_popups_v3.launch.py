"""Launch both final NetClean V3 vision debug popup nodes."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory("nc_vision_debug")
    config_path = os.path.join(package_share, "config", "debug_popups_v3.yaml")

    return LaunchDescription(
        [
            Node(
                package="nc_vision_debug",
                executable="vision1_debug_popup_v3",
                name="vision1_debug_popup",
                output="screen",
                parameters=[config_path],
            ),
            Node(
                package="nc_vision_debug",
                executable="vision2_debug_popup_v3",
                name="vision2_debug_popup",
                output="screen",
                parameters=[config_path],
            ),
        ]
    )