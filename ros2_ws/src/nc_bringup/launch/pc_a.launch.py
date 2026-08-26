from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('nc_bringup'),
        'config',
        'netclean.yaml',
    )
    config_arg = DeclareLaunchArgument(
        'config',
        default_value=default_config,
        description='NetClean PC A ROS parameter YAML',
    )
    config = LaunchConfiguration('config')

    return LaunchDescription([
        config_arg,
        Node(
            package='nc_control',
            executable='control_node',
            name='control_node',
            output='screen',
            parameters=[config],
        ),
        Node(
            package='nc_robot',
            executable='robot1_node',
            name='robot1_node',
            output='screen',
            parameters=[config],
        ),
        Node(
            package='nc_robot',
            executable='robot2_node',
            name='robot2_node',
            output='screen',
            parameters=[config],
        ),
    ])
