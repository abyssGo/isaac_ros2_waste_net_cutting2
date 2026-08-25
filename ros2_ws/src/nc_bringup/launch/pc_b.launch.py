from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nc_vision',
            executable='vision1_node',
            name='vision1_node',
            output='screen',
            parameters=[
                {'use_sim_time': True}
            ],
        ),

        Node(
            package='nc_vision',
            executable='vision2_node',
            name='vision2_node',
            output='screen',
            parameters=[
                {'use_sim_time': True}
            ],
        ),
    ])
