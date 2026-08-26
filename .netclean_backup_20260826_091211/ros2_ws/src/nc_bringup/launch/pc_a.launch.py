from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nc_control',
            executable='control_node',
            name='control_node',
            output='screen',
            parameters=[
                {'use_sim_time': True}
            ],
        ),

        Node(
            package='nc_robot',
            executable='robot1_node',
            name='robot1_node',
            output='screen',
            parameters=[
                {'use_sim_time': True}
            ],
        ),

        Node(
            package='nc_robot',
            executable='robot2_node',
            name='robot2_node',
            output='screen',
            parameters=[
                {'use_sim_time': True}
            ],
        ),
    ])
