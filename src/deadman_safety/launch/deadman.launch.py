import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    deadman_node = Node(
        package='deadman_safety',
        executable='deadman_node',
        name='deadman_node',
        output='screen'
    )
    return LaunchDescription([deadman_node])