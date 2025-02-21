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

    custom_joy_node = Node(
        package='deadman_safety',
        executable='custom_joy_node',
        name='custom_joy_node',
        output='screen'
    )

    return LaunchDescription([deadman_node, custom_joy_node])