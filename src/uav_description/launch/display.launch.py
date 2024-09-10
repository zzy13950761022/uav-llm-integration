import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Define paths to your URDF and RViz config files
    pkg_path = get_package_share_directory('uav_description')
    urdf_file = os.path.join(pkg_path, 'urdf', 'pioneer.urdf')
    rviz_config_file = os.path.join(pkg_path, 'rviz', 'default.rviz')

    # Ensure the URDF file exists
    with open(urdf_file, 'r') as infp:
        robot_description = infp.read()

    return LaunchDescription([
        # Node for RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]
        ),
        
        # # Node for Robot State Publisher
        # Node(
        #     package='robot_state_publisher',
        #     executable='robot_state_publisher',
        #     name='robot_state_publisher',
        #     output='screen',
        #     parameters=[{'robot_description': robot_description}]
        # ),

        # # Optional: Joint State Publisher (only if you have joints in your URDF)
        # Node(
        #     package='joint_state_publisher_gui',
        #     executable='joint_state_publisher_gui',
        #     name='joint_state_publisher',
        #     output='screen'
        # ),
    ])