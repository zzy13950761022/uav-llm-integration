import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    pkg_uav_actual = get_package_share_directory('uav_actual')
    config_path = os.path.join(pkg_uav_actual, 'config')
    models_path = os.path.join(pkg_uav_actual, 'models')
    
    # Launch the SICK LiDAR node
    sick_scan_node = Node(
        package='sick_scan_xd',
        executable='sick_scan_xd_node',
        name='sick_scan_xd',
        parameters=[os.path.join(config_path, 'sick_scan_config.yaml')],
        output='screen'
    )
    
    # Launch the Pioneer driver node (ARIA-based)
    driver_node = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='uav_actual',
                executable='driver_node',
                name='driver_node',
                output='screen'
            )
        ]
    )
    
    # Launch robot_state_publisher for the real robot
    robot_state_publisher = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                parameters=[{'robot_description': open(os.path.join(models_path, 'pioneer.urdf')).read()}],
                output='screen'
            )
        ]
    )
    
    # Launch RViz
    rviz_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', os.path.join(config_path, 'rviz_config.rviz')],
                parameters=[{'use_sim_time': False}],
                output='screen'
            )
        ]
    )
    
    return LaunchDescription([
        sick_scan_node,
        driver_node,
        robot_state_publisher,
        rviz_node
    ])