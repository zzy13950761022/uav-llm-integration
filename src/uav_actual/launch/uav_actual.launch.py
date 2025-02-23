import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    pkg_uav_actual = get_package_share_directory('uav_actual')
    config_path = os.path.join(pkg_uav_actual, 'config')
    models_path = os.path.join(pkg_uav_actual, 'models')
    
    # Launch robot_state_publisher after a 2-second delay
    robot_state_publisher = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                parameters=[{
                    'robot_description': open(os.path.join(models_path, 'pioneer.urdf')).read()
                }],
                output='screen'
            )
        ]
    )

    # Launch joint_state_publisher after a 3-second delay
    joint_state_publisher = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='joint_state_publisher',
                executable='joint_state_publisher',
                name='joint_state_publisher',
                parameters=[{'use_sim_time': False}],
                output='screen'
            )
        ]
    )
    
    # Launch RViz after a 5-second delay to allow time for TF publishers to populate the cache
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

    # Launch the Pioneer driver node (ARIA-based)
    aria_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='uav_actual',
                executable='ariaNode',
                name='ariaNode',
                output='screen',
                arguments=['-rp', '/dev/ttyUSB0']  # Passing the required serial port argument
            )
        ]
    )
    
    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher,
        rviz_node,
        aria_node,
    ])