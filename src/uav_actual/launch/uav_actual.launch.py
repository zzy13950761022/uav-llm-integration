import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    pkg_uav_actual = get_package_share_directory('uav_actual')
    config_path = os.path.join(pkg_uav_actual, 'config')
    models_path = os.path.join(pkg_uav_actual, 'models')

    # Start the SICK LiDAR driver
    sick_scan = Node(
        package='sick_scan_xd',
        executable='sick_scan_xd_node',
        name='sick_scan_xd',
        parameters=[os.path.join(config_path, 'sick_scan_config.yaml')],
        output='screen'
    )

    # Start the Phidget IMU driver
    phidget_imu = Node(
        package='phidget_spatial',
        executable='phidget_spatial_node',
        name='phidget_spatial',
        parameters=[os.path.join(config_path, 'phidget_spatial_config.yaml')],
        output='screen'
    )

    # Start the Luxonis DepthAI camera node
    depthai_cam = Node(
        package='luxonis_depth_ai',
        executable='depthai_node',
        name='depthai_camera',
        parameters=[os.path.join(config_path, 'depthai_config.yaml')],
        output='screen'
    )

    # Start robot_state_publisher with the actual Pioneer URDF
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

    # Start RViz
    rviz_config_path = os.path.join(config_path, 'rviz_config.rviz')
    rviz_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config_path],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        sick_scan,
        phidget_imu,
        depthai_cam,
        robot_state_publisher,
        rviz_node
    ])
