import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_uav_description = get_package_share_directory('uav_description')
    sdf_file = os.path.join(pkg_uav_description, 'sdf', 'world.sdf')
    robot_file = os.path.join(pkg_uav_description, 'urdf', 'pioneer.urdf')

    with open(robot_file, 'r') as infp:
        robot_desc = infp.read()

    rviz_launch_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Open RViz.'
    )

    # Launch Ignition Gazebo
    gazebo = ExecuteProcess(
        cmd=['/usr/bin/ign', 'gazebo', sdf_file], # TODO: This is dodgy AF
        name='ignition_gazebo',
        output='screen'
    )

    # Robot state publisher for URDF
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_desc},
        ]
    )

    # Spawn robot
    robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-z', '0.2'],
        output='screen'
    )

    # Joint state publisher
    joint_state_pub = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher'
    )

    # RQT robot steering
    robot_steering = Node(
        package="rqt_robot_steering",
        executable="rqt_robot_steering",
    )

    # Launch RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        rviz_launch_arg,
        gazebo,
        robot,
        robot_state_publisher,
        joint_state_pub,
        rviz,
        robot_steering
    ])