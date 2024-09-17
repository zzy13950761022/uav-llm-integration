import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():

    # Package directories
    pkg_uav_description = get_package_share_directory('uav_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # File paths for SDF and URDF
    sdf_file = os.path.join(pkg_uav_description, 'sdf', 'world.sdf')
    robot_file = os.path.join(pkg_uav_description, 'urdf', 'pioneer.urdf')

    # Read the URDF description for robot state publisher
    with open(robot_file, 'r') as infp:
        robot_desc = infp.read()

    # RViz launch argument
    rviz_launch_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Open RViz.'
    )

    # Gazebo launch using IncludeLaunchDescription to refer to gz_sim.launch.py
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'),
        ),
        launch_arguments={'gz_args': PathJoinSubstitution([
            pkg_uav_description, 'sdf', 'world.sdf'
        ])}.items(),
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True, 'robot_description': robot_desc}]
    )

    # RViz node
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[{'use_sim_time': True}],
    )

    # Spawn robot in Gazebo
    spawn_robot = ExecuteProcess(
        cmd=['ros2', 'run', 'ros_gz_sim', 'create', '-topic', 'robot_description', '-z', '0.2'],
        name='spawn_robot',
        output='screen'
    )

    # Joint state publisher
    joint_state_pub = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher'
    )

    # RQT robot steering tool
    robot_steering = Node(
        package="rqt_robot_steering",
        executable="rqt_robot_steering",
    )

    # Return the full LaunchDescription
    return LaunchDescription([
        rviz_launch_arg,
        gazebo,
        robot_state_publisher,
        spawn_robot,
        joint_state_pub,
        rviz,
        robot_steering
    ])
