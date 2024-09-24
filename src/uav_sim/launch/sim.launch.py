import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():

    pkg_ros_gz_sim_demos = get_package_share_directory('uav_sim')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    world_path = os.path.join(pkg_ros_gz_sim_demos, 'sdf', 'world.sdf')
    uav_path = os.path.join(pkg_ros_gz_sim_demos, 'urdf', 'pioneer.urdf')

    with open(world_path, 'r') as infp:
        world_desc = infp.read()

    with open(uav_path, 'r') as infp:
        uav_desc = infp.read()

    rviz_launch_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='launch rviz'
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'),
        ),
        launch_arguments={'gz_args': PathJoinSubstitution([
            pkg_ros_gz_sim_demos,
            'sdf',
            'world.sdf'
        ])}.items(),
    )

    # Get the parser plugin convert sdf to urdf using robot_description topic
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': uav_desc},
        ]
    )

    # Launch rviz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        # arguments=['-d', os.path.join(pkg_ros_gz_sim_demos, 'rviz', 'vehicle.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[
            {'use_sim_time': True},
        ]
    )

    robot = ExecuteProcess(
        cmd=['ros2', 'run', 'ros_gz_sim', 'create', '-topic', 'robot_description', '-z', '0.2'],
        name='spawn rouavot',
        output='both'
    )

    joint_state_pub = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher'
    )

    # A gui tool for easy tele-operation.
    robot_steering = Node(
        package='rqt_robot_steering',
        executable='rqt_robot_steering',
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