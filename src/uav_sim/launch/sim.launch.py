import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    # Get paths for package directories
    pkg_ros_gz_sim_uav_sim = get_package_share_directory('uav_sim')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Define paths for meshes, SDF world file, and URDF UAV model
    meshes_path = os.path.join(pkg_ros_gz_sim_uav_sim, 'meshes')
    world_path = os.path.join(pkg_ros_gz_sim_uav_sim, 'sdf', 'world.sdf')
    uav_path = os.path.join(pkg_ros_gz_sim_uav_sim, 'urdf', 'pioneer.urdf')

    # Read world description from the SDF file
    with open(world_path, 'r') as infp:
        world_desc = infp.read()

    # Read UAV model description from the URDF file and replace mesh path placeholder
    with open(uav_path, 'r') as infp:
        uav_desc = infp.read()
    uav_desc = uav_desc.replace('MESH_PATH', meshes_path)

    # Declare launch argument for RViz option
    rviz_launch_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Launch RViz'
    )

    # Include Gazebo simulation launch file and specify the world SDF file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'),
        ),
        launch_arguments={'gz_args': PathJoinSubstitution([
            pkg_ros_gz_sim_uav_sim,
            'sdf',
            'world.sdf'
        ])}.items(),
    )

    # Node for publishing the robot's state using robot_state_publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': uav_desc},  # Send the UAV description to the robot_description topic
        ]
    )

    # Node to launch RViz if specified in the launch argument
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[
            {'use_sim_time': True},
        ]
    )

    # Execute process to spawn the UAV into the simulation environment
    uav = ExecuteProcess(
        cmd=['ros2', 'run', 'ros_gz_sim', 'create', '-topic', 'robot_description', '-z', '0.2'],
        name='spawn_uav',
        output='both'
    )

    # Node to publish joint states for the robot
    joint_state_pub = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher'
    )

    # Node to launch a GUI tool for tele-operating the robot
    robot_steering = Node(
        package='rqt_robot_steering',
        executable='rqt_robot_steering',
    )

    # Return LaunchDescription object containing all the defined launch entities
    return LaunchDescription([
        rviz_launch_arg,        # Argument to enable RViz
        gazebo,                 # Gazebo simulator with specified world
        uav,                    # Process to spawn the UAV into Gazebo
        robot_state_publisher,  # Publishes robot states
        joint_state_pub,        # Publishes joint states for robot
        rviz,                   # RViz for visualization
        robot_steering          # GUI tool for controlling the robot
    ])
