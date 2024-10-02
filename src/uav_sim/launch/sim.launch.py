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

    # Define paths for meshes, SDF world file, URDF UAV model, and GUI templates
    meshes_path = os.path.join(pkg_ros_gz_sim_uav_sim, 'meshes')
    world_path = os.path.join(pkg_ros_gz_sim_uav_sim, 'sdf', 'world.sdf')
    uav_path = os.path.join(pkg_ros_gz_sim_uav_sim, 'urdf', 'pioneer.urdf')
    rviz_config_path = os.path.join(pkg_ros_gz_sim_uav_sim, 'config', 'config.rviz')
    gazebo_config_path = os.path.join(pkg_ros_gz_sim_uav_sim, 'config', 'gazebo.config')
    with open(world_path, 'r') as infp:
        world_desc = infp.read()
    with open(uav_path, 'r') as infp:
        uav_desc = infp.read()
    uav_desc = uav_desc.replace('MESH_PATH', meshes_path) # Replace mesh path placeholder

    # Declare launch argument for RViz
    launch_rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Launch RViz'
    )

    # Launch RViz
    launch_rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],  # Specify custom RViz config file
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[
            {'use_sim_time': True},
        ]
    )

    # Launch Gazebo with specified world file
    launch_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'),
        ),
        launch_arguments={
            'gz_args': PathJoinSubstitution([pkg_ros_gz_sim_uav_sim, 'sdf', 'world.sdf']),
            'config': gazebo_config_path  # Pass custom Gazebo configuration file
        }.items(),
    )

    # Spawn the UAV into the simulation environment
    spawn_uav = ExecuteProcess(
        cmd=['ros2', 'run', 'ros_gz_sim', 'create', '-topic', 'robot_description', '-z', '0.2'],
        name='spawn_uav',
        output='both'
    )

    # Initialise bridge
    sim_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/lidar@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
        ],
        output='screen'
    )

    # Publish the UAV's state for synchronisation
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

    # Publish the simulation's state for synchronisation
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher'
    )

    # Launch UAV controls
    robot_steering = Node(
        package='rqt_robot_steering',
        executable='rqt_robot_steering',
    )

    # Return LaunchDescription object containing all the defined launch entities
    return LaunchDescription([
        launch_rviz_arg,
        launch_rviz,
        launch_gazebo,
        spawn_uav,
        sim_bridge,
        robot_state_publisher,
        joint_state_publisher,
        robot_steering
    ])
