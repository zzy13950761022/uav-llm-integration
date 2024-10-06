import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_uav_sim = get_package_share_directory('uav_sim')
    models_path = pkg_uav_sim + '/models'
    worlds_path = pkg_uav_sim + '/worlds'
    
    # Gazebo Harmonic launch
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', worlds_path + '/world.sdf'],
        output='screen'
    )

    # ROS GZ bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': open(models_path + '/pioneer.urdf').read()}]
    )

    # Spawn the robot in Gazebo Harmonic
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'pioneer', 
                   '-topic', 'robot_description', 
                   '-x', '0', '-y', '0', '-z', '0.5'],
        output='screen'
    )

    # RViz
    rviz_config_file = os.path.join(pkg_uav_sim, 'config', 'rviz_config.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher'
    )

    return LaunchDescription([
        gz_sim,
        bridge,
        robot_state_publisher,
        spawn_entity,
        rviz,
        joint_state_publisher
    ])