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
    config_path = pkg_uav_sim + '/config'
    
    # Gazebo Harmonic launch with GUI config
    gz_sim = ExecuteProcess(
        cmd=[
            'gz', 'sim', '-r', 
            worlds_path + '/world.sdf',
            '--gui-config', config_path + '/gaz.config',
            '--render-engine', 'ogre2'
        ],
        output='screen'
    )

    # ROS GZ bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/lidar@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
                    '/imu@sensor_msgs/msg/Imu@ignition.msgs.IMU',
                    '/model/pioneer3at_body/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
                    '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
                    '/camera@sensor_msgs/msg/Image@ignition.msgs.Image',
                    '/model/pioneer3at_body/tf@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',
                    '/clock@rosgraph_msgs/msg/Clock@ignition.msgs.Clock',],
        output='screen',
        remappings=[('/cmd_vel','/cmd_vel'),
                    ('/model/pioneer3at_body/odometry','/odom'),
                    ('/model/pioneer3at_body/tf','/tf')
        ]
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': open(models_path + '/pioneer.urdf').read()
        }],
        remappings=[
            ('/joint_states', '/pioneer/joint_states')
        ]
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
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', config_path + '/rviz_config.rviz'],
        output='screen'
    )

    # Joint state publisher
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