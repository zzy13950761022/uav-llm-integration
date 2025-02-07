import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    pkg_uav_sim = get_package_share_directory('uav_sim')
    models_path = os.path.join(pkg_uav_sim, 'models')
    worlds_path = os.path.join(pkg_uav_sim, 'worlds')
    config_path = os.path.join(pkg_uav_sim, 'config')
    
    # Start Gazebo simulation immediately
    gz_sim = ExecuteProcess(
        cmd=[
            'gz', 'sim', '-r', os.path.join(worlds_path, 'world.sdf'),
            '--gui-config', os.path.join(config_path, 'gaz.config'),
            '--render-engine', 'ogre2'
        ],
        output='screen'
    )
    
    # Launch static transform publisher (without sim time) after 1 second delay
    # This publishes a constant transform from 'laser_frame' to 'pioneer/base_link/gpu_lidar'
    static_tf_publisher = TimerAction(
        period=1.0,
        actions=[
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='static_transform_publisher',
                arguments=['0', '0', '0', '0', '0', '0', 'laser_frame', 'pioneer/base_link/gpu_lidar'],
                # Do NOT use sim time here so the transform is available immediately.
                output='screen'
            )
        ]
    )
    
    # Spawn the robot entity in Gazebo after a 2-second delay
    spawn_entity = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-name', 'pioneer',
                    '-topic', 'robot_description',
                    '-x', '0', '-y', '0', '-z', '0.5'
                ],
                parameters=[{'use_sim_time': True}],
                output='screen'
            )
        ]
    )
    
    # Launch robot_state_publisher after a 2-second delay
    robot_state_publisher = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                parameters=[{
                    'use_sim_time': True,
                    'robot_description': open(os.path.join(models_path, 'pioneer.urdf')).read()
                }],
                output='screen'
            )
        ]
    )
    
    # Launch joint_state_publisher after a 2-second delay
    joint_state_publisher = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='joint_state_publisher',
                executable='joint_state_publisher',
                name='joint_state_publisher',
                parameters=[{'use_sim_time': True}],
                output='screen'
            )
        ]
    )
    
    # Launch the ROS–Gazebo bridge after a 2-second delay
    bridge = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=[
                    '/lidar@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
                    '/imu@sensor_msgs/msg/Imu@ignition.msgs.IMU',
                    '/model/pioneer3at_body/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
                    '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
                    '/camera@sensor_msgs/msg/Image@ignition.msgs.Image',
                    '/model/pioneer3at_body/tf@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',
                    '/clock@rosgraph_msgs/msg/Clock@ignition.msgs.Clock',
                ],
                parameters=[{'use_sim_time': True}],
                remappings=[
                    ('/cmd_vel', '/cmd_vel'),
                    ('/model/pioneer3at_body/odometry', '/odom'),
                    ('/model/pioneer3at_body/tf', '/tf')
                ],
                output='screen'
            )
        ]
    )
    
    # Launch RViz after a 5-second delay to allow time for TF publishers to populate the cache
    rviz = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', os.path.join(config_path, 'rviz_config.rviz')],
                parameters=[{'use_sim_time': True}],
                output='screen'
            )
        ]
    )
    
    return LaunchDescription([
        gz_sim,
        static_tf_publisher,
        spawn_entity,
        robot_state_publisher,
        joint_state_publisher,
        bridge,
        rviz,
    ])
