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

    # Launch a dummy instance of Gazebo to warm up the system.
    dummy_gazebo = ExecuteProcess(
        cmd=[
            'gz', 'sim', '-r', os.path.join(worlds_path, 'world.sdf'),
            '--gui-config', os.path.join(config_path, 'gaz.config'),
        ],
        output='screen'
    )

    # Shutdown dummy Gazebo after 3 seconds.
    shutdown_dummy = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=['pkill', '-f', 'gz sim'],
                output='screen'
            )
        ]
    )
    
    # Launch the real Gazebo simulation after a short delay
    gz_sim = TimerAction(
        period=4.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'gz', 'sim', '-r', os.path.join(worlds_path, 'world.sdf'),
                    '--gui-config', os.path.join(config_path, 'gaz.config'),
                    # '--render-engine', 'ogre2'
                ],
                output='screen'
            )
        ]
    )
    
    # Launch static transform publisher (without sim time) after 1 second delay
    # This publishes a constant transform from 'laser_frame' to 'pioneer/base_link/gpu_lidar'
    static_tf_lidar = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='static_tf_lidar',
                arguments=[
                    '--x', '0', '--y', '0', '--z', '0',
                    '--roll', '0', '--pitch', '0', '--yaw', '0',
                    '--frame-id', 'laser_frame',
                    '--child-frame-id', 'pioneer/base_link/gpu_lidar'
                ],
                output='screen'
            )
        ]
    )
    
    # Spawn the robot entity in Gazebo after a relative 2-second delay
    spawn_entity = TimerAction(
        period=6.0,
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
    
    # Launch robot_state_publisher after a relative 2-second delay
    robot_state_publisher = TimerAction(
        period=6.0,
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
    
    # Launch joint_state_publisher after a relative 3-second delay
    joint_state_publisher = TimerAction(
        period=7.0,
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
    
    # Launch the ROS–Gazebo bridge after a relative 2-second delay
    bridge_node = TimerAction(
        period=7.0,
        actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=[
                    '/lidar@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
                    '/imu@sensor_msgs/msg/Imu@ignition.msgs.IMU',
                    '/model/pioneer/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
                    '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
                    '/camera@sensor_msgs/msg/Image@ignition.msgs.Image',
                    '/model/pioneer/tf@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',
                    '/clock@rosgraph_msgs/msg/Clock@ignition.msgs.Clock',
                ],
                parameters=[{'use_sim_time': True}],
                remappings=[
                    ('/cmd_vel', '/cmd_vel'),
                    ('/model/pioneer/odometry', '/odom'),
                    ('/model/pioneer/tf', '/tf')
                ],
                output='screen'
            )
        ]
    )
    
    # Launch RViz after a relative 5-second delay to allow time for TF publishers to populate the cache
    rviz_node = TimerAction(
        period=9.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz_node',
                arguments=['-d', os.path.join(config_path, 'rviz_config.rviz')],
                parameters=[{'use_sim_time': True}],
                output='screen'
            )
        ]
    )
    
    return LaunchDescription([
        dummy_gazebo,
        shutdown_dummy,
        gz_sim,
        static_tf_lidar,
        # static_tf_odom,
        spawn_entity,
        robot_state_publisher,
        joint_state_publisher,
        bridge_node,
        rviz_node,
    ])
