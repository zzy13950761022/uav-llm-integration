import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Locate the uav_sim package's launch file
    uav_sim_share = get_package_share_directory('uav_sim')
    uav_sim_launch = os.path.join(uav_sim_share, 'launch', 'uav_sim.launch.py')
    
    # Locate the llm_integration package's launch file
    llm_integration_share = get_package_share_directory('llm_integration')
    llm_integration_launch = os.path.join(llm_integration_share, 'launch', 'llm_integration.launch.py')
    
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(uav_sim_launch)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(llm_integration_launch)
        )
    ])
