import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Locate the uav_actual package's launch file
    uav_actual_share = get_package_share_directory('uav_actual')
    uav_actual_launch = os.path.join(uav_actual_share, 'launch', 'uav_actual.launch.py')
    
    # Locate the llm_integration package's launch file
    llm_integration_share = get_package_share_directory('llm_integration')
    llm_integration_launch = os.path.join(llm_integration_share, 'launch', 'llm_integration.launch.py')
    
    # Locate the deadman_safety package's launch file
    deadman_safety_share = get_package_share_directory('deadman_safety')
    deadman_safety_launch = os.path.join(deadman_safety_share, 'launch', 'deadman_safety.launch.py')
    
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(uav_actual_launch)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(llm_integration_launch)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(deadman_safety_launch)
        )
    ])