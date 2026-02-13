import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    Launch all sensors for autonomous rover
    Author: Sharan G S
    """
    
    pkg_share = get_package_share_directory('rover_sensors')
    
    # RPLidar
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'rplidar.launch.py')
        )
    )
    
    # ZED 2i Camera
    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'zed_camera.launch.py')
        )
    )
    
    return LaunchDescription([
        rplidar_launch,
        zed_launch
    ])
