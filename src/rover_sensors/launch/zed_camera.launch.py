import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    Launch ZED 2i camera with ROS2 wrapper
    Author: Sharan G S
    """
    
    # Get ZED wrapper package
    zed_wrapper_dir = get_package_share_directory('zed_wrapper')
    
    camera_model = LaunchConfiguration('camera_model')
    
    declare_camera_model = DeclareLaunchArgument(
        'camera_model',
        default_value='zed2i',
        description='ZED camera model'
    )
    
    # Include ZED wrapper launch file
    zed_wrapper_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(zed_wrapper_dir, 'launch', 'zed_camera.launch.py')
        ),
        launch_arguments={
            'camera_model': camera_model,
            'publish_tf': 'true',
            'publish_map_tf': 'false',
            'depth_mode': 'NEURAL',
            'resolution': 'HD720',
            'grab_frame_rate': '30'
        }.items()
    )
    
    return LaunchDescription([
        declare_camera_model,
        zed_wrapper_launch
    ])
