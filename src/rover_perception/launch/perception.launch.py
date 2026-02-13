import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('rover_perception')
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    model_path = LaunchConfiguration('model_path')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )
    
    declare_model_path = DeclareLaunchArgument(
        'model_path',
        default_value='yolov8n.pt',
        description='Path to YOLO model file'
    )

    object_detector_node = Node(
        package='rover_perception',
        executable='object_detector.py',
        name='object_detector',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'model_path': model_path,
            'confidence_threshold': 0.5
        }]
    )
    
    obstacle_publisher_node = Node(
        package='rover_perception',
        executable='obstacle_publisher.py',
        name='obstacle_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_model_path,
        object_detector_node,
        obstacle_publisher_node
    ])
