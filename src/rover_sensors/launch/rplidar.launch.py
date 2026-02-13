import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch RPLidar A1/A2 sensor
    Author: Sharan G S
    """
    
    serial_port = LaunchConfiguration('serial_port')
    frame_id = LaunchConfiguration('frame_id')
    
    declare_serial_port = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for RPLidar'
    )
    
    declare_frame_id = DeclareLaunchArgument(
        'frame_id',
        default_value='laser',
        description='Frame ID for laser scans'
    )
    
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar_node',
        parameters=[{
            'serial_port': serial_port,
            'frame_id': frame_id,
            'angle_compensate': True,
            'scan_mode': 'Standard'
        }],
        output='screen'
    )
    
    return LaunchDescription([
        declare_serial_port,
        declare_frame_id,
        rplidar_node
    ])
