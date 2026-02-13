import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch GPS waypoint navigation for autonomous rover
    Author: Sharan G S
    
    Usage:
    ros2 launch rover_gps gps_navigation.launch.py target_latitude:=37.7749 target_longitude:=-122.4194
    """
    
    target_latitude = LaunchConfiguration('target_latitude')
    target_longitude = LaunchConfiguration('target_longitude')
    gps_topic = LaunchConfiguration('gps_topic')
    
    declare_target_lat = DeclareLaunchArgument(
        'target_latitude',
        default_value='0.0',
        description='Target GPS latitude'
    )
    
    declare_target_lon = DeclareLaunchArgument(
        'target_longitude',
        default_value='0.0',
        description='Target GPS longitude'
    )
    
    declare_gps_topic = DeclareLaunchArgument(
        'gps_topic',
        default_value='/emlid/fix',
        description='GPS fix topic from EMLID REACH M2'
    )
    
    gps_navigator_node = Node(
        package='rover_gps',
        executable='gps_waypoint_navigator.py',
        name='gps_waypoint_navigator',
        output='screen',
        parameters=[{
            'target_latitude': target_latitude,
            'target_longitude': target_longitude,
            'gps_topic': gps_topic,
            'use_manual_datum': False
        }]
    )
    
    return LaunchDescription([
        declare_target_lat,
        declare_target_lon,
        declare_gps_topic,
        gps_navigator_node
    ])
