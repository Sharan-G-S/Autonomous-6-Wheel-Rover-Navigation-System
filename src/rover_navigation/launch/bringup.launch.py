import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    Complete system bringup for autonomous 6-wheel rover
    Author: Sharan G S
    
    Launches:
    - Robot description
    - Sensor drivers (RPLidar, ZED 2i)
    - Localization (EKF)
    - SLAM or Navigation (based on mode)
    - Perception (object detection)
    """
    
    # Package directories
    rover_description_dir = get_package_share_directory('rover_description')
    rover_localization_dir = get_package_share_directory('rover_localization')
    rover_slam_dir = get_package_share_directory('rover_slam')
    rover_perception_dir = get_package_share_directory('rover_perception')
    rover_navigation_dir = get_package_share_directory('rover_navigation')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_mode = LaunchConfiguration('slam_mode')
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )
    
    declare_slam_mode = DeclareLaunchArgument(
        'slam_mode',
        default_value='true',
        description='Whether to run SLAM (true) or localization with existing map (false)'
    )
    
    # Robot description
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rover_description_dir, 'launch', 'display.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    
    # Localization (EKF)
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rover_localization_dir, 'launch', 'localization.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    
    # SLAM
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rover_slam_dir, 'launch', 'slam.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    
    # Perception
    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rover_perception_dir, 'launch', 'perception.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    
    # Navigation
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rover_navigation_dir, 'launch', 'navigation.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        declare_slam_mode,
        robot_description_launch,
        localization_launch,
        slam_launch,
        perception_launch,
        # Uncomment when ready for autonomous navigation
        # navigation_launch
    ])
