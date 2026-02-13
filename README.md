# Autonomous 6-Wheel Rover Navigation System

Author: Sharan G S

## Overview

This project implements a complete autonomous navigation system for a 6-wheel rover using ROS2 Humble and the Nav2 stack. The system integrates multiple sensors with Extended Kalman Filter (EKF) based sensor fusion for robust localization, SLAM capabilities for map building, and object detection for dynamic obstacle awareness.

## System Architecture

### Hardware Components

- 6-wheel differential drive rover chassis
- RPLidar A1/A2 (360-degree 2D laser scanner)
- ZED 2i Stereo Camera (RGB, depth, and IMU)
- Wheel encoders (odometry)
- Onboard computer (Jetson Nano/Xavier or equivalent)

### Software Stack

- ROS2 Humble
- Nav2 Navigation Stack
- SLAM Toolbox (2D SLAM)
- robot_localization (EKF sensor fusion)
- YOLOv8 (COCO dataset object detection)
- ZED SDK and ROS2 wrapper

### Key Features

- Multi-sensor fusion using Extended Kalman Filter (EKF)
- Real-time SLAM with loop closure detection
- Autonomous path planning and navigation
- Dynamic obstacle detection and avoidance
- Object recognition using COCO dataset (80 classes)
- Recovery behaviors for stuck situations
- Costmap-based collision avoidance

## ROS2 Packages

### 1. rover_description
Complete URDF/Xacro model of the 6-wheel rover including all sensors.

**Key files:**
- `urdf/rover.urdf.xacro`: Robot description
- `launch/display.launch.py`: Visualization in RViz

### 2. rover_localization
EKF-based sensor fusion for accurate pose estimation.

**Sensor inputs:**
- Wheel odometry (velocity)
- IMU from ZED 2i (orientation, angular velocity, acceleration)
- Lidar odometry from SLAM (position, orientation)

**Key files:**
- `config/ekf.yaml`: EKF configuration
- `launch/localization.launch.py`: Localization node

### 3. rover_slam
SLAM Toolbox integration for map building and localization.

**Features:**
- Synchronous/asynchronous SLAM modes
- Loop closure detection
- Map serialization and loading

**Key files:**
- `config/slam_toolbox_params.yaml`: SLAM parameters
- `launch/slam.launch.py`: SLAM node
- `maps/`: Saved map files

### 4. rover_perception
YOLOv8-based object detection using ZED 2i camera.

**Features:**
- Real-time object detection (COCO dataset - 80 classes)
- Depth integration for 3D obstacle localization
- Publishes detection markers for visualization

**Key files:**
- `scripts/object_detector.py`: YOLOv8 detection node
- `scripts/obstacle_publisher.py`: Costmap integration
- `launch/perception.launch.py`: Perception pipeline

### 5. rover_navigation
Nav2 stack configuration for autonomous navigation.

**Features:**
- Global path planning (NavFn/Smac Planner)
- Local trajectory planning (DWB Controller)
- Dynamic obstacle avoidance
- Recovery behaviors (spin, backup, wait)
- Layered costmaps (static, obstacle, inflation, voxel)

**Key files:**
- `config/nav2_params.yaml`: Complete Nav2 configuration
- `launch/navigation.launch.py`: Nav2 stack
- `launch/bringup.launch.py`: Complete system launch

## Installation

### Prerequisites

Ubuntu 22.04 with ROS2 Humble installed.

```bash
# Install ROS2 Humble (if not already installed)
sudo apt update
sudo apt install ros-humble-desktop
```

### Dependencies

```bash
# Install Nav2
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup

# Install SLAM Toolbox
sudo apt install ros-humble-slam-toolbox

# Install robot_localization
sudo apt install ros-humble-robot-localization

# Install RPLidar driver
sudo apt install ros-humble-rplidar-ros

# Install ZED SDK and ROS2 wrapper
# Follow instructions at: https://www.stereolabs.com/developers/release

# Install additional dependencies
sudo apt install ros-humble-joint-state-publisher \
                 ros-humble-robot-state-publisher \
                 ros-humble-xacro \
                 ros-humble-cv-bridge

# Python dependencies
pip3 install ultralytics opencv-python numpy
```

### Build Workspace

```bash
# Clone or navigate to the workspace
cd ~/Downloads/Path_Planning_Algorithm/rover_ws

# Build all packages
colcon build --symlink-install

# Source the workspace
source install/setup.bash

# Add to bashrc for convenience
echo "source ~/Downloads/Path_Planning_Algorithm/rover_ws/install/setup.bash" >> ~/.bashrc
```

## Hardware Setup

### 1. RPLidar Connection
- Connect RPLidar to USB port
- Verify device: `ls /dev/ttyUSB*`
- Grant permissions: `sudo chmod 666 /dev/ttyUSB0`

### 2. ZED 2i Camera
- Connect ZED 2i via USB 3.0
- Install ZED SDK from Stereolabs website
- Verify installation: `ZED_Explorer`

### 3. Wheel Encoders
- Connect encoder interfaces to GPIO/serial ports
- Configure in robot description URDF
- Publish odometry on `/odom` topic

## Usage

### 1. Visualization Only (No Hardware)

```bash
# Launch robot description in RViz
ros2 launch rover_description display.launch.py
```

### 2. SLAM Mode (Mapping)

```bash
# Terminal 1: Launch sensors and SLAM
ros2 launch rover_navigation bringup.launch.py slam_mode:=true

# Terminal 2: Teleoperate to build map
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Terminal 3: Save map when complete
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map
```

### 3. Autonomous Navigation Mode

```bash
# Launch complete system with navigation
ros2 launch rover_navigation bringup.launch.py slam_mode:=false

# In RViz:
# 1. Click "2D Pose Estimate" to set initial pose
# 2. Click "Nav2 Goal" to set navigation goal
# 3. Robot will autonomously navigate to goal
```

### 4. Individual Components

```bash
# Launch only localization
ros2 launch rover_localization localization.launch.py

# Launch only SLAM
ros2 launch rover_slam slam.launch.py

# Launch only perception
ros2 launch rover_perception perception.launch.py

# Launch only navigation
ros2 launch rover_navigation navigation.launch.py
```

## Configuration

### EKF Tuning
Edit `rover_localization/config/ekf.yaml` to adjust:
- Sensor input topics
- Process noise covariance
- Sensor configurations

### SLAM Parameters
Edit `rover_slam/config/slam_toolbox_params.yaml` to adjust:
- Map resolution
- Loop closure parameters
- Scan matching settings

### Navigation Tuning
Edit `rover_navigation/config/nav2_params.yaml` to adjust:
- Velocity limits
- Acceleration limits
- Costmap parameters
- Planner behavior
- Recovery behaviors

### Object Detection
Edit `rover_perception/launch/perception.launch.py` to adjust:
- YOLO model (yolov8n/s/m/l/x)
- Confidence threshold
- Detection classes

## Topics

### Published Topics
- `/scan` - Lidar scans (sensor_msgs/LaserScan)
- `/zed/zed_node/rgb/image_rect_color` - RGB image
- `/zed/zed_node/depth/depth_registered` - Depth image
- `/zed/zed_node/imu/data` - IMU data
- `/odom` - Wheel odometry
- `/odometry/filtered` - EKF fused odometry
- `/map` - Occupancy grid map
- `/detections/image` - Annotated detection image
- `/detections/markers` - Detection markers

### Subscribed Topics
- `/cmd_vel` - Velocity commands (geometry_msgs/Twist)

## Troubleshooting

### RPLidar not detected
```bash
sudo chmod 666 /dev/ttyUSB0
ls -l /dev/ttyUSB*
```

### ZED camera not working
```bash
# Check ZED SDK installation
ZED_Diagnostic
# Reinstall ZED ROS2 wrapper if needed
```

### EKF not publishing
- Check all sensor topics are publishing
- Verify frame IDs match in URDF and sensor configs
- Check TF tree: `ros2 run tf2_tools view_frames`

### Navigation not working
- Ensure map is loaded
- Set initial pose in RViz
- Check costmaps are being published
- Verify localization is working

### Object detection slow
- Use smaller YOLO model (yolov8n)
- Reduce image resolution
- Lower detection frequency

## Performance Optimization

### For Jetson Nano/Xavier
- Use YOLOv8n (nano) model
- Reduce camera resolution to 720p
- Lower navigation update rates
- Enable GPU acceleration for YOLO

### For More Powerful Hardware
- Use YOLOv8m/l for better accuracy
- Increase costmap resolution
- Higher planning frequencies

## Development

### Adding New Sensors
1. Update URDF in `rover_description/urdf/rover.urdf.xacro`
2. Add sensor driver launch file
3. Update EKF config if sensor provides odometry/IMU data
4. Update costmap config if sensor provides obstacle data

### Custom Planners
1. Create plugin in `rover_navigation`
2. Update `nav2_params.yaml` to use custom planner
3. Rebuild workspace

## Repository Structure

```
rover_ws/
├── src/
│   ├── rover_description/      # Robot URDF model
│   ├── rover_localization/     # EKF sensor fusion
│   ├── rover_slam/             # SLAM Toolbox config
│   ├── rover_perception/       # Object detection
│   └── rover_navigation/       # Nav2 configuration
└── README.md
```

## License

MIT License

## Author

Sharan G S

## References

- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [Nav2 Documentation](https://navigation.ros.org/)
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [robot_localization](http://docs.ros.org/en/noetic/api/robot_localization/html/index.html)
- [YOLOv8](https://github.com/ultralytics/ultralytics)
- [ZED SDK](https://www.stereolabs.com/developers/)

## Contributing

Contributions are welcome. Please open an issue or submit a pull request.

## Acknowledgments

- ROS2 and Nav2 communities
- SLAM Toolbox by Steve Macenski
- Ultralytics for YOLOv8
- Stereolabs for ZED SDK
