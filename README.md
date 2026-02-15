# Autonomous 6-Wheel Rover Navigation System with GPS

Sharan G S

## Overview

This project implements a complete autonomous navigation system for a 6-wheel rover using ROS2 Humble and the Nav2 stack. The system features GPS-based waypoint navigation, multi-sensor fusion with Extended Kalman Filter (EKF), SLAM capabilities, and object detection for dynamic obstacle awareness.

**Key Capability**: Navigate autonomously to GPS coordinates (latitude, longitude) provided by the user.

## System Architecture

### Hardware Components

- **Onboard Computer**: NVIDIA Jetson AGX 64GB
- **Lidar**: RPLidar A1/A2 (360-degree 2D laser scanner)
- **Primary Camera**: ZED 2i Stereo Camera (RGB, depth, and IMU)
- **Multi-angle Cameras**: 5x Sony lens cameras (front-left, front-right, rear, left, right)
- **GPS**: EMLID REACH M2 (RTK-capable GPS receiver)
- **Microcontrollers**: ESP32 (motor control, sensor interfaces)
- **Chassis**: 6-wheel differential drive system
- **Wheel Encoders**: For odometry measurement

### Software Stack

- ROS2 Humble
- Nav2 Navigation Stack
- SLAM Toolbox (2D SLAM)
- robot_localization (EKF sensor fusion with GPS)
- YOLOv8 (COCO dataset object detection)
- ZED SDK and ROS2 wrapper
- EMLID REACH M2 GPS driver

### Key Features

- **GPS Waypoint Navigation**: Navigate to GPS coordinates (lat/lon)
- Multi-sensor fusion using Extended Kalman Filter (wheel odometry + IMU + lidar + GPS)
- Real-time SLAM with loop closure detection
- Autonomous path planning and navigation
- Dynamic obstacle detection and avoidance with YOLOv8
- Object recognition using COCO dataset (80 classes)
- Multi-angle camera coverage for 360-degree perception
- Recovery behaviors for stuck situations
- Costmap-based collision avoidance

## ROS2 Packages

### 1. rover_description
Complete URDF/Xacro model of the 6-wheel rover including all sensors.

**Hardware modeled:**
- Jetson AGX 64GB housing
- 6 wheels (front, middle, rear pairs)
- RPLidar A1/A2
- ZED 2i camera
- 5 Sony cameras at multiple angles
- EMLID REACH M2 GPS
- IMU from ZED 2i

**Key files:**
- `urdf/rover.urdf.xacro`: Complete robot description
- `launch/display.launch.py`: Visualization in RViz

### 2. rover_sensors
Sensor driver configurations and launch files.

**Sensors:**
- RPLidar driver
- ZED 2i camera driver
- Combined sensor launch

**Key files:**
- `launch/rplidar.launch.py`: RPLidar driver
- `launch/zed_camera.launch.py`: ZED 2i driver
- `launch/sensors.launch.py`: All sensors

### 3. rover_localization
EKF-based sensor fusion for accurate pose estimation.

**Sensor inputs:**
- Wheel odometry (velocity)
- IMU from ZED 2i (orientation, angular velocity, acceleration)
- Lidar odometry from SLAM (position, orientation)
- **GPS from EMLID REACH M2 (global position)**

**Key files:**
- `config/ekf.yaml`: EKF configuration with GPS fusion
- `config/navsat_transform.yaml`: GPS to odometry conversion
- `launch/localization.launch.py`: Localization node

### 4. rover_slam
SLAM Toolbox integration for map building and localization.

**Features:**
- Synchronous/asynchronous SLAM modes
- Loop closure detection
- Map serialization and loading

**Key files:**
- `config/slam_toolbox_params.yaml`: SLAM parameters
- `launch/slam.launch.py`: SLAM node
- `maps/`: Saved map files

### 5. rover_perception
YOLOv8-based object detection using ZED 2i camera.

**Features:**
- Real-time object detection (COCO dataset - 80 classes)
- Depth integration for 3D obstacle localization
- Publishes detection markers for visualization
- Optimized for Jetson AGX GPU acceleration

**Key files:**
- `scripts/object_detector.py`: YOLOv8 detection node
- `scripts/obstacle_publisher.py`: Costmap integration
- `launch/perception.launch.py`: Perception pipeline

### 6. rover_navigation
Nav2 stack configuration for autonomous navigation.

**Features:**
- Global path planning (NavFn planner)
- Local trajectory planning (DWB Controller)
- Dynamic obstacle avoidance
- Recovery behaviors (spin, backup, wait)
- Layered costmaps (static, obstacle, inflation, voxel)

**Key files:**
- `config/nav2_params.yaml`: Complete Nav2 configuration
- `launch/navigation.launch.py`: Nav2 stack
- `launch/bringup.launch.py`: Complete system launch

### 7. rover_gps (NEW)
GPS-based waypoint navigation for autonomous locomotion.

**Features:**
- Accept GPS coordinates (latitude, longitude)
- Convert GPS to map frame coordinates
- Send navigation goals to Nav2
- Monitor progress to GPS waypoint

**Key files:**
- `scripts/gps_waypoint_navigator.py`: GPS navigation node
- `scripts/gps_goal_converter.py`: Interactive waypoint setter
- `launch/gps_navigation.launch.py`: GPS navigation launch
- `config/gps_config.yaml`: EMLID REACH M2 configuration

## Installation

### Prerequisites

Ubuntu 22.04 with ROS2 Humble installed on Jetson AGX 64GB.

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

# Install robot_localization (with GPS support)
sudo apt install ros-humble-robot-localization

# Install geographic packages for GPS
sudo apt install ros-humble-geographic-msgs

# Install RPLidar driver
sudo apt install ros-humble-rplidar-ros

# Install ZED SDK and ROS2 wrapper for Jetson
# Follow instructions at: https://www.stereolabs.com/developers/release

# Install additional dependencies
sudo apt install ros-humble-joint-state-publisher \
                 ros-humble-robot-state-publisher \
                 ros-humble-xacro \
                 ros-humble-cv-bridge

# Python dependencies (optimized for Jetson)
pip3 install ultralytics opencv-python numpy
```

### Build Workspace

```bash
# Navigate to the workspace
cd ~/Downloads/Path_Planning_Algorithm/rover_ws

# Build all packages
colcon build --symlink-install

# Source the workspace
source install/setup.bash

# Add to bashrc for convenience
echo "source ~/Downloads/Path_Planning_Algorithm/rover_ws/install/setup.bash" >> ~/.bashrc
```

## Hardware Setup

### 1. Jetson AGX 64GB
- Install JetPack 5.x or later
- Enable all CUDA cores for GPU acceleration
- Configure power mode to MAXN for maximum performance

### 2. RPLidar Connection
- Connect RPLidar to USB port on Jetson
- Verify device: `ls /dev/ttyUSB*`
- Grant permissions: `sudo chmod 666 /dev/ttyUSB0`

### 3. ZED 2i Camera
- Connect ZED 2i via USB 3.0 to Jetson
- Install ZED SDK for Jetson from Stereolabs website
- Verify installation: `ZED_Explorer`

### 4. EMLID REACH M2 GPS
- Connect EMLID REACH M2 to Jetson via USB or UART
- Configure EMLID for NMEA output or use EMLID ROS driver
- Verify GPS fix: Check `/emlid/fix` topic
- For RTK mode, configure base station connection

### 5. Sony Cameras
- Connect Sony cameras via USB or CSI interfaces
- Configure camera drivers for multi-camera support
- Verify camera feeds

### 6. ESP32 Microcontrollers
- Flash ESP32 with motor control firmware
- Connect via serial/UART to Jetson
- Configure for wheel encoder reading and motor commands

### 7. Wheel Encoders
- Connect encoder interfaces to ESP32
- Configure in robot description URDF
- Publish odometry on `/odom` topic

## Usage

### 1. GPS Waypoint Navigation (Primary Use Case)

Navigate the rover to a specific GPS coordinate:

```bash
# Terminal 1: Launch complete system with GPS
ros2 launch rover_navigation bringup.launch.py

# Terminal 2: Navigate to GPS coordinates
ros2 launch rover_gps gps_navigation.launch.py target_latitude:=37.7749 target_longitude:=-122.4194

# The rover will autonomously navigate to the specified GPS coordinates
```

**Interactive GPS waypoint setter:**
```bash
ros2 run rover_gps gps_goal_converter.py
# Then enter GPS coordinates when prompted: 37.7749,-122.4194
```

### 2. SLAM Mode (Mapping)

Build a map of the environment:

```bash
# Terminal 1: Launch sensors and SLAM
ros2 launch rover_navigation bringup.launch.py slam_mode:=true

# Terminal 2: Teleoperate to build map
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Terminal 3: Save map when complete
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map
```

### 3. Autonomous Navigation Mode (Map-based)

Navigate using a pre-built map:

```bash
# Launch complete system with navigation
ros2 launch rover_navigation bringup.launch.py slam_mode:=false

# In RViz:
# 1. Click "2D Pose Estimate" to set initial pose
# 2. Click "Nav2 Goal" to set navigation goal
# 3. Robot will autonomously navigate to goal
```

### 4. Visualization Only (No Hardware)

```bash
# Launch robot description in RViz
ros2 launch rover_description display.launch.py
```

## Configuration

### GPS Configuration
Edit `rover_gps/config/gps_config.yaml`:
- Set EMLID REACH M2 connection parameters
- Configure datum (reference point) for your operating area
- Adjust GPS topic names

### EKF Tuning with GPS
Edit `rover_localization/config/ekf.yaml`:
- GPS odometry input configuration
- Process noise covariance for GPS
- Sensor fusion weights

### Navigation Tuning
Edit `rover_navigation/config/nav2_params.yaml`:
- Velocity limits (optimized for 6-wheel drive)
- Acceleration limits
- Costmap parameters
- Planner behavior

### Object Detection (Jetson Optimization)
Edit `rover_perception/launch/perception.launch.py`:
- Use YOLOv8n for best performance on Jetson AGX
- Adjust confidence threshold
- Enable TensorRT acceleration

## Topics

### Published Topics
- `/scan` - Lidar scans
- `/zed/zed_node/rgb/image_rect_color` - RGB image
- `/zed/zed_node/depth/depth_registered` - Depth image
- `/zed/zed_node/imu/data` - IMU data
- `/emlid/fix` - GPS fix (EMLID REACH M2)
- `/odom` - Wheel odometry
- `/odometry/filtered` - EKF fused odometry (with GPS)
- `/gps/odometry` - GPS converted to odometry
- `/map` - Occupancy grid map
- `/detections/image` - Annotated detection image
- `/detections/markers` - Detection markers
- `/gps_goal` - Current GPS navigation goal

### Subscribed Topics
- `/cmd_vel` - Velocity commands

## GPS Navigation Workflow

1. **System Startup**: Launch complete system with all sensors
2. **GPS Fix**: Wait for GPS to acquire fix (check `/emlid/fix` topic)
3. **Datum Set**: First GPS fix sets the datum (reference point)
4. **Provide Waypoint**: Give GPS coordinates (lat, lon)
5. **Coordinate Conversion**: GPS coordinates converted to map frame
6. **Path Planning**: Nav2 plans path to converted goal
7. **Autonomous Navigation**: Rover navigates avoiding obstacles
8. **Arrival**: Rover reaches GPS waypoint within tolerance

## Troubleshooting

### GPS Issues
```bash
# Check GPS fix
ros2 topic echo /emlid/fix

# Verify GPS is publishing
ros2 topic hz /emlid/fix

# Check EMLID connection
# Access EMLID web interface at emlid-reach.local
```

### RPLidar not detected
```bash
sudo chmod 666 /dev/ttyUSB0
ls -l /dev/ttyUSB*
```

### ZED camera not working on Jetson
```bash
# Check ZED SDK installation
ZED_Diagnostic

# Verify USB 3.0 connection
lsusb | grep ZED
```

### EKF not fusing GPS
- Check GPS topic is publishing
- Verify navsat_transform_node is running
- Check `/gps/odometry` topic
- Verify frame IDs match

### Navigation not working
- Ensure map is loaded
- Set initial pose in RViz
- Check GPS has valid fix
- Verify localization is working

## Performance Optimization for Jetson AGX 64GB

### GPU Acceleration
- Enable CUDA for YOLOv8 inference
- Use TensorRT for optimized inference
- Allocate sufficient GPU memory

### Power Management
```bash
# Set to maximum performance mode
sudo nvpmodel -m 0
sudo jetson_clocks
```

### Camera Resolution
- Use HD720 for ZED 2i (balance between quality and performance)
- Ardu camera resolution if needed

### Navigation Tuning
- Adjust planning frequency based on CPU load
- Optimize costmap update rates

## Repository Structure

```
rover_ws/
├── src/
│   ├── rover_description/      # Robot URDF model (Jetson AGX, all sensors)
│   ├── rover_sensors/          # Sensor drivers (RPLidar, ZED 2i)
│   ├── rover_localization/     # EKF sensor fusion (with GPS)
│   ├── rover_slam/             # SLAM Toolbox config
│   ├── rover_perception/       # YOLOv8 object detection
│   ├── rover_navigation/       # Nav2 configuration
│   └── rover_gps/              # GPS waypoint navigation
└── README.md
```

## License

MIT License

## Author

# Made with 💚 from Sharan G S

