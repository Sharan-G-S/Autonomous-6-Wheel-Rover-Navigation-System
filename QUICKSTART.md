# Quick Start Guide

Author: Sharan G S

## Hardware

- Jetson AGX 64GB
- RPLidar A1/A2
- ZED 2i Camera
- EMLID REACH M2 GPS
- 5x Sony cameras
- ESP32 microcontrollers
- 6-wheel rover chassis

## Installation (Quick)

```bash
# Install ROS2 Humble dependencies
sudo apt update
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup \
                 ros-humble-slam-toolbox ros-humble-robot-localization \
                 ros-humble-rplidar-ros ros-humble-geographic-msgs

# Install Python dependencies
pip3 install -r requirements.txt

# Build workspace
cd rover_ws
colcon build --symlink-install
source install/setup.bash
```

## GPS Waypoint Navigation (Primary Feature)

Navigate rover to GPS coordinates:

```bash
# Terminal 1: Launch complete system
ros2 launch rover_navigation bringup.launch.py

# Terminal 2: Navigate to GPS coordinates
ros2 launch rover_gps gps_navigation.launch.py \
  target_latitude:=37.7749 \
  target_longitude:=-122.4194
```

The rover will autonomously navigate to the specified GPS coordinates!

## Interactive GPS Waypoint Setter

```bash
ros2 run rover_gps gps_goal_converter.py
# Enter coordinates: 37.7749,-122.4194
```

## SLAM Mode (Build Map)

```bash
ros2 launch rover_navigation bringup.launch.py slam_mode:=true
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Navigation Mode (Map-based)

```bash
ros2 launch rover_navigation bringup.launch.py slam_mode:=false
# Set pose and goal in RViz
```

## Visualization Only

```bash
ros2 launch rover_description display.launch.py
```

## Saving Maps

```bash
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map
```

## Common Issues

**GPS not working:**
```bash
ros2 topic echo /emlid/fix
```

**Lidar not detected:**
```bash
sudo chmod 666 /dev/ttyUSB0
```

**Camera not working:**
```bash
ZED_Diagnostic
```

## Jetson AGX Performance Mode

```bash
sudo nvpmodel -m 0
sudo jetson_clocks
```

For detailed documentation, see README.md
