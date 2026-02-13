# Quick Start Guide

Author: Sharan G S

## Installation (Quick)

```bash
# Install ROS2 Humble dependencies
sudo apt update
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup \
                 ros-humble-slam-toolbox ros-humble-robot-localization \
                 ros-humble-rplidar-ros

# Install Python dependencies
pip3 install -r requirements.txt

# Build workspace
cd rover_ws
colcon build --symlink-install
source install/setup.bash
```

## Running the System

### 1. SLAM Mode (Build Map)
```bash
ros2 launch rover_navigation bringup.launch.py slam_mode:=true
```

### 2. Navigation Mode (Autonomous)
```bash
ros2 launch rover_navigation bringup.launch.py slam_mode:=false
```

### 3. Visualization Only
```bash
ros2 launch rover_description display.launch.py
```

## Setting Navigation Goals

1. Open RViz (launched automatically)
2. Click "2D Pose Estimate" to set initial robot position
3. Click "Nav2 Goal" to set destination
4. Robot will autonomously navigate avoiding obstacles

## Saving Maps

```bash
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map
```

## Common Issues

**Lidar not detected:**
```bash
sudo chmod 666 /dev/ttyUSB0
```

**Camera not working:**
```bash
ZED_Diagnostic
```

For detailed documentation, see README.md
