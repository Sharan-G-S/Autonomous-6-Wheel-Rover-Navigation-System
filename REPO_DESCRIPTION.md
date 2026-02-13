# Repository Description for GitHub

Autonomous 6-Wheel Rover with GPS Waypoint Navigation, Multi-Sensor Fusion, and Object Detection

## Short Description

Complete ROS2-based autonomous navigation system for a 6-wheel rover with GPS waypoint navigation. Features multi-sensor fusion (RPLidar, ZED 2i, EMLID REACH M2 GPS, wheel encoders), SLAM mapping, EKF localization, YOLOv8 object detection, and Nav2 path planning. Navigate to GPS coordinates autonomously.

## Key Features

- GPS-based autonomous navigation (provide lat/lon, rover navigates there)
- Multi-sensor fusion using Extended Kalman Filter (wheel odometry + IMU + lidar + GPS)
- Real-time 2D SLAM with loop closure detection
- Autonomous path planning and obstacle avoidance using Nav2 stack
- YOLOv8-based object detection with COCO dataset (80 classes)
- Multi-angle camera coverage (5 Sony cameras + ZED 2i)
- Dynamic obstacle avoidance with layered costmaps
- Recovery behaviors for autonomous operation
- Optimized for NVIDIA Jetson AGX 64GB

## Hardware

- NVIDIA Jetson AGX 64GB (onboard computer)
- 6-wheel differential drive rover
- RPLidar A1/A2 (360-degree laser scanner)
- ZED 2i Stereo Camera (RGB + Depth + IMU)
- EMLID REACH M2 GPS (RTK-capable positioning)
- 5x Sony lens cameras (multi-angle perception)
- ESP32 microcontrollers (motor control, sensors)
- Wheel encoders for odometry

## Software Stack

- ROS2 Humble
- Nav2 Navigation Stack
- SLAM Toolbox
- robot_localization (EKF with GPS fusion)
- YOLOv8 (Ultralytics)
- ZED SDK
- EMLID GPS driver

## Usage Example

```bash
# Navigate to GPS coordinates
ros2 launch rover_gps gps_navigation.launch.py \
  target_latitude:=37.7749 \
  target_longitude:=-122.4194
```

## Topics

robotics, ros2, autonomous-navigation, gps-navigation, waypoint-navigation, slam, path-planning, obstacle-avoidance, sensor-fusion, ekf, lidar, stereo-camera, object-detection, yolo, nav2, mobile-robot, 6-wheel-rover, jetson-agx, emlid-reach, gps-rtk

## Author

Sharan G S
