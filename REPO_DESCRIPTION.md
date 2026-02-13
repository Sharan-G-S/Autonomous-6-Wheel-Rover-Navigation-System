# Repository Description for GitHub

Autonomous 6-Wheel Rover Navigation System with SLAM, Path Planning, and Object Detection

## Short Description

Complete ROS2-based autonomous navigation system for a 6-wheel rover featuring multi-sensor fusion (RPLidar, ZED 2i camera, wheel encoders), SLAM mapping, EKF localization, YOLOv8 object detection, and Nav2 path planning for obstacle-rich environments.

## Key Features

- Multi-sensor fusion using Extended Kalman Filter (EKF)
- Real-time 2D SLAM with loop closure detection
- Autonomous path planning and navigation using Nav2 stack
- YOLOv8-based object detection with COCO dataset (80 classes)
- Dynamic obstacle avoidance with layered costmaps
- Recovery behaviors for autonomous operation
- Complete ROS2 Humble implementation

## Hardware

- 6-wheel differential drive rover
- RPLidar A1/A2 (360-degree laser scanner)
- ZED 2i Stereo Camera (RGB + Depth + IMU)
- Wheel encoders for odometry
- Jetson Nano/Xavier or equivalent onboard computer

## Software Stack

- ROS2 Humble
- Nav2 Navigation Stack
- SLAM Toolbox
- robot_localization (EKF)
- YOLOv8 (Ultralytics)
- ZED SDK

## Topics

robotics, ros2, autonomous-navigation, slam, path-planning, obstacle-avoidance, sensor-fusion, ekf, lidar, stereo-camera, object-detection, yolo, nav2, mobile-robot, 6-wheel-rover

## Author

Sharan G S
