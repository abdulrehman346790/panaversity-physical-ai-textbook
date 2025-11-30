# ROS 2 Topic Contracts: Chapter 4 — Sensors & Perception

**Date**: 2025-11-30

## Summary

The "API contracts" for this feature are the ROS 2 topics that will be published by the Webots simulation and subscribed to by the Python examples. These topics and their associated message types define the interface between the sensor simulation and the student's code.

## Topic Definitions

### 1. Camera Topic

- **Topic Name**: `/camera/image`
- **Message Type**: `sensor_msgs/msg/Image`
- **Description**: Publishes color images from the simulated camera on the robot.
- **Publisher**: Webots ROS 2 Bridge
- **Subscribers**: `camera_subscriber.py`, RViz2

### 2. LiDAR Topic

- **Topic Name**: `/lidar`
- **Message Type**: `sensor_msgs/msg/LaserScan`
- **Description**: Publishes 2D laser scan data from the simulated LiDAR.
- **Publisher**: Webots ROS 2 Bridge
- **Subscribers**: `lidar_subscriber.py`, RViz2

### 3. IMU Topic

- **Topic Name**: `/imu`
- **Message Type**: `sensor_msgs/msg/Imu`
- **Description**: Publishes inertial measurement data (orientation, angular velocity, linear acceleration) from the simulated IMU.
- **Publisher**: Webots ROS 2 Bridge
- **Subscribers**: `imu_subscriber.py`, RViz2
