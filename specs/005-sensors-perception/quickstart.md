# Quickstart: Chapter 4 — Sensors & Perception

**Date**: 2025-11-30

## Overview

This guide provides the essential steps to get the Chapter 4 examples up and running. It assumes you have a working ROS 2 Jazzy and Webots installation.

## Steps

### 1. Run the Simulation

First, launch one of the Webots simulation worlds. The simulation will automatically start publishing sensor data to ROS 2 topics.

**For the basic demo (LiDAR only):**
```bash
# Navigate to your project directory
webots webots/worlds/basic_sensor_demo.wbt
```

**For the advanced demo (Camera, LiDAR, IMU):**
```bash
# Navigate to your project directory
webots webots/worlds/advanced_sensor_demo.wbt
```

### 2. Visualize in RViz2

While the simulation is running, open RViz2 and load the provided configuration file to see all the sensor data visualized.

```bash
# In a new terminal, source your ROS 2 workspace
ros2 run rviz2 rviz2 -d rviz/04-sensors.rviz
```
You should see:
- A point cloud from the LiDAR.
- An image from the camera (in the advanced demo).
- A 3D marker representing the IMU's orientation.

### 3. Run the Python Subscribers

To see the raw sensor data being processed in code, run the Python subscriber nodes using the provided launch file.

```bash
# In a new terminal, source your ROS 2 workspace
ros2 launch examples/ros2/sensors/sensor_subscribers.launch.py
```
Your terminal will start printing messages received from the `/camera/image`, `/lidar`, and `/imu` topics.
