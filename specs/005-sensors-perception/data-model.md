# Data Model: Chapter 4 — Sensors & Perception

**Date**: 2025-11-30

## Summary

This chapter does not introduce a new persistent data model. Instead, it focuses on the data structures of the real-time sensor data streams, which are defined by standard ROS 2 message types. The "data model" for this feature is therefore the set of ROS 2 messages that students will learn to subscribe to and interpret.

## Key Data Structures (ROS 2 Messages)

The following standard `sensor_msgs` are the primary data structures for this chapter.

### 1. `sensor_msgs/msg/Image`

- **Purpose**: Represents a 2D grayscale or color image.
- **Used for**: Camera data stream.
- **Key Fields**:
  - `header (std_msgs/Header)`: Timestamp and frame ID.
  - `height (uint32)`: Image height in pixels.
  - `width (uint32)`: Image width in pixels.
  - `encoding (string)`: Pixel encoding (e.g., `rgb8`, `mono8`).
  - `data (uint8[])`: Raw image data.
- **Reference**: [ROS 2 Documentation for `sensor_msgs/msg/Image`](https://docs.ros.org/en/jazzy/p/sensor_msgs/msg/Image.html)

### 2. `sensor_msgs/msg/LaserScan`

- **Purpose**: Represents data from a 1D or 2D laser scanner (LiDAR).
- **Used for**: LiDAR data stream.
- **Key Fields**:
  - `header (std_msgs/Header)`: Timestamp and frame ID.
  - `angle_min (float32)`: Start angle of the scan [rad].
  - `angle_max (float32)`: End angle of the scan [rad].
  - `angle_increment (float32)`: Angular distance between measurements [rad].
  - `ranges (float32[])`: Array of range measurements [m].
  - `intensities (float32[])`: Optional array of intensity measurements.
- **Reference**: [ROS 2 Documentation for `sensor_msgs/msg/LaserScan`](https://docs.ros.org/en/jazzy/p/sensor_msgs/msg/LaserScan.html)

### 3. `sensor_msgs/msg/Imu`

- **Purpose**: Represents data from an Inertial Measurement Unit (IMU).
- **Used for**: IMU data stream.
- **Key Fields**:
  - `header (std_msgs/Header)`: Timestamp and frame ID.
  - `orientation (geometry_msgs/Quaternion)`: Orientation estimate.
  - `angular_velocity (geometry_msgs/Vector3)`: Angular velocity [rad/s].
  - `linear_acceleration (geometry_msgs/Vector3)`: Linear acceleration [m/s^2].
- **Reference**: [ROS 2 Documentation for `sensor_msgs/msg/Imu`](https://docs.ros.org/en/jazzy/p/sensor_msgs/msg/Imu.html)
