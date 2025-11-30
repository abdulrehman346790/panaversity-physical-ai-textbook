# Data Model: Advanced Humanoid Control & AI Integration

**Date**: 2025-11-30  
**Feature**: Chapter 5 — Advanced Humanoid Control & AI Integration

## Key Entities and Data Structures

### 1. Humanoid Robot Control Data

#### Joint States
- **Type**: sensor_msgs/JointState
- **Purpose**: Represents current positions, velocities, and efforts of robot joints
- **Key Fields**: 
  - position[]: joint positions (radians)
  - velocity[]: joint velocities (rad/s) 
  - effort[]: joint efforts (N⋅m)

#### Robot Pose
- **Type**: geometry_msgs/PoseStamped
- **Purpose**: Robot's position and orientation in space
- **Key Fields**:
  - pose.position: x, y, z coordinates
  - pose.orientation: quaternion (x, y, z, w)

### 2. Sensor Fusion Data

#### LiDAR Data
- **Type**: sensor_msgs/LaserScan
- **Purpose**: Environment mapping and obstacle detection
- **Key Fields**:
  - ranges[]: distance measurements
  - angle_min, angle_max: angular range
  - time_increment: time between measurements

#### IMU Data
- **Type**: sensor_msgs/Imu
- **Purpose**: Balance and orientation information
- **Key Fields**:
  - orientation: quaternion representing orientation
  - angular_velocity: angular velocity vector
  - linear_acceleration: linear acceleration vector

#### Camera Data
- **Type**: sensor_msgs/Image
- **Purpose**: Visual information for perception
- **Key Fields**:
  - height, width: image dimensions
  - encoding: pixel format
  - data[]: image data

### 3. AI Command Interface Data

#### High-Level Commands
- **Type**: Custom message humanoid_control/HighLevelCommand
- **Purpose**: AI-generated commands for robot execution
- **Key Fields**:
  - command_type: enum (NAVIGATE, GRASP, FOLLOW_PATH, etc.)
  - target_position: optional geometry_msgs/Point
  - parameters: string map of command-specific parameters

#### Command Response
- **Type**: Custom message humanoid_control/CommandResponse
- **Purpose**: Feedback on command execution status
- **Key Fields**:
  - status: enum (SUCCESS, FAILURE, IN_PROGRESS)
  - message: human-readable status message
  - execution_time: time taken to execute

### 4. Cognitive Planning Data

#### Planning Tasks
- **Type**: Custom message cognitive_planning/PlanningTask
- **Purpose**: Structured tasks for cognitive planner
- **Key Fields**:
  - task_id: unique identifier
  - goal_description: natural language goal
  - constraints: list of execution constraints
  - priority: task priority level

#### Behavior Tree Nodes
- **Type**: Custom message cognitive_planning/BehaviorTreeNode
- **Purpose**: Components of behavior tree for decision making
- **Key Fields**:
  - node_id: unique identifier
  - node_type: enum (ACTION, CONDITION, DECORATOR, etc.)
  - parameters: configuration parameters
  - children: list of child node IDs

## Coordinate Frames (TF2)

### Robot Coordinate Frames
- **base_link**: Robot's main body coordinate frame
- **odom**: Odometry reference frame (world-fixed)
- **map**: Global map coordinate frame
- **sensor frames**: Individual frames for each sensor (laser_link, camera_link, imu_link)
- **end_effector**: Tool center point for manipulation tasks

## Message Flow

### Sensor Pipeline
```
LiDAR (laser_scan) → robot_state_publisher → base_link
Camera (image_raw) → image_proc → rectified images
IMU (imu_data) → robot_localization → pose estimate
```

### AI Command Flow
```
Natural language → LLM → HighLevelCommand → CommandBridge → ROS 2 Actions
```

### Planning Flow
```
Task goal → Cognitive Planner → Behavior Tree → Action Execution → Feedback
```