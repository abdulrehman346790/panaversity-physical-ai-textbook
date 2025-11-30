# Quickstart Guide: Advanced Humanoid Control & AI Integration

**Date**: 2025-11-30  
**Feature**: Chapter 5 — Advanced Humanoid Control & AI Integration

## Prerequisites

Before starting this chapter, ensure you have:

1. **ROS 2 Humble Hawksbill** installed and configured
2. **Webots R2024b** installed with ROS 2 interface
3. **Basic ROS 2 knowledge** (topics, services, actions, tf2)
4. **Python 3.10+** for development
5. **Access to humanoid robot simulation** (ROBOTIS OP3 model recommended)

## Setup Environment

### 1. Clone the Repository
```bash
git clone [repository-url]
cd panaversity-physical-ai
```

### 2. Install ROS 2 Dependencies
```bash
# Navigate to workspace
cd ~/panaversity-physical-ai
# Install dependencies
rosdep install --from-paths src --ignore-src -r -y
# Build packages
colcon build --packages-select humanoid_control sensor_fusion ai_command_interface cognitive_planning
# Source the workspace
source install/setup.bash
```

### 3. Install Webots ROS 2 Interface
```bash
# Install Webots Python API
pip3 install webots-ros2
# Or from source if needed
pip3 install git+https://github.com/cyberbotics/webots_ros2
```

## Running the Examples

### 1. Basic Humanoid Motion Control
```bash
# Terminal 1: Launch Webots simulation with OP3 robot
ros2 launch humanoid_control op3_sim.launch.py

# Terminal 2: Run motion controller
ros2 run humanoid_control motion_controller
```

### 2. Multi-Sensor Fusion
```bash
# Launch sensor fusion demo
ros2 launch sensor_fusion multi_sensor_demo.launch.py
# Visualize in RViz
ros2 run rviz2 rviz2 -d src/sensor_fusion/config/sensor_fusion.rviz
```

### 3. AI Command Interface
```bash
# Terminal 1: Start the AI bridge
ros2 run ai_command_interface ai_bridge

# Terminal 2: Send commands via CLI
ros2 run ai_command_interface send_command "Move to the red cube"
```

### 4. Cognitive Planning Demo
```bash
# Launch planning demo
ros2 launch cognitive_planning navigation_demo.launch.py

# Send a high-level task
ros2 run cognitive_planning send_task "Navigate to the kitchen and find the blue mug"
```

## Key ROS 2 Packages

### humanoid_control
- **Purpose**: Basic motion control for humanoid robots
- **Key Nodes**: 
  - `motion_controller`: Processes high-level motion commands
  - `balance_controller`: Maintains robot stability using IMU feedback
- **Key Topics**:
  - `/joint_commands`: Joint positions for motion
  - `/imu/data`: IMU data for balance

### sensor_fusion
- **Purpose**: Integrates LiDAR, IMU, and camera data
- **Key Nodes**: 
  - `sensor_processor`: Fuses sensor data into coherent perception
- **Key Topics**:
  - `/fused_perception`: Unified sensor perception
  - `/camera/image_rect`: Rectified camera images
  - `/scan`: LiDAR scan data

### ai_command_interface
- **Purpose**: Bridges natural language to ROS 2 commands
- **Key Nodes**:
  - `command_bridge`: Translates AI commands to ROS 2 actions
  - `llm_interface`: Connects to LLM for natural language processing
- **Key Services**:
  - `/process_command`: Process natural language commands

### cognitive_planning
- **Purpose**: High-level task planning and execution
- **Key Nodes**:
  - `behavior_tree_executor`: Executes behavior trees
  - `task_planner`: Creates behavior trees from high-level goals
- **Key Actions**:
  - `/execute_task`: Execute high-level tasks

## Common Issues and Troubleshooting

### 1. Webots Simulation Not Starting
- Check if Webots is properly installed and licensed
- Verify ROS 2 Webots interface is installed
- Ensure proper environment variables are set

### 2. TF Transform Issues
- Check if robot_state_publisher is running
- Verify all coordinate frames are properly published
- Use `ros2 run tf2_tools view_frames` to visualize transforms

### 3. Sensor Data Not Synchronized
- Verify timestamp synchronization between sensors
- Check sensor message filters configuration
- Use `ros2 bag` to record and analyze sensor data timing

## Next Steps

After completing this quickstart:

1. Proceed to the detailed documentation for each module
2. Experiment with the examples provided
3. Modify parameters to understand system behavior
4. Integrate with physical hardware (if available)