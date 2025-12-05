---
sidebar_position: 3
---

# AI Integration: LLM-Guided Robot Commands

In this section, we'll explore how to connect Large Language Models (LLMs) with our ROS 2-based humanoid robot for natural language command processing. This integration allows robots to understand and execute high-level tasks described in human language.

## Introduction to AI-Robot Integration

AI integration bridges the gap between human-friendly language and robot-executable commands. Instead of programming specific movement sequences, users can instruct robots using natural language: "Navigate to the blue cube" or "Move to the kitchen and find the red mug."

## AI Command Architecture

Our AI integration system follows a command bridge pattern:

1. **Natural Language Input**: User provides high-level commands in natural language
2. **LLM Processing**: LLM interprets the command and structures it
3. **Command Bridge**: Translates structured commands to ROS 2 messages
4. **ROS 2 Execution**: Commands are executed by the robot's control system
5. **Feedback Loop**: Execution results are reported back to the user

## Implementation Approach

We'll implement this using a hybrid approach combining local models for accessibility with optional cloud integration for advanced features.

### Component Architecture

The AI command system consists of:

1. **AI Interface Node**: Connects to LLMs (local or cloud-based)
2. **Command Bridge**: Translates high-level commands to ROS 2 actions
3. **Command Executor**: Executes ROS 2 commands based on AI input
4. **Feedback System**: Provides execution status to the AI interface

## Example Implementation

Let's walk through the core implementation:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from humanoid_control_msgs.msg import HighLevelCommand, CommandResponse
import json
import requests
import subprocess

class AICommandBridge(Node):
    def __init__(self):
        super().__init__('ai_command_bridge')
        
        # Publisher for high-level commands
        self.command_pub = self.create_publisher(
            HighLevelCommand,
            '/ai_commands',
            10
        )
        
        # Publisher for responses
        self.response_pub = self.create_publisher(
            CommandResponse,
            '/ai_command_responses',
            10
        )
        
        # Subscriber for natural language commands
        self.command_sub = self.create_subscription(
            String,
            '/natural_language_commands',
            self.command_callback,
            10
        )
        
        # Use a local LLM (Ollama in this example) for processing
        self.ollama_endpoint = "http://localhost:11434/api/generate"
        
        self.get_logger().info("AI Command Bridge initialized")
    
    def command_callback(self, msg):
        """Process natural language command and convert to ROS 2 action"""
        try:
            # Process the natural language command
            structured_command = self.process_natural_language(msg.data)
            
            # Create and publish high-level command
            command_msg = HighLevelCommand()
            command_msg.command_type = structured_command['command_type']
            command_msg.target_pose = self.create_pose_from_data(
                structured_command.get('target', {})
            )
            command_msg.parameters = json.dumps(structured_command.get('parameters', {}))
            
            self.command_pub.publish(command_msg)
            
            # Send initial response
            response_msg = CommandResponse()
            response_msg.status = "IN_PROGRESS"
            response_msg.message = f"Executing: {msg.data}"
            self.response_pub.publish(response_msg)
            
        except Exception as e:
            self.get_logger().error(f"Error processing command: {str(e)}")
            response_msg = CommandResponse()
            response_msg.status = "FAILURE"
            response_msg.message = f"Error: {str(e)}"
            self.response_pub.publish(response_msg)
    
    def process_natural_language(self, command_text):
        """Process natural language command using LLM"""
        # Define the prompt for the LLM to structure the command
        prompt = f"""
        You are a command interpreter for a humanoid robot. Parse the following natural language command and return a structured JSON object.
        
        Command: "{command_text}"
        
        Return a JSON with these fields:
        - command_type: NAVIGATE, GRASP, FOLLOW_PATH, etc.
        - target: (optional) x, y coordinates or object description
        - parameters: (optional) additional parameters for the command
        
        Respond with only the JSON object, nothing else:
        """
        
        try:
            # Call local Ollama instance
            response = requests.post(
                self.ollama_endpoint,
                json={
                    "model": "llama3.2",
                    "prompt": prompt,
                    "stream": False
                }
            )
            
            # Extract JSON from response
            result = response.json()
            text = result['response']
            
            # Extract JSON from the response text (it might have extra text)
            start_idx = text.find('{')
            end_idx = text.rfind('}') + 1
            json_str = text[start_idx:end_idx]
            
            return json.loads(json_str)
        except Exception as e:
            self.get_logger().error(f"LLM processing error: {e}")
            # Fallback command structure
            return {
                "command_type": "UNKNOWN",
                "parameters": {"raw_command": command_text}
            }
    
    def create_pose_from_data(self, target_data):
        """Create a PoseStamped from target data"""
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        
        if 'x' in target_data and 'y' in target_data:
            pose.pose.position.x = float(target_data['x'])
            pose.pose.position.y = float(target_data['y'])
            pose.pose.position.z = 0.0
            # Simple orientation (facing target)
            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 1.0
        
        return pose

def main(args=None):
    rclpy.init(args=args)
    ai_bridge = AICommandBridge()
    
    try:
        rclpy.spin(ai_bridge)
    except KeyboardInterrupt:
        pass
    finally:
        ai_bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Running the AI Command Interface

### Prerequisites

1. Install and run a local LLM (e.g., Ollama with Llama 3.2 model):
   ```bash
   # Install Ollama (https://ollama.ai)
   curl -fsSL https://ollama.ai/install.sh | sh
   
   # Pull the Llama 3.2 model
   ollama pull llama3.2
   ```

2. Start the Ollama server:
   ```bash
   ollama serve
   ```

### Launching the Interface

```bash
# Terminal 1: Launch Webots simulation
ros2 launch humanoid_control op3_sim.launch.py

# Terminal 2: Start the AI command bridge
ros2 run ai_command_interface ai_bridge

# Terminal 3: Send natural language commands
ros2 topic pub /natural_language_commands std_msgs/String "data: 'Move to position (1, 2)'"

# Or run a command-line interface
ros2 run ai_command_interface command_line_interface
```

## Command Line Interface

We also provide a command-line interface for easier testing:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
import sys

class AICommandCLI(Node):
    def __init__(self):
        super().__init__('ai_command_cli')
        self.publisher = self.create_publisher(
            String,
            '/natural_language_commands',
            10
        )
        
        self.get_logger().info("AI Command CLI ready. Type commands and press Enter.")
        self.get_logger().info("Type 'quit' to exit.")
    
    def run(self):
        """Run the CLI interface"""
        while rclpy.ok():
            try:
                command = input("> ")
                if command.lower() == 'quit':
                    break
                
                # Publish the command
                msg = String()
                msg.data = command
                self.publisher.publish(msg)
                
            except KeyboardInterrupt:
                break
            except Exception as e:
                self.get_logger().error(f"Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    cli = AICommandCLI()
    
    # Run in a separate thread to allow ROS spinning
    cli_thread = threading.Thread(target=cli.run)
    cli_thread.start()
    
    try:
        rclpy.spin(cli)
    except KeyboardInterrupt:
        pass
    finally:
        cli_thread.join()
        cli.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Advanced: Cloud LLM Integration

For more advanced capabilities, you can integrate with cloud-based LLMs. Here's an example with OpenAI:

```python
import openai

class CloudAIBridge(Node):
    def __init__(self):
        super().__init__('cloud_ai_bridge')
        # Set your API key in an environment variable
        openai.api_key = os.getenv('OPENAI_API_KEY')
        
        # Rest of implementation similar to local LLM version
        # but using openai.ChatCompletion.create() for processing
```

## Error Handling and Validation

The AI command system includes several layers of validation:

1. **Command Validation**: Ensures the LLM output follows expected format
2. **Safety Checks**: Validates commands for potential safety issues
3. **Feedback Loop**: Informs the LLM of execution results

```python
def validate_command(self, command):
    """Validate command before execution"""
    # Check if command type is recognized
    valid_types = ['NAVIGATE', 'GRASP', 'FOLLOW_PATH', 'SPEAK', 'UNKNOWN']
    if command.command_type not in valid_types:
        raise ValueError(f"Unknown command type: {command.command_type}")
    
    # Check safety constraints
    if command.command_type == 'NAVIGATE':
        # Verify target location is valid
        if not self.is_navigable(command.target_pose):
            raise ValueError("Target location is not navigable")
```

## Practical Exercise

Create an enhanced AI command interface that can handle multi-step tasks:

1. Implement a simple task queue to handle multiple commands in sequence
2. Add a feedback mechanism that reports command completion to the LLM
3. Extend the command structure to support more complex operations
4. Create a simple command history for debugging purposes

## Troubleshooting

### LLM Connection Issues
- Verify the Ollama server is running: `curl http://localhost:11434`
- Check that the required model is downloaded
- Ensure no network/firewall issues

### Command Interpretation Errors
- The LLM might not understand the command format
- Try using simple, clear language
- Check the prompt engineering in the `process_natural_language` method

## Summary

AI integration enables natural interaction with humanoid robots through language. Our command bridge pattern provides a flexible interface between high-level language commands and low-level ROS 2 actions. This approach makes robotics more accessible and demonstrates the integration of AI with physical systems.

In the next section, we'll explore cognitive planning, which enables robots to reason about complex multi-step tasks.