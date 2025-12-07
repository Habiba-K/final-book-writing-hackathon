---
title: Standard Code Example Formatting
sidebar_position: 10
---

# Standard Code Example Formatting for Python/ROS 2 Content

This document establishes the standard formatting guidelines for Python and ROS 2 code examples in the Digital Twin module, ensuring consistency and clarity for students.

## Python Code Formatting Standards

### 1. Basic Structure
```python
#!/usr/bin/env python3
# Copyright [Year] [Copyright Holder]
# License: [Appropriate License]

"""
Module docstring: Brief description of the module's purpose.
"""

import os
import sys
# Standard library imports
# Third-party imports
import rclpy
from rclpy.node import Node
# ROS 2 message imports
from std_msgs.msg import String
from sensor_msgs.msg import JointState
```

### 2. Function and Class Documentation
```python
class ExampleNode(Node):
    """
    ExampleNode - A sample ROS 2 node for educational purposes.

    This node demonstrates proper class structure and documentation
    for educational ROS 2 examples.
    """

    def __init__(self):
        """Initialize the ExampleNode with default parameters."""
        super().__init__('example_node')

        # Create a publisher
        self.publisher = self.create_publisher(String, 'topic_name', 10)

        # Create a timer
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info('ExampleNode initialized')

    def timer_callback(self):
        """Callback function for the timer."""
        msg = String()
        msg.data = 'Hello, Digital Twin!'
        self.publisher.publish(msg)
        self.get_logger().info(f'Published: {msg.data}')
```

### 3. Comment Standards
- **Inline Comments**: Use sparingly, only for complex logic
- **Block Comments**: Use for explaining multi-line operations
- **Documentation Strings**: Always include for classes and functions

```python
def example_function(param1, param2):
    """
    Brief description of what the function does.

    Args:
        param1 (str): Description of the first parameter
        param2 (int): Description of the second parameter

    Returns:
        bool: Description of the return value

    Example:
        >>> result = example_function("test", 42)
        >>> print(result)
        True
    """
    # This is an inline comment explaining complex logic
    result = complex_operation(param1, param2)

    # Another inline comment
    return result > 0
```

## ROS 2 Specific Formatting

### 1. Node Structure
```python
def main(args=None):
    """Main function to initialize and run the ROS 2 node."""
    rclpy.init(args=args)

    example_node = ExampleNode()

    try:
        rclpy.spin(example_node)
    except KeyboardInterrupt:
        pass
    finally:
        example_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 2. Message Handling
```python
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')

        # Publisher for velocity commands
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Subscriber for sensor data
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

    def scan_callback(self, msg):
        """
        Process incoming laser scan data.

        Args:
            msg (LaserScan): Laser scan message containing distance data
        """
        # Process the scan data
        min_distance = min(msg.ranges)
        self.get_logger().info(f'Minimum distance: {min_distance}')
```

### 3. Parameter Handling
```python
class ParameterExampleNode(Node):
    def __init__(self):
        super().__init__('parameter_example_node')

        # Declare parameters with default values
        self.declare_parameter('robot_name', 'my_robot')
        self.declare_parameter('max_velocity', 0.5)
        self.declare_parameter('safety_distance', 1.0)

        # Access parameters
        self.robot_name = self.get_parameter('robot_name').value
        self.max_velocity = self.get_parameter('max_velocity').value
        self.safety_distance = self.get_parameter('safety_distance').value

        self.get_logger().info(
            f'Initialized with robot_name: {self.robot_name}'
        )
```

## Code Example Annotation Guidelines

### 1. Educational Annotations
```python
def publish_robot_state(self, position, orientation):
    # Create a new message object
    msg = JointState()

    # Set the timestamp to current time
    msg.header.stamp = self.get_clock().now().to_msg()

    # Set the frame ID for coordinate reference
    msg.header.frame_id = "base_link"

    # Populate joint positions (this is where robot state is set)
    msg.position = position

    # Populate joint orientations
    msg.orientation = orientation

    # Publish the message to the robot state topic
    self.robot_state_publisher.publish(msg)
```

### 2. Error Handling Examples
```python
def safe_move_robot(self, target_position):
    try:
        # Validate input parameters
        if not self._validate_position(target_position):
            self.get_logger().error('Invalid target position')
            return False

        # Attempt to move the robot
        success = self._execute_move(target_position)

        if success:
            self.get_logger().info('Robot moved successfully')
            return True
        else:
            self.get_logger().warn('Robot move failed')
            return False

    except Exception as e:
        # Log any unexpected errors
        self.get_logger().error(f'Move robot failed: {str(e)}')
        return False
```

## Command Line Examples

### 1. Terminal Commands
```bash
# Launch a ROS 2 package
ros2 launch package_name launch_file.py

# Run a ROS 2 node directly
ros2 run package_name node_name

# List active topics
ros2 topic list

# Echo messages on a specific topic
ros2 topic echo /topic_name std_msgs/msg/String
```

### 2. Package Management
```bash
# Create a new ROS 2 package
ros2 pkg create --build-type ament_python package_name

# Build the workspace
colcon build --packages-select package_name

# Source the workspace
source install/setup.bash
```

## Visual Elements

### 1. Diagram Code (using ASCII or code comments)
```python
"""
Digital Twin Architecture:
┌─────────────┐    ┌──────────┐    ┌─────────────┐
│   Physical  │───▶│   ROS    │───▶│  Simulation │
│    Robot    │    │  Layer   │    │   Layer     │
└─────────────┘    └──────────┘    └─────────────┘
                        │
                        ▼
                  ┌─────────────┐
                  │ Visualization│
                  │    Layer    │
                  └─────────────┘
"""
```

### 2. Configuration Examples
```yaml
# Example configuration file format
robot_config:
  name: "digital_twin_robot"
  joints:
    - joint1
    - joint2
    - joint3
  sensors:
    - camera_front
    - lidar_3d
    - imu
  control_frequency: 50  # Hz
```

## Best Practices Summary

1. **Consistency**: Use the same formatting style throughout all examples
2. **Clarity**: Include educational comments that explain key concepts
3. **Completeness**: Provide complete, runnable examples when possible
4. **Safety**: Include appropriate error handling in complex examples
5. **Documentation**: Always include docstrings for public functions
6. **Standards**: Follow ROS 2 and Python community standards

---

**Next**: [Troubleshooting Guide Template](./troubleshooting-template.md)