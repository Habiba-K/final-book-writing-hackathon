---
title: Chapter 2 - Gazebo Simulation Basics
sidebar_position: 2
---

# Chapter 2: Gazebo Simulation Basics

## 1. Specification

This chapter introduces students to Gazebo simulation basics, focusing on supported models, sensors, and how to launch worlds with text-only commands and theoretical understanding.

### Learning Objectives

After completing this chapter, students will be able to:
- Explain what Gazebo is and its role in robotics simulation
- Identify supported models and sensors in Gazebo
- Describe how to import URDF/SDF in Gazebo
- Understand how Gazebo connects to ROS 2 topics
- Perform basic Gazebo operations through text-only commands

## 2. Core Concepts

### What is Gazebo?

Gazebo is a physics-based robot simulator that provides realistic simulation of robot systems in virtual environments. It offers:

- **Physics Simulation**: Realistic modeling of gravity, collisions, and dynamics
- **Sensor Simulation**: Virtual sensors like cameras, LiDAR, IMU, and force/torque sensors
- **Model Library**: Extensive collection of robot models and environments
- **ROS Integration**: Seamless connection with ROS 2 for control and communication

### Supported Models and Sensors

#### Robot Models
Gazebo supports various robot models through:
- **URDF (Unified Robot Description Format)**: XML format for describing robot structure
- **SDF (Simulation Description Format)**: Native Gazebo format with more features
- **Pre-built Models**: From the Gazebo Model Database

#### Sensor Types
Common sensors simulated in Gazebo:
- **Cameras**: RGB, depth, and stereo vision sensors
- **LiDAR**: 2D and 3D laser scanners
- **IMU**: Inertial measurement units
- **Force/Torque Sensors**: Joint and contact force measurements
- **GPS**: Position and velocity sensors

### URDF/SDF Import Process

#### URDF in Gazebo
URDF files describe robot structure and can be imported into Gazebo with:
- Joint definitions and limits
- Link properties and inertial parameters
- Visual and collision geometries
- Transmission elements for actuator control

#### SDF Advantages
SDF provides additional features beyond URDF:
- Multi-robot scenarios in single worlds
- Advanced physics properties
- Lighting and rendering options
- Plugin support for custom behaviors

## 3. Examples

### Example: Loading a Robot Model

To load a robot model in Gazebo:

1. **Prepare the URDF/SDF file** with proper joint and link definitions
2. **Spawn the model** using Gazebo's model spawning service
3. **Verify the model** appears correctly in the simulation environment

### Example: Connecting to ROS 2

Gazebo connects to ROS 2 through:
- **gazebo_ros_pkgs**: Bridge packages that connect Gazebo to ROS 2
- **Topic Interface**: Publish/subscribe to sensor and control topics
- **Service Interface**: Spawn/delete models and control simulation

## 4. Steps

### Basic Gazebo Workflow

1. **Launch Gazebo** with a specific world file
2. **Load Robot Model** using URDF/SDF description
3. **Connect to ROS 2** topics for control and sensing
4. **Run Simulation** and interact with the virtual robot
5. **Monitor Sensor Data** from the simulated sensors

### Text-Only Commands

Basic commands to work with Gazebo:

```bash
# Launch Gazebo with empty world
ros2 launch gazebo_ros empty_world.launch.py

# List available models
gz model list

# Spawn a model from the database
ros2 run gazebo_ros spawn_entity.py -entity my_robot -database my_robot_model

# Check ROS 2 topics connected to Gazebo
ros2 topic list | grep gazebo
```

## 5. Code

### Example: Basic Gazebo Launch

```xml
<!-- Example world file: my_world.world -->
<sdf version="1.7">
  <world name="default">
    <!-- Include a model from the database -->
    <include>
      <uri>model://r1_rover</uri>
      <pose>0 0 0.5 0 0 0</pose>
    </include>

    <!-- Define a custom light -->
    <light name="sun" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.6 0.4 -0.8</direction>
    </light>

    <!-- Plugins for ROS 2 integration -->
    <plugin name="gazebo_ros_init" filename="libgazebo_ros_init.so">
      <ros>
        <namespace>/demo</namespace>
      </ros>
    </plugin>
  </world>
</sdf>
```

### Example: ROS 2 Connection

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu, JointState
from geometry_msgs.msg import Twist

class GazeboRobotController(Node):
    """
    Example controller that connects to a robot in Gazebo simulation.
    """

    def __init__(self):
        super().__init__('gazebo_robot_controller')

        # Subscribe to simulated sensor data
        self.laser_sub = self.create_subscription(
            LaserScan, '/laser_scan', self.laser_callback, 10
        )

        self.imu_sub = self.create_subscription(
            Imu, '/imu', self.imu_callback, 10
        )

        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )

        # Publish control commands
        self.cmd_vel_pub = self.create_publisher(
            Twist, '/cmd_vel', 10
        )

        self.get_logger().info('Gazebo Robot Controller initialized')

    def laser_callback(self, msg):
        """Process simulated laser scan data."""
        self.get_logger().info(f'Laser range: {msg.ranges[0]}')

    def imu_callback(self, msg):
        """Process simulated IMU data."""
        self.get_logger().info(f'IMU orientation: {msg.orientation.x}')

    def joint_state_callback(self, msg):
        """Process simulated joint states."""
        if msg.position:
            self.get_logger().info(f'Joint position: {msg.position[0]}')

def main(args=None):
    rclpy.init(args=args)
    controller = GazeboRobotController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 6. Final Exercise

1. Research the differences between URDF and SDF formats
2. Explain how to launch a custom world in Gazebo using text commands
3. Describe the process of connecting a simulated robot to ROS 2 topics
4. Identify three common sensor types available in Gazebo and their applications
5. Outline the steps to troubleshoot a Gazebo simulation that fails to start

---

**Previous**: [Chapter 1: Digital Twin Overview](../01-digital-twin-overview/index.md)
**Next**: [Chapter 3: Unity Robotics Integration](../03-unity-robotics-integration/index.md)