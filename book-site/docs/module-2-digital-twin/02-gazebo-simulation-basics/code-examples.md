---
title: Annotated Code Examples for Gazebo Integration
sidebar_position: 7
---

# Annotated Code Examples: Gazebo Integration (If Applicable)

## 1. Basic Gazebo Launch Configuration

### Example: World File with ROS Integration

```xml
<?xml version="1.0"?>
<!-- Example world file that includes ROS 2 integration -->
<sdf version="1.7">
  <world name="simple_world">
    <!-- Include a basic model from the database -->
    <include>
      <uri>model://r1_rover</uri>
      <pose>0 0 0.5 0 0 0</pose>
    </include>

    <!-- Define lighting -->
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

    <!-- Gazebo ROS initialization plugin -->
    <plugin name="gazebo_ros_init" filename="libgazebo_ros_init.so">
      <ros>
        <!-- Define namespace for this simulation -->
        <namespace>/demo</namespace>
      </ros>
      <!-- Set simulation time to be used -->
      <update_rate>1000</update_rate>
    </plugin>

    <!-- Physics engine configuration -->
    <physics name="1ms" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
  </world>
</sdf>
```

**Annotations:**
- `<include>` tag imports pre-built models from Gazebo's model database
- `<plugin>` tag connects Gazebo to ROS 2 using the gazebo_ros plugins
- `<namespace>` sets up a separate namespace for this simulation's topics
- Physics parameters control simulation accuracy and real-time performance

## 2. URDF Robot Model with Gazebo Plugins

### Example: Robot Model with ROS Integration

```xml
<?xml version="1.0"?>
<!-- Robot model with Gazebo plugins for ROS integration -->
<robot name="diff_drive_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Base link definition -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.3 0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 0.8"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.3 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.2" iyz="0.0" izz="0.3"/>
    </inertial>
  </link>

  <!-- Left wheel -->
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="0.0 0.2 0.0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <link name="left_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.005"/>
    </inertial>
  </link>

  <!-- Right wheel -->
  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <origin xyz="0.0 -0.2 0.0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <link name="right_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.005"/>
    </inertial>
  </link>

  <!-- Gazebo plugin for differential drive -->
  <gazebo>
    <!-- Differential drive controller -->
    <plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so">
      <!-- Wheel information -->
      <left_joint>left_wheel_joint</left_joint>
      <right_joint>right_wheel_joint</right_joint>

      <!-- Kinematic model properties -->
      <wheel_separation>0.4</wheel_separation>
      <wheel_diameter>0.2</wheel_diameter>

      <!-- Topic names for ROS communication -->
      <command_topic>cmd_vel</command_topic>
      <odometry_topic>odom</odometry_topic>
      <odometry_frame>odom</odometry_frame>
      <robot_base_frame>base_link</robot_base_frame>

      <!-- Publishing rates -->
      <publish_odom>true</publish_odom>
      <publish_odom_tf>true</publish_odom_tf>
      <publish_wheel_tf>true</publish_wheel_tf>
    </plugin>

    <!-- IMU sensor plugin -->
    <plugin name="imu_sensor" filename="libgazebo_ros_imu.so">
      <ros>
        <!-- Topic to publish IMU data to -->
        <remapping>~/out:=imu</remapping>
      </ros>
      <frame_id>base_link</frame_id>
      <!-- Update rate for sensor -->
      <update_rate>100</update_rate>
    </plugin>
  </gazebo>

</robot>
```

**Annotations:**
- `gazebo` tags define simulation-specific configurations
- `libgazebo_ros_diff_drive.so` plugin connects differential drive control to ROS
- Topic names define how ROS nodes communicate with the simulation
- Frame IDs ensure proper coordinate transformations
- Update rates control sensor data frequency

## 3. Python Code for Interfacing with Gazebo Simulation

### Example: Robot Controller for Gazebo Simulation

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
from tf2_ros import TransformBroadcaster
import math

class GazeboRobotController(Node):
    """
    Example controller that interfaces with a robot in Gazebo simulation.
    This class demonstrates how to connect ROS 2 nodes to Gazebo simulation.
    """

    def __init__(self):
        super().__init__('gazebo_robot_controller')

        # Create publisher for velocity commands to control the simulated robot
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',  # Topic name must match the one in the URDF/Gazebo plugin
            10
        )

        # Create subscribers to receive sensor data from the simulation
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',  # Topic where Gazebo publishes odometry data
            self.odom_callback,
            10
        )

        self.imu_subscriber = self.create_subscription(
            Imu,
            '/imu',   # Topic where Gazebo publishes IMU data
            self.imu_callback,
            10
        )

        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',  # Topic where Gazebo publishes LiDAR data
            self.scan_callback,
            10
        )

        # Timer to periodically send commands to the robot
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Initialize variables to store robot state
        self.robot_pose_x = 0.0
        self.robot_pose_y = 0.0
        self.robot_heading = 0.0

        self.get_logger().info('Gazebo Robot Controller initialized')

    def odom_callback(self, msg):
        """
        Callback function to process odometry data from Gazebo simulation.

        Args:
            msg (Odometry): Odometry message containing robot position and velocity
        """
        # Extract position from odometry message
        self.robot_pose_x = msg.pose.pose.position.x
        self.robot_pose_y = msg.pose.pose.position.y

        # Convert quaternion orientation to Euler angles to get heading
        orientation_q = msg.pose.pose.orientation
        siny_cosp = 2 * (orientation_q.w * orientation_q.z +
                         orientation_q.x * orientation_q.y)
        cosy_cosp = 1 - 2 * (orientation_q.y * orientation_q.y +
                             orientation_q.z * orientation_q.z)
        self.robot_heading = math.atan2(siny_cosp, cosy_cosp)

        # Log current position
        self.get_logger().info(
            f'Robot position: ({self.robot_pose_x:.2f}, {self.robot_pose_y:.2f}), '
            f'heading: {math.degrees(self.robot_heading):.2f}°'
        )

    def imu_callback(self, msg):
        """
        Callback function to process IMU data from Gazebo simulation.

        Args:
            msg (Imu): IMU message containing orientation, angular velocity, and linear acceleration
        """
        # Extract orientation data from IMU message
        orientation = msg.orientation
        self.get_logger().info(
            f'IMU orientation - x: {orientation.x:.3f}, '
            f'y: {orientation.y:.3f}, z: {orientation.z:.3f}, w: {orientation.w:.3f}'
        )

    def scan_callback(self, msg):
        """
        Callback function to process LiDAR scan data from Gazebo simulation.

        Args:
            msg (LaserScan): LiDAR scan message containing distance measurements
        """
        # Find minimum distance in the scan
        if len(msg.ranges) > 0:
            min_distance = min([r for r in msg.ranges if r != float('inf')])
            self.get_logger().info(f'Minimum distance to obstacle: {min_distance:.2f}m')

    def timer_callback(self):
        """
        Periodic callback that sends velocity commands to the simulated robot.
        This function runs at the rate defined by timer_period.
        """
        # Create a twist message to control the robot
        cmd_msg = Twist()

        # Set linear and angular velocities (these will be sent to Gazebo)
        cmd_msg.linear = Vector3(x=0.5, y=0.0, z=0.0)   # Move forward at 0.5 m/s
        cmd_msg.angular = Vector3(x=0.0, y=0.0, z=0.2)  # Rotate at 0.2 rad/s

        # Publish the command to the simulation
        self.cmd_vel_publisher.publish(cmd_msg)

        # Log the command being sent
        self.get_logger().info(
            f'Sent command - linear: {cmd_msg.linear.x:.2f} m/s, '
            f'angular: {cmd_msg.angular.z:.2f} rad/s'
        )

def main(args=None):
    """
    Main function to initialize and run the Gazebo robot controller.

    This function:
    1. Initializes the ROS 2 context
    2. Creates the controller node
    3. Spins the node to process callbacks
    4. Handles cleanup when shutting down
    """
    rclpy.init(args=args)

    controller = GazeboRobotController()

    try:
        # Spin the node to continuously process callbacks
        rclpy.spin(controller)
    except KeyboardInterrupt:
        # Handle Ctrl+C gracefully
        pass
    finally:
        # Cleanup resources
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Annotations:**
- Topic names must match those configured in Gazebo plugins
- Callback functions process data from simulated sensors
- Timer creates periodic behavior for sending commands
- Vector3 messages control robot movement in simulation
- The code structure matches the Robot → ROS → Simulator pattern

## 4. Launch File for Gazebo Simulation

### Example: Launch File to Start Simulation

```python
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """
    Generate launch description for Gazebo simulation with robot controller.

    This launch file combines:
    1. Gazebo world simulation
    2. Robot model spawning
    3. Robot controller node
    """

    # Declare launch arguments that can be passed to the launch file
    world_name_arg = DeclareLaunchArgument(
        'world_name',
        default_value='empty_world',
        description='Choose one of the world files from `/gazebo_ros/empty_worlds`'
    )

    # Get the package share directory for gazebo_ros
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')

    # Include the Gazebo server and client launch files
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': LaunchConfiguration('world_name'),
            'verbose': 'true'
        }.items()
    )

    # Define the robot controller node
    robot_controller = Node(
        package='your_robot_package',
        executable='gazebo_robot_controller',
        name='gazebo_robot_controller',
        output='screen',
        parameters=[
            {'use_sim_time': True}  # Use simulation time from Gazebo
        ]
    )

    # Return the complete launch description
    return LaunchDescription([
        world_name_arg,
        gazebo,
        robot_controller
    ])
```

**Annotations:**
- Launch arguments allow customization of the simulation
- `use_sim_time: True` makes nodes use Gazebo's simulation clock
- The launch file orchestrates the entire simulation setup
- Parameters can be passed to customize behavior

These code examples demonstrate the core concepts of connecting ROS 2 to Gazebo simulation, showing how to configure models, control robots, and process sensor data in the simulation environment.