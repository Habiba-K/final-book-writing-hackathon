---
title: Chapter 3 - Unity Robotics Integration
sidebar_position: 3
---

# Chapter 3: Unity Robotics Integration

## 1. Specification

This chapter covers Unity robotics integration, focusing on Unity robotics packages, how Unity receives robot transforms and sensor data, and basic visualization techniques with theoretical understanding.

### Learning Objectives

After completing this chapter, students will be able to:
- Explain Unity robotics packages and their purpose in robotics visualization
- Describe how Unity receives robot transforms and sensor data
- Understand the process of loading robot models in Unity
- Explain how to visualize live data in Unity with minimal complexity
- Identify common troubleshooting approaches for Unity integration issues

## 2. Core Concepts

### Unity Robotics Packages Overview

Unity provides specialized packages for robotics integration:

**Unity Robotics Hub**: Central hub for all robotics-related tools and samples, providing easy access to robotics-specific assets and examples.

**ROS-TCP-Connector**: Enables communication between Unity and ROS 2 via TCP/IP networking, allowing bidirectional data flow.

**Unity Perception Package**: Tools for generating synthetic training data for machine learning, including sensor simulation and domain randomization.

**XR Interaction Framework**: Extensions for immersive interaction with robotic systems in VR/AR environments.

### How Unity Receives Robot Transforms and Sensor Data

Unity connects to the robotics data stream through:

**Network Communication**: Unity applications communicate with ROS 2 nodes using TCP/IP sockets, establishing persistent connections for real-time data streaming.

**Transform Synchronization**: Robot joint positions and poses are continuously updated in Unity's coordinate system, ensuring visual representation matches physical or simulated robot state.

**Sensor Data Visualization**: Raw sensor data (LiDAR, camera feeds, IMU readings) is processed and rendered in Unity's 3D environment for intuitive understanding.

### Robot Model Loading Process

Loading robot models in Unity involves:

**Model Import**: 3D robot models can be imported in standard formats (FBX, OBJ) or converted from URDF descriptions using specialized tools.

**Rigging and Animation**: Robot joints are configured to match the physical robot's kinematic structure, enabling accurate pose representation.

**Coordinate System Alignment**: Unity's coordinate system is aligned with ROS conventions to ensure proper transform mapping.

## 3. Examples

### Example: Unity Visualization Pipeline

The Unity visualization pipeline follows this process:

1. **Data Reception**: Unity receives transform and sensor data from ROS 2 topics
2. **Transform Processing**: Raw transform data is processed and applied to robot model
3. **Scene Rendering**: Updated robot pose is rendered in Unity's 3D environment
4. **User Interaction**: Visualization can be manipulated through Unity's interface

### Example: Sensor Data Integration

Unity can visualize various sensor data types:

- **Camera Feeds**: Render textures display real or simulated camera data
- **LiDAR Point Clouds**: 3D point clouds visualize distance measurements
- **IMU Orientation**: Visual indicators show robot orientation and acceleration
- **Force/Torque Sensors**: Visual overlays display contact forces and torques

## 4. Steps

### Understanding Unity Robotics Integration

1. **Install Unity Robotics Packages**: Add ROS-TCP-Connector and other relevant packages to Unity project
2. **Configure Network Settings**: Set up IP addresses and ports for ROS 2 communication
3. **Import Robot Model**: Load 3D model and configure joints to match robot kinematics
4. **Map Transforms**: Establish correspondence between ROS transforms and Unity coordinates
5. **Test Connection**: Verify data flow between ROS 2 and Unity applications
6. **Visualize Data**: Implement visualization for sensor data and robot state

### Basic Unity Visualization Setup

1. **Create Unity Project**: Start new project with robotics packages
2. **Import Robot Model**: Add robot 3D model to scene
3. **Configure ROS Connection**: Set up TCP connection to ROS 2 network
4. **Map Joints**: Associate Unity transforms with ROS joint states
5. **Add Visualization**: Implement sensor data display elements
6. **Test Integration**: Verify robot movements are reflected in Unity

## 5. Code

### Example: Basic Unity-ROS Connection

```csharp
using UnityEngine;
using System.Collections;
using Unity.Robotics.ROSTCPConnector;

public class RobotController : MonoBehaviour
{
    // ROS TCP Connector instance
    ROSConnection ros;

    // Topic name for joint states
    string jointStatesTopic = "/joint_states";

    // Array to store joint names
    string[] jointNames;

    // Array to store joint positions
    float[] jointPositions;

    // Start is called before the first frame update
    void Start()
    {
        // Get the ROS connection static instance
        ros = ROSConnection.GetOrCreateInstance();

        // Subscribe to joint states topic
        ros.Subscribe<sensor_msgs.JointState>(jointStatesTopic, JointStateCallback);
    }

    // Callback function for joint state messages
    void JointStateCallback(sensor_msgs.JointState jointState)
    {
        // Update joint names if they haven't been set yet
        if (jointNames == null || jointNames.Length != jointState.name.Count)
        {
            jointNames = jointState.name.ToArray();
            jointPositions = new float[jointState.position.Count];
        }

        // Update joint positions
        for (int i = 0; i < Mathf.Min(jointPositions.Length, jointState.position.Count); i++)
        {
            jointPositions[i] = (float)jointState.position[i];

            // Apply joint position to corresponding Unity transform
            ApplyJointPosition(i, jointPositions[i]);
        }
    }

    // Apply joint position to Unity transform
    void ApplyJointPosition(int jointIndex, float position)
    {
        // This is a simplified example - in practice, you would map
        // joint names to specific transforms in your robot model
        if (jointIndex < transform.childCount)
        {
            Transform jointTransform = transform.GetChild(jointIndex);
            // Apply rotation or position based on joint type
            jointTransform.localRotation = Quaternion.Euler(0, position * Mathf.Rad2Deg, 0);
        }
    }

    // Update is called once per frame
    void Update()
    {
        // Continuously update robot visualization based on received data
    }
}
```

### Example: Sensor Data Visualization

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine.Rendering;

public class SensorVisualizer : MonoBehaviour
{
    // Reference to ROS connection
    ROSConnection ros;

    // Topic for LiDAR data
    string lidarTopic = "/scan";

    // Point cloud renderer
    PointCloudRenderer pointCloudRenderer;

    // Start method to initialize connection
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();

        // Subscribe to LiDAR data
        ros.Subscribe<sensor_msgs.LaserScan>(lidarTopic, LidarCallback);

        // Initialize point cloud renderer
        InitializePointCloud();
    }

    // Callback for LiDAR data
    void LidarCallback(sensor_msgs.LaserScan scan)
    {
        // Process LiDAR ranges into point cloud
        Vector3[] points = ProcessLidarToPoints(scan);

        // Update point cloud visualization
        UpdatePointCloud(points);
    }

    // Convert LiDAR data to 3D points
    Vector3[] ProcessLidarToPoints(sensor_msgs.LaserScan scan)
    {
        // Calculate angle increment
        float angleIncrement = (float)scan.angle_increment;
        float currentAngle = (float)scan.angle_min;

        // Create array for points
        System.Collections.Generic.List<Vector3> points =
            new System.Collections.Generic.List<Vector3>();

        // Process each range measurement
        for (int i = 0; i < scan.ranges.Count; i++)
        {
            float range = (float)scan.ranges[i];

            // Skip invalid ranges
            if (range >= scan.range_min && range <= scan.range_max)
            {
                // Calculate 3D position from polar coordinates
                float x = range * Mathf.Cos(currentAngle);
                float y = range * Mathf.Sin(currentAngle);

                points.Add(new Vector3(x, 0, y));
            }

            currentAngle += angleIncrement;
        }

        return points.ToArray();
    }

    // Update point cloud visualization
    void UpdatePointCloud(Vector3[] points)
    {
        // In a real implementation, this would update the point cloud renderer
        // with the new points for visualization
    }

    // Initialize point cloud renderer
    void InitializePointCloud()
    {
        // Setup would include creating materials, mesh filters, etc.
    }
}
```

## 6. Final Exercise

1. Explain the role of Unity Robotics Hub in the robotics development workflow
2. Describe how Unity connects to ROS 2 and receives robot transform data
3. Outline the steps for importing and configuring a robot model in Unity
4. Identify three different types of sensor data that can be visualized in Unity
5. List common troubleshooting steps for Unity-ROS connection issues

---

**Previous**: [Chapter 2: Gazebo Simulation Basics](../02-gazebo-simulation-basics/index.md)
**Next**: [Chapter 4: ROS–Gazebo–Unity Bridge](../04-ros-gazebo-unity-bridge/index.md)