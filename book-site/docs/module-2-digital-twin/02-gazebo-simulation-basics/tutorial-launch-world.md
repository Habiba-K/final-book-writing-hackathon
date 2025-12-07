---
title: Tutorial - Launching a World in Gazebo
sidebar_position: 3
---

# Tutorial: Launching a World in Gazebo

## Overview

This short tutorial demonstrates how to launch a world in Gazebo using text-only commands. This is a fundamental operation in Gazebo simulation that allows you to set up a virtual environment for your robot.

## Prerequisites

- ROS 2 Humble/Iron installed
- Gazebo ROS packages installed
- Basic understanding of ROS 2 command-line tools

## Steps to Launch a World

### 1. Launch Empty World

To launch Gazebo with an empty world:

```bash
# Launch Gazebo with default empty world
ros2 launch gazebo_ros empty_world.launch.py
```

### 2. Launch with Custom World File

To launch Gazebo with a custom world file:

```bash
# Launch Gazebo with a specific world file
ros2 launch gazebo_ros empty_world.launch.py world_name:=path/to/your/world.sdf
```

### 3. Launch with Specific Parameters

To customize the launch with specific parameters:

```bash
# Launch with verbose output and GUI disabled
ros2 launch gazebo_ros empty_world.launch.py verbose:=true gui_required:=false
```

### 4. Spawn a Robot Model

Once Gazebo is running, you can spawn a robot model:

```bash
# Spawn a model from the Gazebo model database
ros2 run gazebo_ros spawn_entity.py -entity my_robot -database robot_name

# Spawn a model from a local URDF file
ros2 run gazebo_ros spawn_entity.py -entity my_robot -file path/to/robot.urdf
```

## Verifying the Launch

After launching, verify that everything is working:

```bash
# Check that Gazebo-related topics are available
ros2 topic list | grep gazebo

# Check the spawned models
gz model list

# Check the simulation status
ros2 service call /pause_physics std_srvs/srv/Empty
```

## Troubleshooting Common Issues

### Issue: Gazebo fails to launch
**Solution**: Ensure Gazebo is properly installed and your graphics drivers support OpenGL 3.3+

### Issue: Model fails to spawn
**Solution**: Verify the model exists in the database or the URDF file path is correct

### Issue: No GUI appears
**Solution**: Use `gui_required:=true` parameter or check X11 forwarding if using Docker/SSH

## Key Points

- Gazebo worlds can be launched with various parameters to customize the simulation
- Robot models can be spawned into running simulations
- The launch process connects Gazebo to ROS 2 for communication
- Text-only commands allow for automated and remote simulation setup

## Next Steps

After successfully launching a world, you can:
- Connect ROS 2 nodes to control your robot
- Add sensors to your robot model
- Create custom world files with obstacles and environments