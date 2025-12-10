# Quickstart Guide: VLA Module

## Prerequisites

Before starting with the Vision-Language-Action (VLA) module, ensure you have:

1. **ROS 2 Humble Hawksbill** installed on Ubuntu 22.04 LTS
2. **Python 3.8+** with pip package manager
3. **Git** for version control
4. **Basic understanding** of ROS 2 concepts (nodes, topics, services, actions)
5. **Previous modules completed** (Modules 1-3) for simulation environment

## Installation

### 1. Clone the Repository
```bash
git clone <repository-url>
cd <repository-directory>
```

### 2. Set Up Python Environment
```bash
cd src/vla_module
python3 -m venv vla_env
source vla_env/bin/activate  # On Windows: vla_env\Scripts\activate
pip install --upgrade pip
```

### 3. Install Dependencies
```bash
pip install openai-whisper
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cpu
pip install opencv-python
pip install numpy
pip install librosa
pip install transformers
```

### 4. Install ROS 2 Dependencies
```bash
# From your ROS 2 workspace
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

## Basic Usage

### 1. Launch the Simulation Environment
```bash
# Terminal 1: Start the humanoid robot simulation
ros2 launch vla_module simulation.launch.py
```

### 2. Start the VLA System
```bash
# Terminal 2: Start the Vision-Language-Action pipeline
source install/setup.bash
ros2 run vla_module vla_manager.py
```

### 3. Test Voice Command
Speak a simple command like "Pick up the red cube" to test the system. The humanoid robot in simulation should respond by moving to pick up the object.

## Running Individual Components

### Voice Processing Only
```bash
ros2 run vla_module voice_processor.py
```

### Vision Processing Only
```bash
ros2 run vla_module vision_processor.py
```

### Intent Extraction Only
```bash
ros2 run vla_module intent_extractor.py
```

## Configuration

The VLA system can be configured using the `config/vla_config.yaml` file:

```yaml
voice_processor:
  model_size: "tiny"  # Options: tiny, base, small, medium, large
  language: "en"
  device: "cpu"  # Options: cpu, cuda (if GPU available)

intent_extractor:
  confidence_threshold: 0.7
  max_command_length: 100

vision_processor:
  camera_topic: "/camera/rgb/image_raw"
  detection_threshold: 0.5

action_executor:
  timeout: 30  # seconds
  max_retries: 3
```

## Testing the System

### Unit Tests
```bash
# Run all unit tests
python3 -m pytest tests/unit/

# Run specific component tests
python3 -m pytest tests/unit/test_voice_processor.py
```

### Integration Test
```bash
# Run end-to-end integration test
python3 -m pytest tests/integration/test_vla_pipeline.py
```

## Example Commands

Try these example voice commands to test the system:

1. "Pick up the red cube"
2. "Move to the blue cylinder"
3. "What objects do you see?"
4. "Go to the table and bring me the cup"
5. "Identify the green sphere"

## Troubleshooting

### Common Issues

1. **Audio Input Not Working**
   - Check that microphone permissions are granted
   - Verify audio device is selected correctly
   - Test with: `arecord -d 3 test.wav`

2. **Whisper Model Not Loading**
   - Ensure sufficient disk space (models can be 100MB-3GB)
   - Check internet connection for initial download
   - Verify Python environment is activated

3. **ROS 2 Communication Issues**
   - Ensure ROS 2 environment is sourced in all terminals
   - Check that nodes are discovering each other: `ros2 node list`
   - Verify correct network configuration

4. **Simulation Not Responding**
   - Check that Gazebo is running properly
   - Verify robot model is loaded in simulation
   - Confirm ROS 2 actions are properly connected

### Debugging Commands

```bash
# Check active ROS 2 nodes
ros2 node list

# Check ROS 2 topics
ros2 topic list

# Monitor audio input topic
ros2 topic echo /audio_input

# Monitor robot state
ros2 topic echo /robot_state
```

## Next Steps

After completing the quickstart:

1. Explore the individual components in the `src/vla_module/` directory
2. Modify the configuration parameters to see different behaviors
3. Add new voice commands by extending the intent classifier
4. Experiment with different vision processing approaches
5. Complete the full module exercises following the course curriculum

## Resources

- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)
- [OpenAI Whisper Documentation](https://github.com/openai/whisper)
- [Module 4 Course Materials](link-to-course-materials)
- [Troubleshooting Guide](link-to-troubleshooting)