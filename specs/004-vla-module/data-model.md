# Data Model: VLA Module

## Core Entities

### VoiceCommand
- **id**: string - Unique identifier for the command
- **transcription**: string - Text transcription of the spoken command
- **confidence**: float - Confidence score of the speech recognition (0.0-1.0)
- **timestamp**: datetime - Time when the command was received
- **raw_audio**: bytes - Raw audio data (optional, for debugging)
- **status**: enum - Current processing status (RECEIVED, TRANSCRIBING, PROCESSED, FAILED)

### Intent
- **id**: string - Unique identifier for the intent
- **command_text**: string - Original transcribed text
- **action_type**: string - Type of action to perform (PICK_UP, MOVE_TO, IDENTIFY, etc.)
- **parameters**: object - Action-specific parameters (object_name, location, etc.)
- **confidence**: float - Confidence score of intent classification (0.0-1.0)
- **extracted_entities**: array - Named entities extracted from the command

### VisionData
- **id**: string - Unique identifier for the vision data
- **timestamp**: datetime - Time when the image was captured
- **image_data**: bytes - Image data from the camera feed
- **detected_objects**: array - List of detected objects with bounding boxes
- **scene_description**: string - Text description of the scene
- **spatial_relationships**: object - Relationships between objects in the scene

### ActionCommand
- **id**: string - Unique identifier for the action
- **action_type**: string - Type of ROS 2 action to execute
- **parameters**: object - Action-specific parameters
- **target_robot**: string - Identifier of the target robot
- **priority**: int - Execution priority (0-10)
- **timeout**: int - Maximum time to wait for action completion (seconds)
- **dependencies**: array - Other actions that must complete first

### TaskSequence
- **id**: string - Unique identifier for the task sequence
- **name**: string - Human-readable name for the sequence
- **actions**: array - Ordered list of ActionCommand objects
- **status**: enum - Current status (PENDING, RUNNING, COMPLETED, FAILED, PAUSED)
- **current_step**: int - Index of the current action being executed
- **created_at**: datetime - Time when the sequence was created
- **completed_at**: datetime - Time when the sequence was completed (if applicable)

### RobotState
- **robot_id**: string - Unique identifier for the robot
- **position**: object - Current position (x, y, z coordinates)
- **orientation**: object - Current orientation (quaternion or euler angles)
- **joint_states**: object - Current state of all joints
- **gripper_state**: string - Current gripper state (OPEN, CLOSED, MOVING)
- **battery_level**: float - Current battery level (0.0-1.0)
- **last_updated**: datetime - Time when the state was last updated

### PerceptionResult
- **id**: string - Unique identifier for the perception result
- **timestamp**: datetime - Time when perception was performed
- **detected_objects**: array - Objects detected in the scene
- **object_properties**: object - Properties of detected objects (color, size, etc.)
- **confidence_scores**: object - Confidence scores for each detection
- **semantic_labels**: array - Semantic labels for scene understanding

## Data Flow Relationships

```
VoiceCommand --[transcription]--> Intent --[action mapping]--> ActionCommand
VisionData --[object detection]--> PerceptionResult --[scene fusion]--> ActionCommand
ActionCommand --[execution]--> TaskSequence --[monitoring]--> RobotState
```

## API Contracts

### Voice Processing Service
- Input: Audio stream or audio file
- Output: VoiceCommand object with transcription
- Error handling: Returns error status if transcription fails

### Intent Extraction Service
- Input: VoiceCommand object
- Output: Intent object with action type and parameters
- Error handling: Returns UNKNOWN intent if classification fails

### Vision Processing Service
- Input: Image data from camera
- Output: VisionData object with detected objects
- Error handling: Returns empty object list if detection fails

### Action Mapping Service
- Input: Intent and VisionData objects
- Output: ActionCommand object
- Error handling: Returns error if no appropriate action can be determined

## Validation Rules

1. **VoiceCommand**: Must have non-empty transcription and valid confidence score (0.0-1.0)
2. **Intent**: Must have valid action_type and required parameters based on action type
3. **VisionData**: Must have valid timestamp and non-empty detected_objects array
4. **ActionCommand**: Must have valid action_type and appropriate parameters
5. **TaskSequence**: Must have non-empty actions array and valid status
6. **RobotState**: Must have valid position and orientation coordinates

## Serialization Format

All data objects will be serialized using JSON format for compatibility with ROS 2 message passing and ease of debugging. Binary data (audio, images) will be base64 encoded when included in JSON messages.