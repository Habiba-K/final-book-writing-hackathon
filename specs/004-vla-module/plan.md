# Implementation Plan: VLA Module

**Branch**: `001-vla-module` | **Date**: 2025-12-10 | **Spec**: [link to spec](./spec.md)
**Input**: Feature specification from `/specs/[004-vla-module]/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

The Vision-Language-Action (VLA) module will implement a system that connects speech input to robotic action execution in simulation. The system will use OpenAI's Whisper for speech-to-text conversion, natural language processing for intent extraction, and ROS 2 for commanding a humanoid robot in simulation. The implementation will follow a modular architecture with separate components for voice processing, language understanding, vision perception, and action execution, all integrated in a simulation environment.

## Technical Context

**Language/Version**: Python 3.8+ (for ROS 2 compatibility)
**Primary Dependencies**: OpenAI Whisper, ROS 2 (Humble Hawksbill), rclpy, transformers, PyTorch, OpenCV, numpy, librosa
**Storage**: N/A (real-time processing)
**Testing**: pytest for unit tests, integration tests with ROS 2 launch files
**Target Platform**: Linux (Ubuntu 22.04 LTS - ROS 2 Humble)
**Project Type**: Single/web - simulation-based education module
**Performance Goals**: <2s response time from speech input to action command
**Constraints**: <1GB memory usage, offline-capable for Whisper, minimal GPU usage for vision processing
**Scale/Scope**: Single-user educational module, 1 humanoid robot simulation

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

Based on the constitution:
- Simplicity: Implementation will use modular design with clear interfaces between components
- Accuracy: All code examples will be validated with official ROS 2 and Whisper documentation
- Minimalism: Focus on core VLA pipeline without unnecessary features
- Free-Tier Friendly: Use of open-source tools and offline models where possible
- Student-Focused Clarity: Clear documentation and step-by-step instructions
- Documentation-Based Development: Follow official ROS 2 and Whisper documentation
- Consistency: Standard formatting and terminology throughout

## Project Structure

### Documentation (this feature)

```text
specs/004-vla-module/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
src/vla_module/
├── voice_processor/           # Whisper-based speech recognition
│   ├── whisper_client.py
│   ├── audio_input.py
│   └── transcription_handler.py
├── intent_extractor/          # Natural language to action mapping
│   ├── nlp_processor.py
│   ├── intent_classifier.py
│   └── action_mapper.py
├── vision_processor/          # Vision-language fusion
│   ├── vision_node.py
│   ├── object_detector.py
│   └── scene_interpreter.py
├── action_executor/           # ROS 2 action commands
│   ├── ros2_action_client.py
│   ├── task_sequencer.py
│   └── humanoid_controller.py
├── vla_integration/           # Main integration pipeline
│   ├── vla_manager.py
│   ├── pipeline_orchestrator.py
│   └── simulation_bridge.py
└── utils/                     # Shared utilities
    ├── config_loader.py
    ├── logger.py
    └── common_types.py

tests/
├── unit/
│   ├── test_voice_processor.py
│   ├── test_intent_extractor.py
│   ├── test_vision_processor.py
│   └── test_action_executor.py
├── integration/
│   ├── test_vla_pipeline.py
│   └── test_simulation_integration.py
└── contract/
    └── test_api_contracts.py
```

**Structure Decision**: Single project structure selected to maintain simplicity and coherence for the educational module. All components are organized in a modular fashion under the vla_module directory with clear separation of concerns.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| Multi-component architecture | Required to separate concerns for educational clarity | Single monolithic component would obscure learning objectives |
| Multiple dependencies | Each component requires specialized libraries (Whisper, ROS 2, CV) | Custom implementations would be beyond scope and educational value |

## Architecture Sketch

```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   Voice Input   │───▶│ Whisper Service  │───▶│ Intent Extractor│
│   (Microphone)  │    │ (Transcription)  │    │ (NLP Mapping)   │
└─────────────────┘    └──────────────────┘    └─────────────────┘
                                                        │
┌─────────────────┐    ┌──────────────────┐             ▼
│   Camera Feed   │───▶│ Vision Service   │───────▶  Action Mapper
│   (Simulation)  │    │ (Object Detection│         (ROS 2 Actions)
└─────────────────┘    │ Scene Understanding)      ┌─────────────┐
                       └──────────────────┘         │   Humanoid  │
                                                    │ Simulation  │
                                                    │   (Gazebo)  │
                                                    └─────────────┘
```

The architecture follows a pipeline pattern where voice and vision inputs are processed independently, then fused together in the action mapper to generate appropriate ROS 2 commands for the humanoid robot simulation.

## Research Summary

Based on research of the key technologies:

1. **OpenAI Whisper**: An open-source automatic speech recognition (ASR) system that converts speech to text. Can be deployed locally to meet offline requirements. Uses PyTorch and requires Python 3.8+.

2. **ROS 2 (Humble Hawksbill)**: The robotics middleware that will handle communication between different components and control the humanoid robot simulation. Uses rclpy for Python integration.

3. **Vision Processing**: Will use OpenCV and potentially transformer-based models for object detection and scene understanding. Can leverage ROS 2 vision packages.

4. **Natural Language Processing**: Will use transformer-based models or simpler NLP techniques to extract intent from transcribed speech and map to appropriate ROS 2 actions.

5. **Simulation Environment**: Likely Gazebo or similar simulation that supports humanoid robots and can interface with ROS 2.

## Key Technical Decisions

### 1. Whisper Model Selection
- **Option A**: OpenAI's hosted API (simplest but requires internet)
- **Option B**: Local deployment with smaller models (faster, offline but less accurate)
- **Option C**: Local deployment with larger models (accurate but resource-intensive)

**Decision**: Option B - Use a smaller Whisper model (e.g., whisper-tiny or whisper-base) deployed locally to maintain offline capability while keeping resource usage reasonable for student hardware.

### 2. NLP Approach for Intent Extraction
- **Option A**: Simple keyword matching (minimal dependencies, less flexible)
- **Option B**: Transformer-based classification (more accurate, requires more resources)
- **Option C**: Rule-based parsing (moderate complexity, customizable)

**Decision**: Option A - Start with keyword matching and simple pattern recognition to maintain simplicity and meet the constitution's simplicity principle, with extensibility for more advanced NLP in the future.

### 3. Vision Integration
- **Option A**: Use ROS 2 vision packages (standard, well-documented)
- **Option B**: Custom OpenCV pipeline (more control, potentially more complex)
- **Option C**: Integration with existing perception stack from previous modules

**Decision**: Option C - Integrate with existing perception capabilities from Modules 2 and 3 (Gazebo simulation and Isaac perception components) to maintain consistency and build on previous learning.

## Section Structure for Implementation

The VLA module will be implemented in 5 interconnected components following the chapter structure:

1. **Voice to Intent (Whisper)** - Core speech recognition and transcription
2. **NL to ROS 2 Actions** - Natural language processing and action mapping
3. **Multi-Modal Perception** - Vision-language fusion for scene understanding
4. **Autonomous Task Execution** - Task sequencing and error recovery
5. **Capstone Integration** - Complete pipeline integration and testing

## Quality Validation Approach

1. **Unit Testing**: Each component will have dedicated unit tests covering core functionality
2. **Integration Testing**: Tests for the pipeline connecting all components
3. **Simulation Testing**: End-to-end testing in the humanoid robot simulation
4. **Performance Validation**: Verify response times and resource usage meet constraints
5. **Educational Validation**: Confirm that students can successfully complete the learning objectives

## Testing Strategy

Testing will be tied directly to the acceptance criteria from the specification:

- **FR-001**: Test Whisper transcription accuracy with various audio inputs
- **FR-002**: Test intent extraction with various natural language commands
- **FR-003**: Test mapping from language to ROS 2 actions
- **FR-004**: Test vision-language fusion with various scene configurations
- **FR-005**: Test task sequencing and progress monitoring
- **FR-006**: Test complete VLA pipeline integration
- **FR-007**: Test feedback mechanisms for students
- **FR-008**: Test error recovery scenarios
- **FR-009**: Test complete "voice command → pick object" scenario

Each test will be designed to validate the specific acceptance scenarios defined in the specification.