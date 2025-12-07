# Data Model: Module 2 Digital Twin (Gazebo & Unity)

## Key Entities

### Digital Twin
- **Description**: Virtual representation of a physical robot system that connects Robot → ROS → Simulator → Unity (theory focus)
- **Attributes**:
  - virtual_representation: The digital model of the physical robot
  - physical_properties: Mass, dimensions, joint constraints
  - sensor_models: Simulated sensor configurations
  - behavior_characteristics: Movement patterns, response to environment
- **Relationships**: Contains Simulation Pipeline, Robot Model, and Bridge System

### Simulation Pipeline
- **Description**: The data flow system connecting ROS, Gazebo, and Unity components (theory focus)
- **Attributes**:
  - data_flow_direction: Direction of information between components
  - synchronization_mechanisms: How components maintain consistent state
  - communication_protocols: Methods of data transmission
- **Relationships**: Connected to Digital Twin, processes Robot Model data

### Robot Model
- **Description**: Digital representation of a physical robot in URDF/SDF format (theory focus)
- **Attributes**:
  - urdf_sdf_definition: Robot description in standard formats
  - physical_properties: Physical characteristics of the robot
  - sensor_configurations: Sensor placements and types
  - joint_definitions: Joint types and constraints
- **Relationships**: Used by Simulation Pipeline, represented in Digital Twin

### Bridge System
- **Description**: The communication layer enabling message flow between components (theory focus)
- **Attributes**:
  - message_formats: Standardized data formats for communication
  - protocol_adapters: Adapters for different system interfaces
  - synchronization_layer: Maintains consistency across systems
- **Relationships**: Connects all components in the Digital Twin architecture

## State Transitions

### Digital Twin States
- **Inactive**: Digital twin exists but simulation is not running
- **Initializing**: Simulation environment is being set up
- **Running**: Simulation is active and responding to inputs
- **Paused**: Simulation is temporarily stopped
- **Terminated**: Simulation has ended

### Simulation Pipeline States
- **Disconnected**: Components are not communicating
- **Connecting**: Establishing communication channels
- **Synchronized**: All components maintain consistent state
- **Desynchronized**: Components have inconsistent states
- **Recovering**: Attempting to restore synchronization

## Validation Rules

### Digital Twin Validation
- Must have proper architecture definition (Robot → ROS → Simulator → Unity)
- Must maintain consistency across all connected components
- Must handle state synchronization properly

### Simulation Pipeline Validation
- Data flow must follow defined architecture patterns
- Communication protocols must be standardized
- Synchronization mechanisms must be reliable

### Robot Model Validation
- URDF/SDF definitions must be valid
- Physical properties must be realistic
- Sensor configurations must be feasible

### Bridge System Validation
- Message formats must be standardized
- Communication must be bidirectional where required
- Synchronization must be maintained across all components

## Relationships

```
Digital Twin 1 -- 1 Simulation Pipeline
Simulation Pipeline 1 -- * Robot Model
Robot Model 1 -- 1 Bridge System
Bridge System 1 -- 1 Digital Twin (bidirectional communication)
```

The Digital Twin contains the core concept that connects all other entities. The Simulation Pipeline manages the flow of data between components. The Robot Model provides the specific representation used in the simulation. The Bridge System enables communication between all components, ensuring the entire architecture functions as intended.