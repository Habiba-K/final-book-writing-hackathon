---
title: File Naming Conventions
sidebar_position: 12
---

# File Naming Conventions for Digital Twin Module

This document establishes consistent file naming conventions for all files in the Digital Twin module to ensure organization, maintainability, and ease of navigation.

## Directory Structure Convention

### Module-Level Directories
```
book-site/
└── docs/
    └── module-2-digital-twin/
        ├── 01-digital-twin-overview/
        ├── 02-gazebo-simulation-basics/
        ├── 03-unity-robotics-integration/
        ├── 04-ros-gazebo-unity-bridge/
        ├── 05-digital-twin-validation/
        ├── assets/
        ├── images/
        └── shared/
```

### Chapter-Level Directories
Each chapter directory should follow the pattern:
- `01-` prefix with sequential numbering
- Hyphen-separated descriptive name
- Lowercase letters only

## File Naming Standards

### Markdown Files
- **Main chapter file**: `index.md` (placed in each chapter directory)
- **Specification files**: `specification.md`
- **Exercise files**: `exercises.md`
- **Examples files**: `examples.md`
- **Code files**: `code-examples.md`

### Example Directory Structure
```
01-digital-twin-overview/
├── index.md                 # Main chapter content
├── specification.md         # Chapter specification
├── exercises.md             # Chapter exercises
├── examples.md              # Additional examples
├── code-examples.md         # Extended code examples
└── assets/                  # Chapter-specific assets
    ├── diagram.png
    └── sample-code.py
```

## Naming Rules

### 1. Character Usage
- Use lowercase letters only
- Use hyphens (`-`) as word separators
- Avoid spaces, underscores, or special characters
- Use alphanumeric characters only

### 2. Length Guidelines
- Keep names under 50 characters when possible
- Use descriptive but concise names
- Prioritize clarity over brevity

### 3. Specific File Types

#### Documentation Files
- `index.md` - Main content page for each section
- `specification.md` - Technical specifications
- `tutorial.md` - Step-by-step guides
- `reference.md` - Technical reference material
- `faq.md` - Frequently asked questions
- `troubleshooting.md` - Problem-solving guides

#### Code Files
- Python files: `*.py` (use descriptive names like `robot_controller.py`)
- Launch files: `*.launch.py` (like `simulation.launch.py`)
- URDF files: `*.urdf` or `*.xacro` (like `robot_model.urdf`)
- Configuration files: `*.yaml` (like `robot_config.yaml`)

#### Asset Files
- Images: `*.png`, `*.jpg`, `*.svg` (use descriptive names)
- Videos: `*.mp4`, `*.mov` (when applicable)
- 3D Models: `*.stl`, `*.dae`, `*.fbx` (for simulation assets)

## Examples of Good Naming

### Chapter Directories
- ✅ `01-digital-twin-overview`
- ✅ `02-gazebo-simulation-basics`
- ✅ `03-unity-robotics-integration`
- ✅ `04-ros-gazebo-unity-bridge`
- ✅ `05-digital-twin-validation`

### Asset Files
- ✅ `robot-model-diagram.png`
- ✅ `simulation-workflow.svg`
- ✅ `sensor-data-flow.png`
- ✅ `architecture-overview.svg`

### Code Files
- ✅ `robot_controller.py`
- ✅ `sensor_publisher.py`
- ✅ `simulation.launch.py`
- ✅ `robot_config.yaml`

## Examples of Bad Naming (to Avoid)

### Directory Names
- ❌ `Chapter1-Digital Twin Overview` (spaces and uppercase)
- ❌ `01_digital_twin_overview` (underscores)
- ❌ `first-chapter` (not descriptive enough)

### File Names
- ❌ `MyImage.PNG` (uppercase extension)
- ❌ `Robot Controller.py` (spaces)
- ❌ `code_example_1.py` (underscores)
- ❌ `very_long_file_name_that_is_hard_to_read.py` (too long)

## Version Control Considerations

### Git-Friendly Naming
- All names should be compatible with Git
- Avoid characters that might cause issues on different platforms
- Use consistent casing to avoid platform-specific issues

### Branch Naming (for development)
- Feature branches: `feature/chapter-name` (e.g., `feature/gazebo-basics`)
- Bug fixes: `fix/issue-description` (e.g., `fix/typo-in-chapter-1`)
- Documentation: `docs/update-description` (e.g., `docs/add-exercises`)

## Implementation Guidelines

### For New Chapters
1. Use the sequential numbering system (01, 02, 03, etc.)
2. Include descriptive but concise names
3. Always include an `index.md` file as the main content page
4. Follow the same subdirectory structure for consistency

### For Assets
1. Place shared assets in the module root `assets/` directory
2. Place chapter-specific assets in `chapter-name/assets/` directory
3. Use descriptive names that clearly indicate content
4. Group related assets in appropriately named subdirectories

## Migration and Maintenance

### Adding New Files
- Always follow the naming conventions from the start
- Check existing patterns before creating new file types
- Update this document when new file types are introduced

### Refactoring Existing Files
- Update file names to match conventions when making significant changes
- Update all references to the old file names
- Test all links and imports after renaming

---

**Next**: [Resources Page](./resources.md)