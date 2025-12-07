# Physical AI & Humanoid Robotics Textbook

A comprehensive technical textbook for learning Physical AI and Humanoid Robotics, built with Docusaurus.

## Overview

This project provides a structured learning path through 4 modules covering:

1. **Module 1**: Robotic Nervous System (ROS 2) - Weeks 3-5
2. **Module 2**: Digital Twin (Gazebo & Unity) - Weeks 6-8
3. **Module 3**: NVIDIA Isaac (AI-Robot Brain) - Weeks 9-11
4. **Module 4**: Vision-Language-Action (VLA Pipelines) - Weeks 12-14

## Quick Start

### Prerequisites

- Node.js 18+
- npm or yarn
- Git

### Installation

```bash
# Clone the repository
git clone <your-repo-url>
cd book-writing-hackathon

# Install dependencies
cd book-site
npm install
```

### Development

```bash
# Start development server
npm start

# Visit http://localhost:3000
```

### Build

```bash
# Build for production
npm run build

# Serve production build locally
npm run serve
```

### Deployment

```bash
# Deploy to GitHub Pages
npm run deploy
```

## Project Structure

```
book-writing-hackathon/
├── book-site/                  # Docusaurus site
│   ├── docs/                   # Module content
│   │   ├── module-1/          # ROS 2 content
│   │   ├── module-2/          # Gazebo/Unity content
│   │   ├── module-3/          # Isaac content
│   │   └── module-4/          # VLA content
│   ├── src/
│   │   ├── components/        # React components
│   │   ├── data/              # Module data
│   │   └── pages/             # Custom pages
│   └── static/                # Static assets
└── specs/                     # Feature specifications
    └── 001-docusaurus-book-layout/
        ├── spec.md            # Requirements
        ├── plan.md            # Architecture
        ├── tasks.md           # Implementation tasks
        └── contracts/         # Module data contracts
```

## Features

- **Interactive Homepage**: 4 module overview cards with chapter listings
- **Responsive Design**: Works on mobile, tablet, and desktop
- **Accessible Navigation**: Keyboard-navigable with WCAG 2.1 AA compliance
- **GitHub Pages Deployment**: Automatic deployment via GitHub Actions

## Module Content

### Module 1: Robotic Nervous System (ROS 2)
- ROS 2 Fundamentals
- Nodes, Topics, and Services
- Actions and Python Agents
- URDF for Humanoids
- Launch Files and Packages

### Module 2: Digital Twin (Gazebo & Unity)
- Digital Twin Concepts
- Gazebo Fundamentals
- Physics Simulation
- Sensor Simulation
- ROS 2 Control Integration
- Unity Visualization

### Module 3: NVIDIA Isaac (AI-Robot Brain)
- Isaac Sim Introduction
- Synthetic Data Generation
- Isaac ROS Perception
- Visual SLAM and Navigation
- Sim-to-Real Transfer

### Module 4: Vision-Language-Action (VLA)
- Voice to Intent
- Natural Language to ROS 2 Actions
- Multi-Modal Perception
- Autonomous Task Execution
- Capstone Integration

## Development Workflow

1. **Edit Content**: Modify Markdown files in `book-site/docs/module-X/`
2. **Test Locally**: Run `npm start` to see changes
3. **Build**: Run `npm run build` to verify production build
4. **Deploy**: Push to main branch for automatic deployment

## Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md) for development guidelines.

## License

[Specify license]

## Resources

- [Docusaurus Documentation](https://docusaurus.io)
- [ROS 2 Humble](https://docs.ros.org/en/humble/)
- [NVIDIA Isaac Sim](https://docs.nvidia.com/isaac-sim/)
- [Gazebo](https://gazebosim.org/docs)

## Support

For issues or questions, please open an issue in the repository.
