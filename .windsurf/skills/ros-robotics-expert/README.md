# ROS Robotics Expert Skill

A comprehensive skill for Robotics Operating System (ROS) development, specifically tailored for the robotics-mcp project.

## Overview

This skill provides expertise in ROS 2 for robot development, including:
- ROS architecture and core concepts
- Development environment setup
- Navigation and control systems
- Perception and sensor integration
- Debugging and troubleshooting

## Quick Start

1. **Install ROS 2**: Follow [setup instructions](modules/setup-and-tools.md)
2. **Create workspace**: `mkdir -p ~/ros2_ws/src && cd ~/ros2_ws`
3. **Build**: `colcon build && source install/setup.bash`

## Key ROS Concepts

### Communication Patterns
- **Topics**: Publish/subscribe messaging for continuous data streams
- **Services**: Request/response for synchronous operations
- **Actions**: Asynchronous operations with feedback and cancellation

### Essential Commands
```bash
# Check ROS installation
ros2 --help

# List active nodes
ros2 node list

# Monitor topics
ros2 topic list
ros2 topic echo /topic_name

# Launch files
ros2 launch package_name launch_file.py
```

## Development Workflow

1. **Design**: Plan your robot system architecture
2. **Model**: Create URDF/SDF robot descriptions
3. **Implement**: Write ROS nodes for functionality
4. **Test**: Use RViz and Gazebo for simulation
5. **Deploy**: Transfer to physical hardware

## Common Packages

- **Navigation**: `nav2_bringup` for autonomous navigation
- **Perception**: `vision_opencv` for computer vision
- **Control**: `ros2_control` for hardware interface
- **Simulation**: `gazebo_ros` for physics simulation

## Resources

- [ROS 2 Documentation](https://docs.ros.org/en/humble/)
- [ROS Answers](https://answers.ros.org/)
- [ROS Discourse](https://discourse.ros.org/)

## Integration with Robotics MCP

This skill is specifically designed for the robotics-mcp project and provides ROS expertise for:
- Robot control and navigation
- Sensor data processing
- Multi-robot coordination
- Simulation and testing