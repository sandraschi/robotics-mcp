# Docker Setup for Robotics MCP

Docker containers for local ROS development and testing.

## 🐳 ROS 1.4 (Melodic) Container

Full ROS 1.4 environment for Scout development.

### Quick Start

```powershell
# Build and start
docker-compose -f docker/docker-compose.ros1.yml up -d

# Setup workspace
.\docker\scripts\setup-ros1-workspace.ps1 -Build

# Enter container
docker exec -it robotics-ros1-dev bash
```

### Features

- ✅ ROS 1.4 (Melodic) desktop-full
- ✅ rosbridge_suite for WebSocket communication
- ✅ Scout SDK mounted and ready
- ✅ X11 support for RViz
- ✅ Persistent workspace volume

### Ports

- `9090`: rosbridge WebSocket
- `11311`: ROS master
- `11312-11320`: ROS communication

### Usage

See [ROS1_LOCAL_SETUP.md](../docs/ROS1_LOCAL_SETUP.md) for complete guide.

## 📚 Documentation

- [ROS 1.4 Local Setup](../docs/ROS1_LOCAL_SETUP.md) - Complete guide
- [Scout SDK README](../../external/moorebot-scout-sdk/README.md)

