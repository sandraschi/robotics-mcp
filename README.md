# Robotics MCP Server

**By FlowEngineer sandraschi**

**Unified robotics control via MCP - Physical and virtual robots (bot + vbot + drones)**

## ⚠️ **CRITICAL REQUIREMENTS**

### **Hardware (Recommended)**
- **Physical Robot**: Moorebot Scout, Unitree Go2/G1/H1, PX4/ArduPilot drones
- **Without hardware**: Virtual robotics only (Unity3D + VRChat)

### **Software (MANDATORY)**
- ✅ **Unity 3D** (6000.2.14f1+) - [Installation Guide](docs/SETUP_PREREQUISITES.md#unity-3d-required-for-virtual-robotics)
- ✅ **VRChat** - [Installation Guide](docs/SETUP_PREREQUISITES.md#vrchat-required-for-social-vr-robotics)
- ✅ **5 MCP Servers** - [Installation Guide](docs/SETUP_PREREQUISITES.md#-required-mcp-servers)

**Without these, the system will not function.** See [Complete Setup Guide](docs/SETUP_PREREQUISITES.md).

[![FastMCP](https://img.shields.io/badge/FastMCP-2.13+-blue)](https://gofastmcp.com)
[![Python](https://img.shields.io/badge/Python-3.10+-green)](https://www.python.org)
[![Unity](https://img.shields.io/badge/Unity-6000.2.14+-black)](https://unity.com)
[![VRChat](https://img.shields.io/badge/VRChat-Required-blue)](https://hello.vrchat.com)
[![License](https://img.shields.io/badge/License-MIT-yellow)](LICENSE)
[![Status](https://img.shields.io/badge/Status-ALPHA-orange)](README.md#-important-alpha-status--dependencies)

## ⚠️ **REQUIRED: Prerequisites & Dependencies**

### **🔴 PHYSICAL ROBOTS (RECOMMENDED)**
**You SHOULD own one of these supported robots:**
- **Moorebot Scout** (primary focus) - Mecanum wheeled robot with LiDAR
- **Yahboom ROSMASTER Series** (ROS 2) - Multimodal AI robots with camera, navigation, optional arm/gripper
- **Unitree Go2** - Advanced quadrupedal robot
- **Unitree G1** - Humanoid robot with arms
- **Unitree H1** - Humanoid robot platform

*Physical robots provide the complete robotics experience. Virtual robots are for testing/simulation.*

### **🟡 REQUIRED SOFTWARE**
**You MUST install these applications:**

#### **Unity 3D** (Required for virtual robotics)
```bash
# Download and install Unity Hub from:
# https://unity.com/download

# Then install Unity Editor version 6000.2.14f1 or later:
# 1. Open Unity Hub
# 2. Go to "Installs" tab
# 3. Click "Add" → "Official releases"
# 4. Select "6000.2.14f1" (LTS recommended)
# 5. Install with default components + Android Build Support
```

#### **VRChat** (Required for social VR robotics)
```bash
# Download from Steam:
# https://store.steampowered.com/app/438100/VRChat/

# Or from VRChat website:
# https://hello.vrchat.com/
```

### **🟢 REQUIRED MCP SERVERS**
**You MUST install and configure these MCP servers:**

#### **1. Unity3D-MCP** (Virtual robot control)
```bash
# Clone and install:
git clone https://github.com/sandraschi/unity3d-mcp.git
cd unity3d-mcp
pip install -e .

# Add to Cursor MCP configuration
```

#### **2. OSC-MCP** (Real-time communication)
```bash
# Clone and install:
git clone https://github.com/sandraschi/osc-mcp.git
cd osc-mcp
pip install -e .

# Add to Cursor MCP configuration
```

#### **3. VRChat-MCP** (Social VR integration)
```bash
# Clone and install:
git clone https://github.com/sandraschi/vrchat-mcp.git
cd vrchat-mcp
pip install -e .

# Add to Cursor MCP configuration
```

#### **4. Blender-MCP** (3D model creation)
```bash
# Clone and install:
git clone https://github.com/sandraschi/blender-mcp.git
cd blender-mcp
pip install -e .

# Requires Blender 4.0+ installed
```

#### **5. Avatar-MCP** (Avatar management)
```bash
# Clone and install:
git clone https://github.com/sandraschi/avatar-mcp.git
cd avatar-mcp
pip install -e .
```

---

## ⚠️ Important: Alpha Status & Current State

**This server is in ALPHA status with Cursor MCP integration working:**

- **✅ Cursor MCP Integration**: Server now starts successfully in Cursor IDE
- **✅ Unity Integration**: Now enabled with robust error handling and fallbacks
- **⏸️ Other MCP Servers**: Composition temporarily limited for stability
- **Status**: Active development - features may change, break, or be incomplete
- **Virtual Robotics**: Prioritized (vbot) - physical robot support coming after hardware arrives (XMas 2025)
- **Core Functionality**: 7 portmanteau tools available for robot control and management

See [Cursor MCP Setup](#-cursor-mcp-setup) section for integration details.

## 🎯 Overview

Robotics MCP Server provides unified control for **physical robots** (ROS-based), **virtual robots** (Unity/VRChat), and **drones** (PX4/ArduPilot), with a focus on Moorebot Scout, Unitree robots, open-source drones, and virtual robotics testing.

**🚀 Project Stats**: ~9,200 lines of code, ~2,600 lines of tests, ~4,100 lines of documentation

⚠️ **ALPHA STATUS - CURSOR INTEGRATION WORKING**: Server successfully starts in Cursor MCP mode. Mounted servers temporarily disabled for stability.

### Key Features

- **Physical Robot Control**: Moorebot Scout (ROS 1.4), Unitree Go2/G1
- **YDLIDAR SuperLight (95g)** LiDAR integration for Scout
- **Drone Control**: PX4/ArduPilot drones with MAVLink, video streaming, navigation
- **Virtual Robot Control**: Unity3D/VRChat/Resonite integration via existing MCP servers
- **ROS Bridge Integration**: [Robot Operating System](docs/ROS_FUNDAMENTALS.md) 1.4 (Melodic) via rosbridge_suite
- **LiDAR Sensing**: [Affordable 3D LiDAR Guide](docs/LIDAR_GUIDE.md) - Livox Mid-360 ($399), RPLIDAR ($99), and more
- **Tiny Controllers**: [Pico & Micro Boards](docs/TINY_CONTROLLERS_GUIDE.md) - Raspberry Pi Pico, ESP32, Arduino Nano for small robots
- **Motion Detection**: [Pyroelectric Sensors Guide](docs/PYROELECTRIC_SENSORS_GUIDE.md) - AM312, HC-SR501 ultra-small PIR sensors ($1-5)
- **Multi-Robot Coordination**: Physical robots, virtual robots, and drones together
- **World Labs Marble/Chisel**: Environment generation and import
- **Drone Video Streaming**: RTSP/WebRTC streaming with OpenIPC integration
- **Dual Transport**: stdio (MCP) + HTTP (FastAPI) endpoints
- **MCP Server Composition**: Ready for integration with `osc-mcp`, `unity3d-mcp`, `vrchat-mcp`, `avatar-mcp`, `blender-mcp`, `gimp-mcp` (temporarily disabled)
- **14 Portmanteau Tools**: `robotics_system`, `robot_control`, `robot_behavior`, `robot_manufacturing`, `robot_virtual`, `robot_model_tools`, `vbot_crud`, `drone_control`, `drone_streaming`, `drone_navigation`, `drone_flight_control`, `workflow_management`, `robot_navigation`, `robot_camera`, `robot_animation`
- **Robot Model Creation**: Framework ready for automated 3D model creation

## 📚 Documentation

- **[Setup Prerequisites](docs/SETUP_PREREQUISITES.md)** ⚠️ **REQUIRED: Complete installation guide for Unity3D, VRChat, and all MCP servers**
- **[AI Research Workflow](docs/AI_RESEARCH_WORKFLOW.md)** 🧠 **Architect first: AI-powered research methodology for all development**
- **[Vienna Technical Museum Makerspace](docs/VIENNA_TECHNICAL_MUSEUM_MAKERSPACE.md)** 🛠️ **Fantastic makerspace - free equipment, pay only consumables!**
- **[Progress Report](docs/PROGRESS_REPORT.md)** 🎉 **Comprehensive project status and achievements!**
- **[Unity Vbot Instantiation Guide](docs/UNITY_VBOT_INSTANTIATION.md)** - Complete guide for instantiating virtual robots in Unity3D with proper terminology

## 🚀 Quick Start

### Prerequisites Check
⚠️ **BEFORE STARTING**: Complete all [Setup Prerequisites](docs/SETUP_PREREQUISITES.md) - Unity3D, VRChat, and MCP servers are REQUIRED.

### Installation

#### From PyPI (Recommended)

```bash
pip install robotics-mcp
```

#### From GitHub Releases

```bash
# Direct wheel download
pip install https://github.com/sandraschi/robotics-mcp/releases/download/v1.0.1b2/robotics_mcp-1.0.1b2-py3-none-any.whl

# Or from git
pip install git+https://github.com/sandraschi/robotics-mcp.git
```

#### For Development

```bash
# Clone repository
git clone https://github.com/sandraschi/robotics-mcp.git
cd robotics-mcp

# Install in development mode
pip install -e ".[dev]"
```

### Cursor MCP Integration

✅ **The robotics-mcp server now works in Cursor IDE!**

**Setup Steps:**
1. **Complete Prerequisites**: Install Unity3D, VRChat, and all required MCP servers
2. Install the package: `pip install -e ".[dev]"`
3. Add to Cursor MCP configuration using the provided `mcpb.json`
4. The server will automatically start when enabled in Cursor

**Available Tools:**
- `robotics_system` - System management (help, status, list_robots)
- `robot_control` - Unified physical & virtual robot control (Scout, Unitree, Yahboom, Dreame)
- `robot_behavior` - Advanced robot behavior, animation, and manipulation
- `robot_manufacturing` - 3D printing, CNC, laser cutting operations
- `robot_virtual` - Virtual robotics environments and testing
- `robot_model_tools` - 3D model creation, conversion, and optimization
- `vbot_crud` - Virtual robot lifecycle management (create, read, update, delete)
- `drone_control` - Core drone flight operations (takeoff, land, move, status)
- `drone_streaming` - Video streaming and recording (FPV, RTSP, WebRTC)
- `drone_navigation` - GPS navigation and waypoints (follow_me, geofencing)
- `drone_flight_control` - Advanced flight modes (missions, parameter tuning)
- `workflow_management` - Robotics workflow orchestration and automation
- `robot_navigation` - Path planning, obstacle avoidance, SLAM
- `robot_camera` - Camera control, computer vision, object detection
- `robot_animation` - Animation sequences, motion planning, kinematics

### MCP Server Integration

✅ **ENABLED WITH SAFETY**: Unity3D-MCP is now enabled with robust error handling, timeouts, and fallbacks.

**Active Integration:**
- **`osc-mcp`**: ✅ **ENABLED** - OSC communication for real-time robot control
- **`unity3d-mcp`**: ✅ **ENABLED** - Unity3D integration for virtual robotics
- **`vrchat-mcp`**: ⏸️ **DISABLED** - VRChat integration (protocol conflicts)
- **`avatar-mcp`**: ⏸️ **DISABLED** - Avatar management (timeseries conflicts)
- **`blender-mcp`**: ⏸️ **DISABLED** - 3D model creation (protocol hangs)
- **`gimp-mcp`**: ⏸️ **DISABLED** - Texture creation (protocol hangs)

**Safety Features:**
- **30-second timeouts** for server loading
- **3 retry attempts** with exponential backoff
- **Graceful fallbacks** to mock operations if Unity unavailable
- **Never blocks** robotics-mcp server operation
- **Comprehensive logging** for debugging

**Configuration:**
All required MCP servers are automatically loaded with error protection. If Unity is not available, virtual robot operations fall back to mock mode with full functionality preservation.

### Configuration (Optional)

The server works out-of-the-box without configuration. For advanced setups, create `~/.robotics-mcp/config.yaml`:

```yaml
robotics:
  moorebot_scout:
    enabled: false
    robot_id: "scout_01"
    ip_address: "192.168.1.100"
    port: 9090
    mock_mode: true
  virtual:
    enabled: true
    platform: "unity"
server:
  enable_http: true
  http_port: 8080
  log_level: "INFO"
```

**MCP Integration Config** (for when mounted servers are re-enabled):
```yaml
mcp_integration:
  osc_mcp:
    enabled: true
    prefix: "osc"
  unity3d_mcp:
    enabled: true
    prefix: "unity"
  vrchat_mcp:
    enabled: true
    prefix: "vrchat"
  avatar_mcp:
    enabled: true
    prefix: "avatar"
  blender_mcp:
    enabled: true
    prefix: "blender"
  gimp_mcp:
    enabled: true
    prefix: "gimp"
```

### Running the Server

**Primary Usage**: Configure as MCP server in Cursor IDE using `mcpb.json`

#### Manual Operation (Development)

```bash
# MCP stdio mode (for testing)
python -m robotics_mcp --mode stdio

# HTTP API mode
python -m robotics_mcp --mode http --port 8080

# Dual mode (stdio + HTTP)
python -m robotics_mcp --mode dual --port 8080
```

## 🛠️ Usage

### MCP Tools

#### Robot Control

```python
# Get robot status
await robot_control(robot_id="scout_01", action="get_status")

# Move robot
await robot_control(
    robot_id="scout_01",
    action="move",
    linear=0.2,
    angular=0.0
)

# Stop robot
await robot_control(robot_id="scout_01", action="stop")

# Yahboom robot control
await robot_control(robot_id="yahboom_01", action="get_status")
await robot_control(robot_id="yahboom_01", action="home_patrol")
await robot_control(robot_id="yahboom_01", action="camera_capture")
await robot_control(robot_id="yahboom_01", action="ai_query", query="What's in front of me?")
```

#### Virtual Robotics

```python
# Spawn virtual robot in Unity
await virtual_robotics(
    robot_type="scout",
    action="spawn_robot",
    platform="unity",
    position={"x": 0, "y": 0, "z": 0}
)

# Load Marble environment
await virtual_robotics(
    action="load_environment",
    environment="stroheckgasse_apartment",
    platform="unity"
)
```

#### Robot Model Tools

```python
# Create Scout model from scratch (uses blender-mcp + gimp-mcp)
await robot_model_create(
    robot_type="scout",
    output_path="D:/Models/scout_model.fbx",
    format="fbx",
    dimensions={"length": 0.115, "width": 0.10, "height": 0.08},
    create_textures=True,
    texture_style="realistic"
)

# Import robot model into Unity
await robot_model_import(
    robot_type="scout",
    model_path="D:/Models/scout_model.fbx",
    format="fbx",
    platform="unity",
    project_path="D:/Projects/UnityRobots"
)

# Convert model between formats
await robot_model_convert(
    source_path="D:/Models/scout.fbx",
    source_format="fbx",
    target_format="glb",
    target_path="D:/Models/scout.glb"
)
```

#### Drone Control

```python
# Get drone status
await drone_control(
    operation="get_status",
    drone_id="px4_quad_01"
)

# Take off to 5m altitude
await drone_control(
    operation="takeoff",
    drone_id="px4_quad_01",
    altitude=5.0
)

# Move drone with velocity control
await drone_control(
    operation="move",
    drone_id="px4_quad_01",
    velocity_x=1.0,  # 1 m/s forward
    velocity_y=0.0,  # no lateral movement
    velocity_z=0.0,  # no vertical movement
    yaw_rate=0.1    # slight rotation
)

# Arm drone motors
await drone_control(
    operation="arm",
    drone_id="px4_quad_01"
)

# Return to launch position
await drone_control(
    operation="return_home",
    drone_id="px4_quad_01"
)
```

#### Drone Streaming

```python
# Start FPV video stream
await drone_streaming(
    operation="start_fpv",
    drone_id="px4_quad_01",
    quality="720p"
)

# Get stream URL for external viewing
stream_info = await drone_streaming(
    operation="get_stream_url",
    drone_id="px4_quad_01",
    protocol="rtsp"
)
print(f"Stream URL: {stream_info['url']}")

# Start recording video
await drone_streaming(
    operation="start_recording",
    drone_id="px4_quad_01",
    filename="flight_2025-01-17.mp4"
)

# Take a snapshot
await drone_streaming(
    operation="take_snapshot",
    drone_id="px4_quad_01",
    filename="aerial_view.jpg"
)
```

#### Drone Navigation

```python
# Get current GPS position
position = await drone_navigation(
    operation="get_position",
    drone_id="px4_quad_01"
)
print(f"Lat: {position['latitude']}, Lon: {position['longitude']}, Alt: {position['altitude']}")

# Set a waypoint for navigation
await drone_navigation(
    operation="set_waypoint",
    drone_id="px4_quad_01",
    latitude=37.7749,
    longitude=-122.4194,
    altitude=10.0
)

# Enable follow-me mode
await drone_navigation(
    operation="enable_follow_me",
    drone_id="px4_quad_01",
    target_id="operator_gps"
)

# Set geofence boundaries
await drone_navigation(
    operation="set_geofence",
    drone_id="px4_quad_01",
    fence_points=[
        {"lat": 37.7740, "lon": -122.4200},
        {"lat": 37.7750, "lon": -122.4200},
        {"lat": 37.7750, "lon": -122.4180},
        {"lat": 37.7740, "lon": -122.4180}
    ],
    max_altitude=30.0
)
```

#### Drone Flight Control

```python
# Set flight mode to AUTO
await drone_flight_control(
    operation="set_flight_mode",
    drone_id="px4_quad_01",
    mode="AUTO"
)

# Get available flight modes
modes = await drone_flight_control(
    operation="get_flight_modes",
    drone_id="px4_quad_01"
)
print(f"Available modes: {modes['modes']}")

# Upload a mission plan
await drone_flight_control(
    operation="upload_mission",
    drone_id="px4_quad_01",
    mission_plan={
        "waypoints": [
            {"lat": 37.7749, "lon": -122.4194, "alt": 10.0},
            {"lat": 37.7750, "lon": -122.4200, "alt": 15.0}
        ],
        "commands": ["takeoff", "waypoint", "land"]
    }
)

# Start mission execution
await drone_flight_control(
    operation="start_mission",
    drone_id="px4_quad_01",
    mission_id="recon_mission_01"
)

# Tune drone parameters
await drone_flight_control(
    operation="set_parameter",
    drone_id="px4_quad_01",
    param_name="WPNAV_SPEED",
    param_value=500  # cm/s
)
```

### Web Interface

The Robotics MCP server includes a modern web-based control panel for easy robot management:

```
http://localhost:8081
```

**Features:**
- Real-time robot control with intuitive movement buttons
- Live status monitoring (battery, position, sensors)
- Camera capture and arm/gripper control
- Command logging and connection status
- Responsive design for desktop and mobile

See [Web Interface Documentation](web/README.md) for detailed usage instructions.

### HTTP API

#### Health Check

```bash
curl http://localhost:8080/api/v1/health
```

#### List Robots

```bash
curl http://localhost:8080/api/v1/robots
```

#### Control Robot

```bash
curl -X POST http://localhost:8080/api/v1/robots/scout_01/control \
  -H "Content-Type: application/json" \
  -d '{"action": "move", "linear": 0.2, "angular": 0.0}'
```

#### List Tools

```bash
curl http://localhost:8080/api/v1/tools
```

#### Call Tool

```bash
curl -X POST http://localhost:8080/api/v1/tools/robot_control \
  -H "Content-Type: application/json" \
  -d '{"robot_id": "scout_01", "action": "get_status"}'
```

## 📚 Documentation

- **[ROS Fundamentals](docs/ROS_FUNDAMENTALS.md)** 🤖 **Complete guide to the Robot Operating System - what ROS is, why it matters, core concepts, and ROS 1 vs ROS 2**
- **[LiDAR Guide](docs/LIDAR_GUIDE.md)** 📡 **Affordable 3D sensing - Livox Mid-360 ($399), RPLIDAR ($99), integration, and robotics applications**
- **[Tiny Controllers Guide](docs/TINY_CONTROLLERS_GUIDE.md)** 🎮 **Smallest microcontroller boards for robotics - Raspberry Pi Pico, ESP32, Arduino Nano, Teensy**
- **[Pyroelectric Sensors Guide](docs/PYROELECTRIC_SENSORS_GUIDE.md)** 🔍 **Ultra-small motion detection - AM312 (6x4.5mm), HC-SR501, PIR sensors ($1-5)**
- **[Component Reuse Hacks](docs/COMPONENT_REUSE_HACKS.md)** 🔧 **Creative electronics salvage - Philips Hue bulbs, HDD motors, smartphone cameras**
- **[World Labs Unity Integration Fix](docs/WORLDLABS_UNITY_INTEGRATION_FIX.md)** 🏠 **Resolve Marble .spz to Unity splat format incompatibilities + Scout vbot improvements**
- **[Import Nekomimi-chan VRM Guide](docs/IMPORT_NEKOMIMI_VRM_GUIDE.md)** 🐱 **High-priority VRM avatar import for VRoid Studio model in avatar-mcp**
- **[Blender VRM Workflow for Robotics](docs/BLENDER_VRM_WORKFLOW_ROBOTICS.md)** 🔧 **Create custom VRM models for dogbots, diggers, articulated arms despite humanoid limitations**
- **[VRM Tools Alternatives](docs/VRM_TOOLS_ALTERNATIVES.md)** 🎨 **More generalized VRM creation tools beyond VRoid Studio for robots and non-humanoids**
- **[Comprehensive Project Notes](docs/COMPREHENSIVE_NOTES.md)** 📖 **Complete project documentation!**
- **[VRM vs Robot Models](docs/VRM_VS_ROBOT_MODELS.md)** 🤖 **VRM format guide - when to use VRM vs FBX/GLB**
- **[Unity Vbot Instantiation Guide](docs/UNITY_VBOT_INSTANTIATION.md)** 🎮 **Complete guide for instantiating virtual robots in Unity3D**
- [Implementation Plan](PLAN.md)
- [Quick Start: VRChat](docs/QUICK_START_VRCHAT.md) ⚡ **Get Scout into VRChat!**
- [ROS 1.4 Local Setup](docs/ROS1_LOCAL_SETUP.md) 🐳 **Full local ROS environment for Scout!**
- [VRChat Integration Guide](docs/VRChat_INTEGRATION.md)
- [VRChat Scout Setup](docs/VRCHAT_SCOUT_SETUP.md) - Complete guide
- [Architecture](docs/ARCHITECTURE.md)
- [API Reference](docs/API_REFERENCE.md)
- [MCP Integration](docs/MCP_INTEGRATION.md)

## 🧪 Testing

**Comprehensive test suite**: 21 test files, 2,642 lines of tests covering all 11 tools!

```bash
# Run all tests
pytest

# Run unit tests only
pytest tests/unit

# Run integration tests
pytest tests/integration

# Run with coverage
pytest --cov=robotics_mcp --cov-report=html

# Or use the PowerShell script
.\scripts\run-tests.ps1
```

## 🔧 Development

### Project Structure

```
robotics-mcp/
├── src/robotics_mcp/
│   ├── server.py              # Main FastMCP server
│   ├── clients/               # Robot client implementations
│   ├── integrations/          # MCP server integration wrappers
│   ├── tools/                 # Portmanteau tool implementations
│   └── utils/                 # Utilities (config, state, mock data)
├── tests/
│   ├── unit/                  # Unit tests
│   └── integration/           # Integration tests
├── docs/                      # Documentation
├── scripts/                   # Utility scripts
└── mcpb/                      # MCPB packaging
```

### Code Quality

```bash
# Format code
black src/ tests/

# Lint code
ruff check src/ tests/

# Type checking
mypy src/
```

## 🔧 Troubleshooting

### Cursor MCP Integration Issues

**Problem:** Server not appearing in Cursor MCP tools

**Symptoms:**
- "robotics-mcp" not showing in MCP tools list
- Tools not available in Cursor

**Quick Fix:**
1. Open Cursor Settings → Features → Model Context Protocol
2. Add new server using the `mcpb.json` configuration
3. Ensure the server shows as "Healthy" in the list
4. Restart Cursor if needed

**Server Status Check:**
```bash
# Verify server starts correctly
python -c "from robotics_mcp.server import RoboticsMCP; RoboticsMCP(); print('SUCCESS')"
```

**Configuration File:** Use `mcpb.json` in the project root for Cursor MCP setup.

### Other Issues

**Server won't start:**
- Check Python version: `python --version` (requires 3.10+)
- Verify dependencies: `pip install -e ".[dev]"`
- Check logs: `C:\Users\sandr\AppData\Roaming\Cursor\logs\`

**Tools not appearing:**
- Verify MCP server is enabled in Cursor IDE settings
- Check server logs for errors
- Try disabling and re-enabling the server

**Unity integration not working:**
- Ensure Unity Editor is running
- Verify Unity project path is correct
- Check `unity3d-mcp` server is healthy

## 🤝 Contributing

Contributions welcome! Please see [CONTRIBUTING.md](CONTRIBUTING.md) for guidelines.

## 📄 License

MIT License - see [LICENSE](LICENSE) for details.

## 🙏 Acknowledgments

- FastMCP framework
- ROS community
- Unity3D, VRChat, World Labs Marble/Chisel
- MCP ecosystem contributors

---

**Status**: ✅ **ALPHA - Cursor Integration Working** - Core robotics-mcp functionality operational, mounted servers temporarily disabled for stability

**⏸️ MCP Server Composition**: Currently runs standalone. Integration with `osc-mcp`, `unity3d-mcp`, `vrchat-mcp`, `avatar-mcp`, `blender-mcp`, `gimp-mcp` planned for future releases.

