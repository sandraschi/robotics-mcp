# Robotics MCP Server

**Unified robotics control via MCP - Physical and virtual robots (bot + vbot)**

[![FastMCP](https://img.shields.io/badge/FastMCP-2.13+-blue)](https://gofastmcp.com)
[![Python](https://img.shields.io/badge/Python-3.10+-green)](https://www.python.org)
[![License](https://img.shields.io/badge/License-MIT-yellow)](LICENSE)
[![Status](https://img.shields.io/badge/Status-ALPHA-orange)](README.md#-important-alpha-status--dependencies)

## ⚠️ Important: Alpha Status & Dependencies

**This server is in ALPHA and requires multiple composited MCP servers to function:**

- **Required MCP Servers**: `osc-mcp`, `unity3d-mcp`, `vrchat-mcp`, `avatar-mcp`, `blender-mcp`, `gimp-mcp`
- **Status**: Active development - features may change, break, or be incomplete
- **Virtual Robotics**: Prioritized (vbot) - physical robot support coming after hardware arrives (XMas 2025)
- **MCP Server Composition**: This server composes 6+ MCP servers - all must be properly configured and running

See [MCP Server Dependencies](#-mcp-server-dependencies) section for setup details.

## 🎯 Overview

Robotics MCP Server provides unified control for both **physical robots** (ROS-based) and **virtual robots** (Unity/VRChat), with a focus on Moorebot Scout, Unitree robots, and virtual robotics testing.

**🚀 Project Stats**: ~9,200 lines of code, ~2,600 lines of tests, ~4,100 lines of documentation

⚠️ **ALPHA STATUS - ONGOING DEVELOPMENT**: This server is in active development and requires multiple composited MCP servers to function properly. See [MCP Server Dependencies](#-mcp-server-dependencies) below.

### Key Features

- **Physical Robot Control**: Moorebot Scout (ROS 1.4), Unitree Go2/G1
- **YDLIDAR SuperLight (95g)** LiDAR integration for Scout
- **Virtual Robot Control**: Unity3D/VRChat/Resonite integration via existing MCP servers
- **ROS Bridge Integration**: [Robot Operating System](docs/ROS_FUNDAMENTALS.md) 1.4 (Melodic) via rosbridge_suite
- **LiDAR Sensing**: [Affordable 3D LiDAR Guide](docs/LIDAR_GUIDE.md) - Livox Mid-360 ($399), RPLIDAR ($99), and more
- **Tiny Controllers**: [Pico & Micro Boards](docs/TINY_CONTROLLERS_GUIDE.md) - Raspberry Pi Pico, ESP32, Arduino Nano for small robots
- **Motion Detection**: [Pyroelectric Sensors Guide](docs/PYROELECTRIC_SENSORS_GUIDE.md) - AM312, HC-SR501 ultra-small PIR sensors ($1-5)
- **Multi-Robot Coordination**: Physical and virtual robots together
- **World Labs Marble/Chisel**: Environment generation and import
- **Dual Transport**: stdio (MCP) + HTTP (FastAPI) endpoints
- **MCP Server Composition**: Integrates with `osc-mcp`, `unity3d-mcp`, `vrchat-mcp`, `avatar-mcp`, `blender-mcp`, `gimp-mcp`
- **Robot Model Creation**: Automated 3D model creation using `blender-mcp` (geometry) + `gimp-mcp` (textures)

## 📚 Documentation

- **[Progress Report](docs/PROGRESS_REPORT.md)** 🎉 **Comprehensive project status and achievements!**
- **[Unity Vbot Instantiation Guide](docs/UNITY_VBOT_INSTANTIATION.md)** - Complete guide for instantiating virtual robots in Unity3D with proper terminology

## 🚀 Quick Start

### Installation

```bash
# Clone repository
git clone https://github.com/sandraschi/robotics-mcp.git
cd robotics-mcp

# Install dependencies
pip install -e ".[dev]"

# Or install from PyPI (when published)
pip install robotics-mcp
```

### MCP Server Dependencies

⚠️ **CRITICAL**: This server requires multiple composited MCP servers to function. You must install and configure all of the following:

**Required MCP Servers:**
- **`osc-mcp`**: OSC communication for real-time robot control
- **`unity3d-mcp`**: Unity3D integration for virtual robotics
- **`vrchat-mcp`**: VRChat integration for social VR testing
- **`avatar-mcp`**: Avatar management and animation
- **`blender-mcp`**: 3D model creation (geometry)
- **`gimp-mcp`**: Texture creation and image processing

**Setup Steps:**
1. Install each MCP server in your Claude Desktop/Cursor IDE configuration
2. Ensure all servers are enabled and running
3. Configure `robotics-mcp` to use the correct prefixes (see Configuration below)
4. Verify connectivity using `robotics_system(operation="status")` tool

**Troubleshooting:**
- If tools fail, check that all required MCP servers are enabled
- Use `robotics_system(operation="status")` to verify mounted server connectivity
- See [MCP Integration](docs/MCP_INTEGRATION.md) for detailed setup

### Configuration

Create configuration file at `~/.robotics-mcp/config.yaml`:

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
    unity:
      host: "localhost"
      port: 8080
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
server:
  enable_http: true
  http_port: 8080
  log_level: "INFO"
```

### Running the Server

#### stdio Mode (MCP Protocol)

```bash
python -m robotics_mcp.server --mode stdio
```

#### HTTP Mode (FastAPI)

```bash
python -m robotics_mcp.server --mode http --port 8080
```

#### Dual Mode (Both stdio + HTTP)

```bash
python -m robotics_mcp.server --mode dual --port 8080
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

### MCP Server UI Desync (Critical for Composite Servers)

**Problem:** Cursor IDE UI shows "error, no tools" for MCP servers, but the server process is running fine. This is **critical** for `robotics-mcp` because it composes 6+ MCP servers - if ANY desyncs, the entire workflow breaks.

**Symptoms:**
- Cursor IDE shows "error, no tools" for one or more MCP servers
- Agent silently fails when trying to use robotics-mcp tools
- Workflow breaks without clear error message

**Quick Fix:**
1. Open Cursor IDE Settings → Features → Model Context Protocol
2. For each server showing "error, no tools": Disable → Enable
3. Wait ~5-10 seconds for reconnection
4. Verify tools appear

**Health Check:**
```powershell
# Check health of all robotics-mcp dependencies
.\scripts\check-robotics-mcp-health.ps1
```

**Automated Fix:**
```powershell
# Get instructions for fixing desynced servers
.\scripts\fix-mcp-desync.ps1
```

**Full Documentation:**
- [Composite MCP UI Desync - Critical Issue](docs/COMPOSITE_MCP_UI_DESYNC_CRITICAL.md)
- [Cursor IDE Log Structure Analysis](docs/CURSOR_IDE_LOG_STRUCTURE_ANALYSIS.md)

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

**Status**: ⚠️ **ALPHA - Ongoing Development** - Virtual robotics (vbot) prioritized, physical robot support coming after hardware arrives (XMas 2025)

**⚠️ Requires Multiple MCP Servers**: This server composes 6+ MCP servers (`osc-mcp`, `unity3d-mcp`, `vrchat-mcp`, `avatar-mcp`, `blender-mcp`, `gimp-mcp`) - all must be properly configured and running for full functionality.

