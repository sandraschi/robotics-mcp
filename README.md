# Robotics MCP Server

**Unified robotics control via MCP - Physical and virtual robots (bot + vbot)**

[![FastMCP](https://img.shields.io/badge/FastMCP-2.13+-blue)](https://gofastmcp.com)
[![Python](https://img.shields.io/badge/Python-3.10+-green)](https://www.python.org)
[![License](https://img.shields.io/badge/License-MIT-yellow)](LICENSE)

## 🎯 Overview

Robotics MCP Server provides unified control for both **physical robots** (ROS-based) and **virtual robots** (Unity/VRChat), with a focus on Moorebot Scout, Unitree robots, and virtual robotics testing.

### Key Features

- **Physical Robot Control**: Moorebot Scout (ROS 1.4), Unitree Go2/G1
- **YDLIDAR SuperLight (95g)** LiDAR integration for Scout
- **Virtual Robot Control**: Unity3D/VRChat/Resonite integration via existing MCP servers
- **ROS Bridge Integration**: ROS 1.4 (Melodic) via rosbridge_suite
- **Multi-Robot Coordination**: Physical and virtual robots together
- **World Labs Marble/Chisel**: Environment generation and import
- **Dual Transport**: stdio (MCP) + HTTP (FastAPI) endpoints
- **MCP Server Composition**: Integrates with `osc-mcp`, `unity3d-mcp`, `vrchat-mcp`, `avatar-mcp`

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

- [Implementation Plan](PLAN.md)
- [Quick Start: VRChat](docs/QUICK_START_VRCHAT.md) ⚡ **Get Scout into VRChat!**
- [VRChat Integration Guide](docs/VRChat_INTEGRATION.md)
- [VRChat Scout Setup](docs/VRCHAT_SCOUT_SETUP.md) - Complete guide
- [Architecture](docs/ARCHITECTURE.md)
- [API Reference](docs/API_REFERENCE.md)
- [MCP Integration](docs/MCP_INTEGRATION.md)

## 🧪 Testing

```bash
# Run all tests
pytest

# Run unit tests only
pytest tests/unit

# Run integration tests
pytest tests/integration

# Run with coverage
pytest --cov=robotics_mcp --cov-report=html
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

**Status**: Beta - Virtual robotics (vbot) prioritized, physical robot support coming after hardware arrives (XMas 2025)

