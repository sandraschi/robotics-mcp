# Robotics MCP - Comprehensive Project Notes

**Last Updated**: 2025-12-02  
**Version**: 0.1.0  
**Status**: Active Development - Virtual Robotics Phase

---

## 📋 Table of Contents

1. [Project Overview](#project-overview)
2. [Architecture](#architecture)
3. [Core Components](#core-components)
4. [MCP Server Composition](#mcp-server-composition)
5. [Tool Implementations](#tool-implementations)
6. [Robot Clients](#robot-clients)
7. [State Management](#state-management)
8. [Configuration System](#configuration-system)
9. [Transport Layers](#transport-layers)
10. [Testing Strategy](#testing-strategy)
11. [Development Workflows](#development-workflows)
12. [Integration Points](#integration-points)
13. [Deployment](#deployment)
14. [Future Roadmap](#future-roadmap)
15. [Troubleshooting](#troubleshooting)

---

## 🎯 Project Overview

### Purpose

Robotics MCP Server provides **unified control** for both **physical robots** (ROS-based) and **virtual robots** (Unity/VRChat), enabling AI assistants (Claude/Cursor) to control robots through a single, consistent interface.

### Key Design Principles

1. **Unified Interface**: Same tools work for both physical bots and virtual bots (vbots)
2. **Virtual-First Development**: Test in Unity/VRChat before hardware arrives
3. **MCP Server Composition**: Leverage existing MCP servers via `mount()`
4. **Portmanteau Pattern**: Consolidate related operations to prevent tool explosion
5. **Mock Mode**: Full functionality without hardware for development
6. **Dual Transport**: stdio (MCP protocol) + HTTP (FastAPI REST API)

### Target Robots

- **Moorebot Scout**: ROS 1.4 (Melodic), indoor security robot
- **Unitree Go2/G1**: ROS 2, quadruped/humanoid robots
- **Virtual Robots**: Unity3D, VRChat, Resonite environments

---

## 🏗️ Architecture

### High-Level Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    Claude/Cursor AI                          │
│                  (MCP Client)                                │
└──────────────────────┬──────────────────────────────────────┘
                       │ MCP Protocol (stdio/HTTP)
                       ▼
┌─────────────────────────────────────────────────────────────┐
│              Robotics MCP Server (FastMCP 2.13)              │
│  ┌──────────────────────────────────────────────────────┐   │
│  │  Portmanteau Tools                                    │   │
│  │  - robot_control (bot + vbot)                        │   │
│  │  - virtual_robotics (Unity/VRChat)                   │   │
│  │  - robot_sensors (sensor data)                       │   │
│  │  - robot_lidar (LiDAR operations)                    │   │
│  │  - robot_navigation (SLAM, mapping)                  │   │
│  └──────────────────────────────────────────────────────┘   │
│  ┌──────────────────────────────────────────────────────┐   │
│  │  MCP Server Composition (mount)                       │   │
│  │  - osc-mcp (prefix: osc)                             │   │
│  │  - unity3d-mcp (prefix: unity)                       │   │
│  │  - vrchat-mcp (prefix: vrchat)                       │   │
│  │  - avatar-mcp (prefix: avatar)                       │   │
│  └──────────────────────────────────────────────────────┘   │
│  ┌──────────────────────────────────────────────────────┐   │
│  │  State Manager (bot + vbot registry)                 │   │
│  │  - RobotStateManager                                 │   │
│  │  - RobotState (physical/virtual)                     │   │
│  └──────────────────────────────────────────────────────┘   │
└──────────────┬──────────────────────────────┬───────────────┘
               │                              │
    ┌──────────▼──────────┐      ┌───────────▼──────────┐
    │  Physical Robots     │      │  Virtual Robots      │
    │  (ROS-based)         │      │  (Unity/VRChat)      │
    │                      │      │                      │
    │  ┌────────────────┐  │      │  ┌────────────────┐  │
    │  │ ROS Bridge     │  │      │  │ Unity3D        │  │
    │  │ (rosbridge)    │  │      │  │ VRChat         │  │
    │  │ Port: 9090     │  │      │  │ Resonite       │  │
    │  └────────────────┘  │      │  └────────────────┘  │
    │                      │      │                      │
    │  - Moorebot Scout    │      │  - Virtual Scout     │
    │  - Unitree Go2/G1    │      │  - Virtual Go2/G1    │
    └──────────────────────┘      └──────────────────────┘
```

### Component Layers

1. **Transport Layer**: stdio (MCP) + HTTP (FastAPI)
2. **Tool Layer**: Portmanteau tools for unified operations
3. **Integration Layer**: MCP server composition
4. **Client Layer**: Robot-specific clients (ROS, Unity, VRChat)
5. **State Layer**: Robot registry and state management
6. **Config Layer**: YAML-based configuration

---

## 🔧 Core Components

### 1. Main Server (`server.py`)

**Location**: `src/robotics_mcp/server.py`

**Responsibilities**:
- FastMCP 2.13 server initialization
- Dual transport setup (stdio + HTTP)
- MCP server composition (mounting)
- Tool registration
- FastAPI HTTP endpoints
- Server lifecycle management

**Key Classes**:
- `RoboticsMCP`: Main server class
- `RoboticsConfig`: Configuration model

**Key Methods**:
- `__init__()`: Initialize server, managers, tools
- `_mount_mcp_servers()`: Mount external MCP servers
- `_register_tools()`: Register all portmanteau tools
- `_setup_http_routes()`: Setup FastAPI endpoints
- `run()`: Start server (stdio/http/dual mode)

### 2. State Manager (`utils/state_manager.py`)

**Purpose**: Manage robot connections and state

**Key Classes**:
- `RobotState`: Individual robot state
  - `robot_id`: Unique identifier
  - `robot_type`: Type (scout, go2, g1)
  - `platform`: For virtual robots (unity, vrchat)
  - `connected`: Connection status
  - `is_virtual`: Physical vs virtual flag
  - `metadata`: Additional robot data

- `RobotStateManager`: Registry manager
  - `register_robot()`: Register new robot
  - `get_robot()`: Get robot by ID
  - `list_robots()`: List with filtering
  - `update_robot_status()`: Update connection status
  - `unregister_robot()`: Remove robot

**State Storage**: In-memory (can be extended to persistent storage)

### 3. Configuration System (`utils/config_loader.py`)

**Purpose**: Load and manage YAML configuration

**Configuration Structure**:
```yaml
robotics:
  moorebot_scout:
    enabled: false
    robot_id: "scout_01"
    ip_address: "192.168.1.100"
    port: 9090
    mock_mode: true
    lidar:
      enabled: false
      type: "ydlidar_superlight"
  virtual:
    enabled: true
    platform: "unity"
  mcp_integration:
    osc_mcp: {enabled: true, prefix: "osc"}
    unity3d_mcp: {enabled: true, prefix: "unity"}
    vrchat_mcp: {enabled: true, prefix: "vrchat"}
    avatar_mcp: {enabled: true, prefix: "avatar"}
server:
  enable_http: true
  http_port: 8080
  log_level: "INFO"
```

**Config Locations**:
- Default: `~/.robotics-mcp/config.yaml`
- Custom: Via `--config` argument

**Features**:
- YAML parsing with validation
- Default configuration fallback
- Save/load functionality

### 4. Mock Data (`utils/mock_data.py`)

**Purpose**: Generate mock data for testing without hardware

**Functions**:
- `mock_lidar_scan()`: Generate mock LiDAR point cloud
- `mock_robot_status()`: Generate mock robot status
- `mock_sensor_data()`: Generate mock sensor readings
- `mock_map_data()`: Generate mock occupancy grid

**Usage**: Used when `mock_mode: true` or hardware unavailable

---

## 🔗 MCP Server Composition

### Mounted Servers

Robotics MCP uses FastMCP 2.13's `mount()` feature to integrate existing MCP servers:

#### 1. osc-mcp
- **Prefix**: `osc`
- **Purpose**: OSC/MIDI communication
- **Usage**: VRChat world control, robot movement via OSC
- **Tools**: `osc_send_osc`, `osc_receive_osc`, etc.

#### 2. unity3d-mcp
- **Prefix**: `unity`
- **Purpose**: Unity3D automation
- **Usage**: Virtual robot spawning, environment loading, model import
- **Tools**: `unity_import_model`, `unity_execute_method`, `unity_import_marble_world`, etc.

#### 3. vrchat-mcp
- **Prefix**: `vrchat`
- **Purpose**: VRChat control
- **Usage**: VRChat world interaction, OSC message sending
- **Tools**: `vrchat_send_osc_message`, `vrchat_get_avatar_state`, etc.

#### 4. avatar-mcp
- **Prefix**: `avatar`
- **Purpose**: Avatar/robot movement control
- **Usage**: Smooth locomotion for virtual robots
- **Tools**: `avatar_movement_walk`, `avatar_movement_turn`, etc.

### Mounting Strategy

```python
def _mount_mcp_servers(self):
    """Mount external MCP servers for composition."""
    mcp_config = self.config_data.get("robotics", {}).get("mcp_integration", {})
    
    # Mount with prefix and proxy mode (live linking)
    if mcp_config.get("osc_mcp", {}).get("enabled", True):
        try:
            from oscmcp.mcp_server import server as osc_mcp_server
            self.mcp.mount(osc_mcp_server, prefix="osc", as_proxy=True)
            self.mounted_servers["osc"] = osc_mcp_server
        except ImportError:
            logger.warning("osc-mcp not available, skipping mount")
    # ... repeat for other servers
```

**Benefits**:
- No code duplication
- Live linking (changes reflected immediately)
- Graceful fallback if servers unavailable
- Unified tool namespace

---

## 🛠️ Tool Implementations

### Portmanteau Pattern

All tools follow the portmanteau pattern - consolidating related operations into single tools to prevent tool explosion.

### Tool 1: `robot_control`

**File**: `tools/robot_control.py`

**Purpose**: Unified robot control (works for both bot + vbot)

**Operations**:
- `get_status`: Get robot status (battery, position, state)
- `move`: Control movement (linear/angular velocity)
- `stop`: Emergency stop
- `return_to_dock`: Return to charging dock (physical only)
- `stand`: Stand up (Unitree, physical only)
- `sit`: Sit down (Unitree, physical only)
- `walk`: Walking gait (Unitree, physical only)
- `sync_vbot`: Sync virtual bot with physical bot state

**Routing Logic**:
```python
if robot.is_virtual:
    return await self._handle_virtual_robot(...)
else:
    return await self._handle_physical_robot(...)
```

**Virtual Robot Handling**:
- Unity: Uses `avatar-mcp` or `unity3d-mcp` for movement
- VRChat: Uses `vrchat-mcp` OSC messages

**Physical Robot Handling**:
- ROS bridge communication (rosbridge_suite)
- Mock mode support

### Tool 2: `virtual_robotics`

**File**: `tools/virtual_robotics.py`

**Purpose**: Virtual robot operations (Unity/VRChat)

**Operations**:
- `spawn_robot`: Spawn robot in Unity/VRChat scene
- `move`: Control virtual robot movement
- `get_status`: Get virtual robot state
- `get_lidar`: Get virtual LiDAR scan (Unity physics raycast)
- `set_scale`: Scale robot size (for size testing)
- `load_environment`: Load Marble/Chisel environment
- `test_navigation`: Test pathfinding
- `sync_with_physical`: Sync vbot state with physical bot

**Implementation Details**:

**Spawn Robot**:
```python
async def _spawn_robot(...):
    # Register in state manager
    robot = self.state_manager.register_robot(robot_id, robot_type, platform=platform)
    
    if platform == "vrchat":
        # Use VRChat OSC to spawn
        await client.call_tool("vrchat_send_osc_message", ...)
    elif platform == "unity":
        # Use Unity tools to spawn
        await client.call_tool("unity_execute_method", ...)
```

**Load Environment**:
```python
async def _load_environment(...):
    # Use unity3d-mcp to import Marble world
    await client.call_tool("unity_import_marble_world", ...)
```

### Tool 3: `get_status` (System Tool)

**Purpose**: Get robotics MCP server status

**Returns**:
- Server version
- Registered robots (bot + vbot)
- Mounted MCP servers
- HTTP status
- Configuration status

### Tool 4: `list_robots` (System Tool)

**Purpose**: List all registered robots with filtering

**Filters**:
- `robot_type`: Filter by type (scout, go2, g1)
- `is_virtual`: Filter by virtual/physical

---

## 🤖 Robot Clients

### Physical Robot Clients (Planned)

**Location**: `clients/` (stubs for now)

#### 1. Moorebot Scout Client (`moorebot_client.py`)

**Protocol**: ROS 1.4 (Melodic) via rosbridge_suite

**Features**:
- ROS topic subscriptions (sensors, camera, audio)
- ROS service calls (patrol, navigation, docking)
- ROS message publishing (movement commands)
- Mock mode support

**ROS Topics**:
- `/CoreNode/h264`: Video stream
- `/CoreNode/aac`: Audio stream
- `/SensorNode/tof`: Time-of-flight sensor
- `/SensorNode/imu`: IMU data
- `/SensorNode/light`: Light sensor
- `/cmd_vel`: Movement commands
- `/odom`: Odometry

**ROS Services**:
- `/nav_patrol`: Start patrol route
- `/nav_patrol_stop`: Stop patrol
- `/return_home`: Return to dock

#### 2. Unitree Client (`unitree_client.py`)

**Protocol**: Unitree SDK (ROS 2 optional)

**Features**:
- SDK-based control
- ROS 2 bridge (optional)
- Gait control
- Manipulation (G1)

#### 3. ROS Bridge Client (`ros_bridge.py`)

**Purpose**: Generic ROS bridge client

**Library**: `roslibpy` (ROS 1) or `rclpy` (ROS 2)

**Features**:
- WebSocket connection to rosbridge
- Topic subscription/publishing
- Service calls
- Action client support

### Virtual Robot Clients

**Implementation**: Via mounted MCP servers

- **Unity**: `unity3d-mcp` tools
- **VRChat**: `vrchat-mcp` + `osc-mcp`
- **Resonite**: Similar to VRChat

---

## 📊 State Management

### Robot State Model

```python
class RobotState:
    robot_id: str              # Unique identifier
    robot_type: str            # scout, go2, g1
    platform: Optional[str]    # unity, vrchat (for virtual)
    connected: bool            # Connection status
    is_virtual: bool           # Physical vs virtual
    metadata: Dict[str, Any]   # Additional data
```

### State Manager Operations

**Registration**:
```python
robot = state_manager.register_robot(
    robot_id="scout_01",
    robot_type="scout",
    platform="unity",  # None for physical
    metadata={"position": {...}, "scale": 1.0}
)
```

**Querying**:
```python
# Get single robot
robot = state_manager.get_robot("scout_01")

# List all robots
all_robots = state_manager.list_robots()

# Filter by type
scouts = state_manager.list_robots(robot_type="scout")

# Filter by virtual/physical
vbots = state_manager.list_robots(is_virtual=True)
```

**Status Updates**:
```python
state_manager.update_robot_status("scout_01", connected=True)
```

### State Persistence (Future)

Currently in-memory. Future enhancements:
- SQLite database
- JSON file storage
- Redis for distributed systems

---

## ⚙️ Configuration System

### Configuration File Structure

**Location**: `~/.robotics-mcp/config.yaml`

**Sections**:

1. **robotics.moorebot_scout**: Scout-specific config
   - Connection settings
   - LiDAR configuration
   - Navigation settings
   - Patrol routes

2. **robotics.virtual**: Virtual robot settings
   - Platform selection
   - Unity/VRChat connection
   - Environment paths

3. **robotics.mcp_integration**: MCP server mounting
   - Enable/disable servers
   - Prefix configuration

4. **server**: Server settings
   - HTTP enable/port
   - Logging level

### Configuration Loading

```python
config_loader = ConfigLoader(config_path)
config_data = config_loader.load()  # Returns dict with defaults
```

### Default Configuration

If config file doesn't exist, uses sensible defaults:
- Mock mode enabled
- Virtual robots enabled
- All MCP servers enabled
- HTTP on port 8080

---

## 🌐 Transport Layers

### 1. stdio Transport (MCP Protocol)

**Purpose**: Claude Desktop integration

**Protocol**: MCP JSON-RPC over stdin/stdout

**Usage**:
```bash
python -m robotics_mcp.server --mode stdio
```

**Features**:
- Standard MCP protocol
- Tool calls via JSON-RPC
- Resource access
- Prompt templates

### 2. HTTP Transport (FastAPI)

**Purpose**: Web dashboards, REST API access

**Port**: 8080 (configurable)

**Endpoints**:

**Health & Status**:
- `GET /api/v1/health`: Health check
- `GET /api/v1/status`: Server status

**Robot Management**:
- `GET /api/v1/robots`: List all robots
- `GET /api/v1/robots/{robot_id}`: Get robot info
- `POST /api/v1/robots`: Register robot
- `POST /api/v1/robots/{robot_id}/control`: Control robot
- `DELETE /api/v1/robots/{robot_id}`: Unregister robot

**Tool Access**:
- `GET /api/v1/tools`: List all MCP tools
- `POST /api/v1/tools/{tool_name}`: Call MCP tool

**Usage**:
```bash
python -m robotics_mcp.server --mode http --port 8080
```

### 3. Dual Transport

**Purpose**: Both stdio and HTTP simultaneously

**Usage**:
```bash
python -m robotics_mcp.server --mode dual --port 8080
```

**Implementation**: HTTP server runs in background thread, stdio in main thread

---

## 🧪 Testing Strategy

### Test Structure

```
tests/
├── unit/                    # Unit tests
│   ├── test_state_manager.py
│   ├── test_config_loader.py
│   └── test_virtual_robotics.py
└── integration/             # Integration tests
    ├── test_server.py
    └── test_virtual_robotics.py
```

### Unit Tests

**Coverage**:
- State manager operations
- Configuration loading
- Mock data generation
- Tool parameter validation

**Example**:
```python
def test_register_robot(state_manager):
    robot = state_manager.register_robot("scout_01", "scout")
    assert robot.robot_id == "scout_01"
    assert robot.robot_type == "scout"
```

### Integration Tests

**Coverage**:
- Server initialization
- Tool execution
- MCP server mounting
- HTTP endpoint functionality

**Example**:
```python
@pytest.mark.integration
async def test_spawn_virtual_robot():
    server = RoboticsMCP(RoboticsConfig(enable_http=False))
    result = await server.virtual_robotics._spawn_robot(...)
    assert result["status"] == "success"
```

### Mock Mode Testing

All tests use mock mode to avoid hardware dependencies:
- Mock ROS bridge responses
- Mock Unity/VRChat responses
- Mock sensor data

### Test Execution

```bash
# Run all tests
pytest

# Run unit tests only
pytest tests/unit

# Run with coverage
pytest --cov=robotics_mcp --cov-report=html
```

---

## 🔄 Development Workflows

### Virtual-First Development

**Strategy**: Develop and test virtual robots first, then add physical robot support.

**Benefits**:
- No hardware required
- Faster iteration
- Lower cost
- Easier debugging

**Workflow**:
1. Implement virtual robot tools
2. Test in Unity/VRChat
3. Add physical robot support
4. Test with hardware

### Local ROS Development

**Docker Setup**: See `docs/ROS1_LOCAL_SETUP.md`

**Workflow**:
1. Start ROS 1.4 Docker container
2. Build Scout SDK
3. Start rosbridge
4. Connect robotics-mcp via rosbridge
5. Test ROS communication

### VRChat Integration Workflow

**Goal**: Get Scout into VRChat

**Steps**:
1. Spawn virtual robot: `virtual_robotics(action="spawn_robot", platform="vrchat")`
2. Control movement: `robot_control(action="move", linear=0.2)`
3. Test behaviors: Patrol, navigation, etc.

### Code Quality

**Tools**:
- `black`: Code formatting
- `ruff`: Linting
- `mypy`: Type checking
- `pytest`: Testing

**Scripts**:
```powershell
# Check standards
.\scripts\check-standards.ps1

# Run tests
.\scripts\run-tests.ps1
```

---

## 🔌 Integration Points

### 1. ROS Integration

**Protocol**: rosbridge_suite (WebSocket)

**Connection**:
```python
from robotics_mcp.clients.ros_bridge import ROSBridgeClient

client = ROSBridgeClient(host="localhost", port=9090)
await client.connect()
```

**Operations**:
- Subscribe to topics
- Publish to topics
- Call services
- Execute actions

### 2. Unity Integration

**Via**: `unity3d-mcp` (mounted)

**Operations**:
- Import models
- Spawn objects
- Execute Unity methods
- Import Marble/Chisel environments

### 3. VRChat Integration

**Via**: `vrchat-mcp` + `osc-mcp` (mounted)

**Operations**:
- Send OSC messages
- Control avatars
- World interactions

### 4. Avatar Control

**Via**: `avatar-mcp` (mounted)

**Operations**:
- Movement control
- Pose manipulation
- Animation control

---

## 🚀 Deployment

### Development Deployment

**Local**:
```bash
python -m robotics_mcp.server --mode dual
```

**Docker** (ROS 1.4):
```bash
docker-compose -f docker/docker-compose.ros1.yml up -d
```

### Production Deployment

**Requirements**:
- Python 3.10+
- FastMCP 2.13+
- Optional: ROS 1.4/2 for physical robots
- Optional: Unity/VRChat for virtual robots

**Configuration**:
- Set `mock_mode: false` for physical robots
- Configure robot IP addresses
- Set up network access

**Monitoring**:
- HTTP health endpoint: `/api/v1/health`
- Server status: `/api/v1/status`
- Structured logging (structlog)

---

## 🗺️ Future Roadmap

### Phase 1: Foundation ✅ (Complete)
- [x] FastMCP 2.13 server
- [x] Dual transport (stdio + HTTP)
- [x] MCP server composition
- [x] State management
- [x] Configuration system
- [x] Basic tools

### Phase 2: Virtual Robotics ⚡ (In Progress)
- [x] Virtual robot spawning
- [x] VRChat integration
- [x] Unity integration
- [ ] Virtual LiDAR implementation
- [ ] Navigation testing
- [ ] Size testing

### Phase 3: Unitree Integration (Planned)
- [ ] Unitree SDK integration
- [ ] Virtual Unitree models
- [ ] Unitree-specific tools

### Phase 4: Moorebot Scout Physical (After Hardware)
- [ ] ROS 1.4 bridge implementation
- [ ] YDLIDAR integration
- [ ] SLAM and navigation
- [ ] Sensor data acquisition
- [ ] Patrol route management

### Phase 5: Advanced Features (Future)
- [ ] Multi-robot coordination
- [ ] Persistent state storage
- [ ] Advanced navigation
- [ ] Machine learning integration
- [ ] Cloud deployment

---

## 🐛 Troubleshooting

### Common Issues

#### 1. MCP Servers Not Mounting

**Symptoms**: Tools from mounted servers unavailable

**Solutions**:
- Check config: `mcp_integration` section
- Verify servers installed: `pip list | grep mcp`
- Check server logs for import errors
- Verify server compatibility (FastMCP 2.13+)

#### 2. ROS Bridge Connection Fails

**Symptoms**: Cannot connect to physical robot

**Solutions**:
- Verify rosbridge running: `rosnode list | grep rosbridge`
- Check port 9090 accessible
- Verify robot IP address
- Check network connectivity
- Use mock mode for testing

#### 3. Virtual Robot Spawn Fails

**Symptoms**: Robot doesn't appear in Unity/VRChat

**Solutions**:
- Check Unity/VRChat running
- Verify OSC enabled in VRChat
- Check world supports OSC spawning
- Review server logs for errors
- Test with mock mode first

#### 4. HTTP Server Won't Start

**Symptoms**: HTTP endpoints unavailable

**Solutions**:
- Check port 8080 not in use
- Verify `enable_http: true` in config
- Check firewall settings
- Review server logs

#### 5. State Manager Issues

**Symptoms**: Robots not found, duplicate IDs

**Solutions**:
- Check robot registration
- Verify robot_id uniqueness
- Review state manager logs
- Reset state manager if needed

### Debug Mode

**Enable verbose logging**:
```yaml
server:
  log_level: "DEBUG"
```

**Check server status**:
```python
status = await get_status()
print(status)
```

**List all robots**:
```python
robots = await list_robots()
print(robots)
```

---

## 📚 Additional Resources

### Documentation Files

- `PLAN.md`: Implementation plan
- `README.md`: Quick start guide
- `docs/ROS1_LOCAL_SETUP.md`: ROS 1.4 Docker setup
- `docs/VRChat_INTEGRATION.md`: VRChat integration guide
- `docs/VRCHAT_SCOUT_SETUP.md`: Complete VRChat setup
- `docs/QUICK_START_VRCHAT.md`: 30-minute VRChat guide

### External Documentation

- **FastMCP**: https://gofastmcp.com
- **ROS 1.4**: http://wiki.ros.org/melodic
- **rosbridge_suite**: http://wiki.ros.org/rosbridge_suite
- **Scout SDK**: `external/moorebot-scout-sdk/README.md`
- **MCP Central Docs**: `mcp-central-docs/docs/robotics/`

### Code References

- **Main Server**: `src/robotics_mcp/server.py`
- **Tools**: `src/robotics_mcp/tools/`
- **Utils**: `src/robotics_mcp/utils/`
- **Clients**: `src/robotics_mcp/clients/` (stubs)

---

## 📝 Development Notes

### Design Decisions

1. **Portmanteau Pattern**: Chosen to prevent tool explosion while maintaining functionality
2. **Virtual-First**: Prioritize virtual development for efficiency
3. **MCP Composition**: Use `mount()` instead of duplicating code
4. **Mock Mode**: Essential for development without hardware
5. **Dual Transport**: Support both MCP protocol and HTTP for flexibility

### Known Limitations

1. **State Persistence**: Currently in-memory only
2. **Physical Robot Support**: Stubs only, waiting for hardware
3. **ROS 2 Support**: Planned but not yet implemented
4. **Multi-Robot Coordination**: Basic support, advanced features planned

### Performance Considerations

- **State Manager**: In-memory, fast lookups
- **MCP Composition**: Live linking, minimal overhead
- **HTTP Server**: Background thread, non-blocking
- **Mock Data**: Generated on-demand, lightweight

### Security Considerations

- **ROS Bridge**: Network access required, consider firewall
- **HTTP API**: No authentication yet (add for production)
- **OSC Messages**: VRChat local only, relatively safe
- **Configuration**: Sensitive data in config file (encrypt for production)

---

## 🎯 Success Metrics

### Current Status

- ✅ Core server implementation complete
- ✅ Virtual robotics tools functional
- ✅ VRChat integration working
- ✅ Docker ROS 1.4 setup ready
- ⚡ Physical robot support (pending hardware)

### Goals

- **Tomorrow**: Scout in VRChat ✅
- **This Week**: Complete virtual robotics testing
- **Next Month**: Physical Scout integration (when hardware arrives)
- **Future**: Multi-robot coordination, advanced features

---

**Last Updated**: 2025-12-02  
**Maintained By**: Sandra  
**Status**: Active Development

