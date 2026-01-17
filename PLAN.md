# Robotics MCP Server - Implementation Plan

**Last Updated**: 2025-01-17
**Status**: Active Development - Drone Integration Complete
**Target**: FastMCP 2.13+ compliant robotics orchestration server  
**Integration**: Uses `osc-mcp`, `unity3d-mcp`, `vrchat-mcp` via `mount()`

---

## 🎯 Executive Summary

**Goal**: Create a unified MCP server for robotics control, supporting **physical robots (ROS-based)**, **virtual robots (Unity/VRChat)**, and **drones (PX4/ArduPilot)**, with a focus on Moorebot Scout, Unitree robots, virtual robotics testing, and open-source drone integration.

**✅ COMPLETED**: Core drone integration implemented with 4 portmanteau tools (`drone_control`, `drone_streaming`, `drone_navigation`, `drone_flight_control`) supporting PX4/ArduPilot firmware via MAVLink protocol.

**Architecture**: 
- **Portmanteau pattern** to consolidate related operations, preventing tool explosion
- **MCP server composition** via `mount()` to integrate with existing MCP servers
- **Unified control** for both physical bots and virtual bots (vbots)

**Key Features**:
- **Physical Robot Control**: Moorebot Scout (ROS 1.4), Unitree Go2/G1
- **YDLIDAR SuperLight (95g)** LiDAR integration for Scout
- **Virtual Robot Control**: Unity3D/VRChat/Resonite integration via existing MCP servers
- **✅ Drone Control**: PX4/ArduPilot drones with MAVLink, video streaming, navigation
- **ROS Bridge Integration**: ROS 1.4 (Melodic) via rosbridge_suite
- **Multi-Robot Coordination**: Physical robots, virtual robots, and drones together
- **Patrol Route Management**: Routes work for bot, vbot, and drone
- **Sensor Data Aggregation**: Real sensors (Scout) + virtual sensors (Unity) + drone telemetry
- **SLAM and Mapping**: Real SLAM (Scout) + virtual mapping (Unity) + aerial mapping (drone)
- **Obstacle Avoidance**: Real (LiDAR) + virtual (Unity physics) + aerial (drone)
- **Virtual-First Testing**: Test in Unity/VRChat before hardware arrives
- **✅ Drone Video Streaming**: RTSP/WebRTC streaming with OpenIPC integration

**Integration Strategy**:
- **Use existing MCP servers** via `mount()`: `osc-mcp`, `unity3d-mcp`, `vrchat-mcp`, `avatar-mcp`
- **Duplicate only when needed**: Robot-specific operations not covered by existing servers
- **Compose workflows**: Cross-server operations (e.g., spawn vbot in Unity, control via OSC)
- **Virtual-first development**: Build vbot system first (efficient, no hardware needed)
- **World Labs Marble/Chisel**: Use for environment generation (splats/meshes)
- **Drone Integration**: Extend portmanteau tools for aerial robotics (flight control, streaming, navigation)
- **Open-Source Drone Ecosystem**: Support Shenzhen manufacturers (Tdrone, OmniNxt) and PX4/ArduPilot firmware

---

## 📋 Project Structure

```
robotics-mcp/
├── src/
│   └── robotics_mcp/
│       ├── __init__.py
│       ├── server.py                 # Main FastMCP server (mounts other MCP servers)
│       ├── state_manager.py          # State management (bot + vbot)
│       ├── clients/
│       │   ├── __init__.py
│       │   ├── moorebot_client.py    # Moorebot Scout ROS 1.4 client
│       │   ├── ydlidar_client.py     # YDLIDAR SuperLight ROS client
│       │   ├── unitree_client.py     # Unitree SDK client
│       │   └── ros_bridge.py         # Generic ROS 1.4 bridge (rosbridge_suite)
│       ├── integrations/
│       │   ├── __init__.py
│       │   ├── unity3d_integration.py  # Wrapper for unity3d-mcp
│       │   ├── vrchat_integration.py   # Wrapper for vrchat-mcp
│       │   └── osc_integration.py      # Wrapper for osc-mcp
│       ├── tools/
│       │   ├── __init__.py
│       │   ├── robot_control.py      # Portmanteau: movement, status (bot + vbot)
│       │   ├── robot_sensors.py      # Portmanteau: sensor data (bot + vbot)
│       │   ├── robot_lidar.py        # Portmanteau: LiDAR operations (bot + vbot)
│       │   ├── robot_navigation.py   # Portmanteau: SLAM, mapping, obstacle avoidance
│       │   ├── robot_patrol.py       # Portmanteau: patrol routes (bot + vbot)
│       │   ├── robot_camera.py       # Portmanteau: camera/streaming
│       │   ├── virtual_robotics.py   # Portmanteau: virtual robot ops (uses unity3d-mcp/vrchat-mcp)
│       │   └── multi_robot.py        # Portmanteau: coordination (bot + vbot)
│       └── utils/
│           ├── __init__.py
│           ├── mock_data.py          # Mock fixtures for testing
│           └── config_loader.py      # Configuration management
├── tests/
│   ├── unit/
│   │   ├── test_moorebot_client.py
│   │   ├── test_unitree_client.py
│   │   ├── test_virtual_client.py
│   │   ├── test_integrations.py      # Test MCP server integrations
│   │   └── test_tools.py
│   └── integration/
│       ├── test_robotics_integration.py
│       └── test_cross_server_workflows.py  # Test bot + vbot workflows
├── docs/
│   ├── README.md
│   ├── ARCHITECTURE.md
│   ├── MOOREBOT_INTEGRATION.md       # Latest Scout hardware/API specs
│   ├── UNITREE_INTEGRATION.md
│   ├── VIRTUAL_ROBOTICS.md           # Unity/VRChat integration guide
│   ├── MCP_INTEGRATION.md            # How we use osc-mcp, unity3d-mcp, vrchat-mcp
│   └── API_REFERENCE.md
├── config/
│   └── config.yaml.example
├── pyproject.toml
├── requirements.txt
└── README.md
```

---

## 🛠️ Implementation Phases

**⚠️ PRIORITY: Virtual-First Development**  
Since physical Scout hardware isn't available yet (arrives XMas 2025), we're building **vbot (virtual robot) system first**. This is efficient and allows full testing before hardware arrives.

### Phase 1: Foundation (Week 1)

**Goal**: Set up project structure, basic server, and MCP server integrations

**Tasks**:
- [ ] Initialize FastMCP 2.13+ server structure
- [ ] Set up project with `pyproject.toml` and dependencies
- [ ] **Integrate existing MCP servers**:
  - [ ] Mount `osc-mcp` (for robot communication via OSC)
  - [ ] Mount `unity3d-mcp` (for Unity virtual robotics + World Labs Marble/Chisel)
  - [ ] Mount `vrchat-mcp` (for VRChat world control)
  - [ ] Mount `avatar-mcp` (for avatar/robot movement and pose control)
  - [ ] Test MCP server composition
- [ ] Create state manager for robot connections (bot + vbot)
- [ ] Implement configuration loader (YAML-based)
- [ ] Create mock data fixtures for testing
- [ ] Set up basic tool structure (portmanteau pattern)
- [ ] Write unit tests for state management
- [ ] Write integration tests for MCP server mounts
- [ ] Create README with quick start guide

**Deliverables**:
- Working FastMCP server that starts without errors
- Successfully mounted `osc-mcp`, `unity3d-mcp`, `vrchat-mcp`, `avatar-mcp`
- Configuration system with example YAML
- Mock mode for all robot types (bot + vbot)
- Basic test suite
- MCP integration documentation

---

### Phase 2: Virtual Robotics (Vbot) - PRIORITY ⚡ (Week 2)

**Goal**: Build complete virtual robot system using Unity/VRChat + World Labs Marble/Chisel environments

**Why First**: Physical Scout hardware arrives XMas 2025. Building vbot first is efficient - we can test everything virtually, then seamlessly add physical bot support later.

**Tasks**:
- [ ] **Integrate with `unity3d-mcp`** (already mounted in Phase 1):
  - [ ] Use `unity3d-mcp` tools for Unity scene control
  - [ ] **World Labs Marble/Chisel Integration**:
    - [ ] Use `unity_import_marble_world` to import apartment environments
    - [ ] Import splats/meshes from Marble exports
    - [ ] Set up Chisel-edited environments
    - [ ] Test environment loading and rendering
  - [ ] Spawn Scout 3D model in Unity (from SCOUT_3D_MODEL.md)
  - [ ] Control virtual robot movement via Unity API
  - [ ] Test pathfinding and navigation in Marble environments
- [ ] **Integrate with `avatar-mcp`** (already mounted in Phase 1):
  - [ ] Use `avatar-mcp` movement tools (`movement.walk`, `movement.run`, `movement.turn`)
  - [ ] Use `interactive_pose_control` for robot pose manipulation
  - [ ] Map robot movements to avatar movement system
  - [ ] Test robot locomotion in virtual environments
- [ ] **Integrate with `vrchat-mcp`** (already mounted in Phase 1):
  - [ ] Use `vrchat-mcp` tools for VRChat world control
  - [ ] Spawn Scout in VRChat worlds
  - [ ] Control via OSC (using `osc-mcp`)
  - [ ] Test in social VR environment
- [ ] **Integrate with `osc-mcp`** (already mounted in Phase 1):
  - [ ] Use OSC for robot communication
  - [ ] Bridge between Unity/VRChat and robot control
- [ ] Create `virtual_robotics` portmanteau tool:
  - `spawn_robot` - Spawn robot in Unity/VRChat (uses unity3d-mcp/vrchat-mcp)
  - `move` - Control virtual robot movement (uses avatar-mcp movement tools)
  - `get_status` - Virtual robot state
  - `get_lidar` - Virtual LiDAR (Unity physics raycast)
  - `set_scale` - Scale robot size (for size testing)
  - `load_environment` - Load Marble/Chisel environment (uses unity3d-mcp `import_marble_world`)
  - `test_navigation` - Test pathfinding in Marble environments
  - `sync_with_physical` - Sync vbot state with physical bot (for future testing)
- [ ] Create `virtual_testing` portmanteau tool:
  - `test_size_fit` - Test if robot fits in space (in Marble apartment)
  - `test_doorway` - Test doorway clearance
  - `test_patrol_route` - Validate patrol route in Marble environment
  - `compare_robots` - Side-by-side comparison (vbot vs future physical bot)
  - `generate_report` - Test results summary
- [ ] **3D Model Integration**:
  - [ ] Reference `mcp-central-docs/docs/robotics/SCOUT_3D_MODEL.md`
  - [ ] Import Scout 3D model to Unity (via unity3d-mcp or manual)
  - [ ] Set up physics and colliders
  - [ ] Test virtual robot matches physical specs (11.5×10×8 cm)
- [ ] **World Labs Marble/Chisel Workflow**:
  - [ ] Import physical apartment as Marble splat/mesh
  - [ ] Use Chisel to edit/optimize environment
  - [ ] Test robot navigation in rebuilt apartment
  - [ ] Validate size constraints and doorways
- [ ] Write unit tests
- [ ] Write integration tests for cross-server workflows
- [ ] Document virtual robotics workflow

**Deliverables**:
- Complete virtual robot control via MCP (using unity3d-mcp/vrchat-mcp/avatar-mcp)
- World Labs Marble/Chisel environment integration
- Unified vbot control interface
- Size testing capabilities in Marble environments
- Complete documentation
- **Ready to test before hardware arrives!**

**References**: 
- `mcp-central-docs/docs/robotics/VIRTUAL_ROBOTICS_APPROACH.md`
- `mcp-central-docs/docs/robotics/SCOUT_3D_MODEL.md`
- `unity3d-mcp/` - Unity automation + World Labs Marble/Chisel tools
- `vrchat-mcp/` - VRChat control tools
- `avatar-mcp/` - Avatar movement and pose control
- `osc-mcp/` - OSC communication tools

---

### Phase 3: Unitree Integration (Week 3)

**Goal**: Add Unitree Go2 and G1 support (virtual + physical)

**Tasks**:
- [ ] Research Unitree SDK 2.0 API
- [ ] Implement Unitree client wrapper
- [ ] Create virtual Unitree models (for vbot testing)
- [ ] Extend `robot_control` tool for Unitree:
  - `get_status` - Battery, joint states, mode
  - `move` - Locomotion control (Go2: quadruped, G1: bipedal)
  - `stand` - Stand up (G1)
  - `sit` - Sit down (G1)
  - `walk` - Walking gait control
  - `stop` - Emergency stop
- [ ] Extend `robot_sensors` tool for Unitree:
  - `get_lidar` - LiDAR scan data (Go2/G1)
  - `get_camera` - Camera feed (G1 binocular)
  - `get_imu` - IMU data
  - `get_joint_states` - Joint positions/velocities
- [ ] Create `robot_manipulation` portmanteau tool (G1 only):
  - `grasp` - Close gripper
  - `release` - Open gripper
  - `move_arm` - Arm control
  - `get_arm_state` - Current arm position
- [ ] Write unit tests with mock data
- [ ] Document Unitree integration

**Deliverables**:
- Unitree Go2 and G1 control (virtual + physical)
- Mock mode for testing
- Documentation with use cases

**Reference**: `unitree-robotics/` directory structure

---

### Phase 4: Moorebot Scout Physical Integration (Week 4 - After Hardware Arrives)

**Goal**: Complete Moorebot Scout ROS 1.4 bridge integration + YDLIDAR SuperLight LiDAR

**Tasks**:
- [ ] **Review Pilot Labs Scout GitHub Repository**:
  - [ ] Repository already cloned at: `external/moorebot-scout-sdk/`
  - [ ] Review ROS 1.4 (Melodic) workspace structure (`roller_eye`)
  - [ ] **Reference latest Scout documentation**:
    - [ ] `mcp-central-docs/docs/robotics/SCOUT_HARDWARE.md` - Complete hardware specs
    - [ ] `mcp-central-docs/docs/robotics/SCOUT_API.md` - Complete ROS 1.4 API reference
    - [ ] `mcp-central-docs/docs/robotics/SCOUT_USAGE_PLAN.md` - Usage workflows
  - [ ] Document ROS topics, services, and message types (from SCOUT_API.md)
  - [ ] Reference for Scout-specific ROS integration
  - [ ] Update repository if needed: `git pull origin main`
- [ ] Port Moorebot client from `tapo-camera-mcp` (reference implementation)
- [ ] Implement ROS 1.4 bridge WebSocket client (rosbridge_suite for ROS 1)
- [ ] **YDLIDAR SuperLight (95g) Integration**:
  - [ ] Research YDLIDAR lightweight model specifications (G2, X4, or similar)
  - [ ] Install YDLIDAR ROS 1 driver (ydlidar_ros_driver for ROS Melodic)
  - [ ] Implement YDLIDAR client wrapper (ROS 1 topic: `/scan`)
  - [ ] Add mounting configuration (top of Scout, ~8cm height)
  - [ ] Test LiDAR data acquisition via ROS 1
  - [ ] Calibrate LiDAR mounting offset
- [ ] Create `robot_control` portmanteau tool:
  - `get_status` - Battery, position, state
  - `move` - Linear/angular velocity control
  - `stop` - Emergency stop
  - `return_to_dock` - Auto-docking
- [ ] Create `robot_sensors` portmanteau tool:
  - `get_tof` - Time-of-flight distance
  - `get_imu` - IMU data (orientation, acceleration)
  - `get_light` - Light sensor readings
  - `get_all` - All sensor data at once
- [ ] Create `robot_lidar` portmanteau tool:
  - `get_scan` - Get latest LiDAR scan (point cloud)
  - `get_range` - Get distance in specific direction
  - `get_obstacles` - Detect obstacles in scan
  - `get_clearance` - Check clearance in direction
  - `calibrate` - Calibrate LiDAR mounting
  - `get_status` - LiDAR health/status
- [ ] Create `robot_navigation` portmanteau tool:
  - `start_slam` - Start SLAM mapping
  - `stop_slam` - Stop mapping
  - `get_map` - Get current map (occupancy grid)
  - `save_map` - Save map to file
  - `load_map` - Load saved map
  - `localize` - Localize robot in map
  - `plan_path` - Plan path to goal
  - `navigate_to` - Navigate to waypoint (with obstacle avoidance)
  - `enable_obstacle_avoidance` - Enable reactive avoidance
- [ ] Create `robot_camera` portmanteau tool:
  - `get_stream_url` - RTSP stream URL
  - `get_snapshot` - Single frame capture
  - `enable_night_vision` - IR LED control
- [ ] Create `robot_patrol` portmanteau tool:
  - `start_patrol` - Begin patrol route (with LiDAR obstacle avoidance)
  - `stop_patrol` - Stop current patrol
  - `list_routes` - Available patrol routes
  - `create_route` - Define new route
  - `delete_route` - Remove route
- [ ] Write comprehensive unit tests
- [ ] Document Moorebot + YLIDAR integration

**Deliverables**:
- Full Moorebot Scout control via MCP
- ROS 1.4 (Melodic) integration via rosbridge_suite
- Pilot Labs Scout GitHub repo cloned and referenced
- YDLIDAR lightweight LiDAR integration (ROS 1 driver)
- SLAM and mapping capabilities
- Obstacle avoidance navigation
- Mock mode for development without hardware
- Complete test coverage
- Integration documentation

**ROS 1.4 Requirements**:
- Scout runs ROS 1.4 (Melodic) - NOT ROS 2
- Use `rosbridge_suite` (ROS 1), not `rosbridge_server` (ROS 2)
- Use `catkin_make` for building, not `colcon build`
- All drivers must be ROS 1 compatible

**References**: 
- `tapo-camera-mcp/src/tapo_camera_mcp/integrations/moorebot_client.py`
- Pilot Labs Scout GitHub: `https://github.com/Pilot-Labs-Dev/Scout-open-source`
- Local clone: `external/moorebot-scout-sdk/` (already exists)
- ROS 1.4 (Melodic) workspace: `roller_eye`

**YDLIDAR SuperLight Specifications** (Note: "SuperLight" may be model name or description - could be G2, X4, X4PRO, or other lightweight model):
- **Weight**: 95g (lightweight, perfect for Scout)
- **Type**: 2D LiDAR scanner
- **Range**: ~10-12m indoor / ~6m outdoor (typical for lightweight LiDAR)
- **Scan Rate**: 5-12Hz adjustable
- **Interface**: USB/UART
- **Power**: 5V, ~480mA (check Scout power capacity)
- **ROS Topic**: `/scan` (sensor_msgs/LaserScan)
- **Mounting**: Top of Scout (ensure unobstructed 360° view)

---

### Phase 5: Multi-Robot Coordination (Week 5)

**Goal**: Coordinate multiple robots

**Tasks**:
- [ ] Create `multi_robot` portmanteau tool:
  - `list_robots` - All connected robots
  - `get_robot_info` - Robot capabilities
  - `coordinate_patrol` - Multi-robot patrol
  - `assign_zone` - Zone-based assignment
  - `get_coordination_status` - Current coordination state
  - `stop_all` - Emergency stop all robots
- [ ] Implement zone-based coordination
- [ ] Add collision avoidance logic
- [ ] Create coordination algorithms
- [ ] Write integration tests
- [ ] Document coordination patterns

**Deliverables**:
- Multi-robot coordination
- Zone-based patrol
- Safety features

---

### Phase 6: Advanced Features (Week 6)

**Goal**: Polish and advanced capabilities

**Tasks**:
- [ ] Add persistent storage for routes/configs
- [ ] Implement robot health monitoring
- [ ] Add automation triggers (scheduled patrols)
- [ ] Create help system tool
- [ ] Add comprehensive error handling
- [ ] Performance optimization
- [ ] Complete API documentation
- [ ] Create deployment guide

**Deliverables**:
- Production-ready server
- Complete documentation
- Monitoring and health checks

---

## 🎨 Portmanteau Tool Design

### Tool 1: `robot_control`

**Purpose**: Unified robot movement and status control (**works for both bot and vbot**)

**Operations**:
- `get_status` - Get robot status (battery, position, state) - bot + vbot
- `move` - Control movement (linear/angular velocity) - bot + vbot
- `stop` - Emergency stop - bot + vbot
- `return_to_dock` - Return to charging dock (physical bot only)
- `stand` - Stand up (Unitree G1, physical bot only)
- `sit` - Sit down (Unitree G1, physical bot only)
- `walk` - Walking gait (Unitree, physical bot only)
- `sync_vbot` - Sync virtual bot with physical bot state (for testing)

**Parameters**:
- `robot_id` - Robot identifier (e.g., "scout_01", "vbot_scout_01")
- `action` - Operation to perform
- `linear` - Linear velocity (m/s)
- `angular` - Angular velocity (rad/s)
- `duration` - Movement duration (seconds)
- `robot_type` - "physical" or "virtual" (auto-detected from robot_id if prefixed)

**Implementation**:
- **Physical Bot**: Uses ROS 1.4 bridge (`/cmd_vel` topic)
- **Virtual Bot**: Uses `unity3d-mcp` or `vrchat-mcp` tools
- **Unified Interface**: Same tool works for both, routing based on robot_id

---

### Tool 2: `robot_sensors`

**Purpose**: Sensor data retrieval (excluding LiDAR - see `robot_lidar`)

**Operations**:
- `get_tof` - Time-of-flight distance (Moorebot)
- `get_imu` - IMU data (all robots)
- `get_light` - Light sensor (Moorebot)
- `get_camera` - Camera feed (all robots)
- `get_joint_states` - Joint positions (Unitree)
- `get_all` - All available sensors (excluding LiDAR)

**Parameters**:
- `robot_id` - Robot identifier
- `sensor_type` - Sensor to read
- `format` - Output format (json, raw)

---

### Tool 3: `robot_lidar` ⭐ NEW

**Purpose**: YDLIDAR SuperLight and other LiDAR operations

**Operations**:
- `get_scan` - Get latest LiDAR scan (full point cloud)
  - Returns: angles, ranges, intensities
  - Format: sensor_msgs/LaserScan compatible
- `get_range` - Get distance in specific direction
  - Parameters: angle (degrees), min_range, max_range
  - Returns: distance to nearest obstacle
- `get_obstacles` - Detect obstacles in current scan
  - Parameters: min_distance, cluster_threshold
  - Returns: List of obstacle positions (x, y, distance)
- `get_clearance` - Check clearance in direction
  - Parameters: direction (degrees), width (meters)
  - Returns: Clearance distance, passable (bool)
- `calibrate` - Calibrate LiDAR mounting offset
  - Parameters: known_distance, known_angle
  - Returns: Calibration offset values
- `get_status` - LiDAR health and status
  - Returns: Connected, scan_rate, error_count, temperature

**Parameters**:
- `robot_id` - Robot identifier (e.g., "scout_01")
- `action` - Operation to perform
- `angle` - Direction angle in degrees (0-360)
- `min_range` - Minimum range filter (meters)
- `max_range` - Maximum range filter (meters)
- `format` - Output format (json, raw, visualization)

**Example Usage**:
```python
# Get full scan
lidar_scan = robot_lidar(robot_id="scout_01", action="get_scan")

# Check if path is clear ahead
clearance = robot_lidar(
    robot_id="scout_01",
    action="get_clearance",
    angle=0,  # Forward
    width=0.5  # 50cm width
)

# Detect all obstacles
obstacles = robot_lidar(
    robot_id="scout_01",
    action="get_obstacles",
    min_distance=0.2  # Ignore obstacles closer than 20cm
)
```

---

### Tool 4: `robot_navigation` ⭐ NEW

**Purpose**: SLAM, mapping, and obstacle avoidance navigation

**Operations**:
- `start_slam` - Start SLAM mapping session
  - Parameters: map_name, resolution (meters)
  - Returns: SLAM session ID
- `stop_slam` - Stop mapping and finalize map
  - Parameters: session_id
  - Returns: Map saved confirmation
- `get_map` - Get current map (occupancy grid)
  - Parameters: format (image, grid, json)
  - Returns: Map data
- `save_map` - Save map to file
  - Parameters: map_name, file_path
  - Returns: Save confirmation
- `load_map` - Load saved map
  - Parameters: map_name, file_path
  - Returns: Load confirmation
- `localize` - Localize robot in known map
  - Parameters: map_name, initial_pose (optional)
  - Returns: Localized pose (x, y, theta)
- `plan_path` - Plan path to goal (A* or RRT)
  - Parameters: goal_x, goal_y, map_name
  - Returns: Path waypoints
- `navigate_to` - Navigate to waypoint with obstacle avoidance
  - Parameters: goal_x, goal_y, use_lidar (bool)
  - Returns: Navigation status, path taken
- `enable_obstacle_avoidance` - Enable reactive obstacle avoidance
  - Parameters: enabled (bool), safety_distance (meters)
  - Returns: Status confirmation

**Parameters**:
- `robot_id` - Robot identifier
- `action` - Operation to perform
- `map_name` - Map identifier
- `goal_x`, `goal_y` - Target coordinates (meters)
- `resolution` - Map resolution (meters per pixel)
- `safety_distance` - Minimum distance to obstacles (meters)

**Example Usage**:
```python
# Start mapping apartment
slam_id = robot_navigation(
    robot_id="scout_01",
    action="start_slam",
    map_name="stroheckgasse_apartment",
    resolution=0.05  # 5cm per pixel
)

# Navigate to kitchen with obstacle avoidance
result = robot_navigation(
    robot_id="scout_01",
    action="navigate_to",
    goal_x=1.0,
    goal_y=5.0,
    use_lidar=True  # Use YDLIDAR for real-time avoidance
)
```

---

### Tool 5: `robot_patrol`

**Purpose**: Patrol route management

**Operations**:
- `start_patrol` - Begin patrol route
- `stop_patrol` - Stop current patrol
- `list_routes` - Available routes
- `create_route` - Define new route
- `update_route` - Modify existing route
- `delete_route` - Remove route
- `get_patrol_status` - Current patrol state

**Parameters**:
- `robot_id` - Robot identifier
- `action` - Operation
- `route_name` - Route identifier
- `waypoints` - List of waypoints (x, y, room)

---

### Tool 6: `robot_camera`

**Purpose**: Camera and streaming control

**Operations**:
- `get_stream_url` - RTSP/WebRTC stream URL
- `get_snapshot` - Single frame capture
- `enable_night_vision` - IR LED control
- `set_resolution` - Camera resolution
- `start_recording` - Begin video recording
- `stop_recording` - End recording

**Parameters**:
- `robot_id` - Robot identifier
- `action` - Operation
- `format` - Image/video format
- `resolution` - Resolution setting

---

### Tool 7: `virtual_robotics`

**Purpose**: Virtual robot control (Unity/VRChat) - **Uses `unity3d-mcp` and `vrchat-mcp`**

**Operations**:
- `spawn_robot` - Spawn robot in Unity/VRChat scene (uses unity3d-mcp/vrchat-mcp)
- `move` - Control movement (unified interface, works for bot + vbot)
- `get_status` - Virtual state
- `get_lidar` - Virtual LiDAR (Unity physics raycast)
- `set_scale` - Scale robot size (for size testing)
- `load_environment` - Load Marble environment (uses unity3d-mcp)
- `test_navigation` - Pathfinding test
- `sync_with_physical` - Sync vbot state with physical bot (for testing)

**Parameters**:
- `robot_type` - Robot model (e.g., "scout", "go2", "g1")
- `action` - Operation
- `position` - Spawn position (x, y, z)
- `scale` - Size multiplier (for size testing)
- `environment` - Environment name (Marble-generated)
- `platform` - "unity" or "vrchat" (default: "unity")

**Integration Details**:
- **Unity**: Uses `unity3d-mcp` tools (`unity_spawn_object`, `unity_set_object_position`, etc.)
- **VRChat**: Uses `vrchat-mcp` tools (`vrchat_send_osc`, `vrchat_world_control`, etc.)
- **OSC**: Uses `osc-mcp` for communication (`osc_send_osc`)
- **3D Model**: References Scout 3D model from `SCOUT_3D_MODEL.md`

---

### Tool 8: `virtual_testing`

**Purpose**: Virtual testing and validation

**Operations**:
- `test_size_fit` - Test if robot fits
- `test_doorway` - Doorway clearance
- `test_patrol_route` - Route validation
- `compare_robots` - Side-by-side comparison
- `generate_report` - Test results

**Parameters**:
- `robot_type` - Robot to test
- `test_type` - Test to run
- `environment` - Test environment
- `scale` - Robot scale

---

### Tool 9: `multi_robot`

**Purpose**: Multi-robot coordination

**Operations**:
- `list_robots` - All connected robots
- `get_robot_info` - Robot capabilities
- `coordinate_patrol` - Multi-robot patrol
- `assign_zone` - Zone assignment
- `get_coordination_status` - Current state
- `stop_all` - Emergency stop all

**Parameters**:
- `action` - Operation
- `robot_ids` - List of robots
- `zones` - Zone definitions
- `strategy` - Coordination strategy

---

### Tool 10: `robot_manipulation` (Unitree G1 only)

**Purpose**: Arm and gripper control

**Operations**:
- `grasp` - Close gripper
- `release` - Open gripper
- `move_arm` - Arm control
- `get_arm_state` - Current arm position
- `pick_object` - Pick and place

**Parameters**:
- `robot_id` - Robot identifier
- `action` - Operation
- `position` - Target position
- `force` - Gripper force

---

### Tool 11: `drone_control` ✅ COMPLETED

**Purpose**: Core drone flight operations (PX4/ArduPilot compatible)

**Operations**:
- `get_status` - Get comprehensive drone status (battery, position, mode, health)
- `takeoff` - Take off to specified altitude (default 5m)
- `land` - Land at current position
- `move` - Move with specified velocity vectors
- `stop` - Emergency stop all movement
- `return_home` - Return to launch/home position (RTL)
- `set_mode` - Change flight mode (stabilize, alt_hold, loiter, auto, rtl)
- `arm` - Arm drone motors (required before flight)
- `disarm` - Disarm drone motors (safe state)
- `calibrate` - Perform sensor/accelerometer calibration
- `emergency_stop` - Immediate emergency stop and motor disarm

**Parameters**:
- `drone_id` - Unique drone identifier (e.g., "drone_01", "px4_quad_01")
- `velocity_x`, `velocity_y`, `velocity_z` - Velocity components (m/s)
- `yaw_rate` - Rotational velocity (rad/s)
- `altitude` - Target altitude (m) for takeoff
- `mode` - Flight mode name (e.g., "STABILIZE", "ALT_HOLD", "AUTO", "RTL")
- `calibration_type` - Type of calibration ("accelerometer", "compass", "level")

---

### Tool 12: `drone_streaming` ✅ COMPLETED

**Purpose**: Video streaming and recording (FPV, RTSP, WebRTC)

**Operations**:
- `start_fpv` - Start first-person-view video stream
- `stop_fpv` - Stop FPV video stream
- `get_stream_url` - Get URL for active video stream
- `set_stream_quality` - Adjust stream quality and bitrate
- `start_recording` - Begin video recording to file
- `stop_recording` - Stop video recording
- `take_snapshot` - Capture single still image

**Parameters**:
- `drone_id` - Unique drone identifier
- `quality` - Video quality preset ("480p", "720p", "1080p", "4K")
- `protocol` - Streaming protocol ("rtsp", "rtmp", "webrtc", "hls")
- `bitrate` - Target bitrate in kbps
- `filename` - Output filename for recording/snapshot

---

### Tool 13: `drone_navigation` ✅ COMPLETED

**Purpose**: GPS navigation, waypoints, and geofencing

**Operations**:
- `get_position` - Get current GPS position and altitude
- `set_waypoint` - Set single waypoint for navigation
- `follow_waypoints` - Execute multi-waypoint mission
- `clear_waypoints` - Clear all programmed waypoints
- `set_geofence` - Define geofence boundaries and restrictions
- `enable_follow_me` - Start following specified target
- `disable_follow_me` - Stop follow-me mode
- `set_home_location` - Set RTL/home position coordinates

**Parameters**:
- `drone_id` - Unique drone identifier
- `latitude`, `longitude`, `altitude` - GPS coordinates and altitude
- `waypoints` - List of waypoint dictionaries
- `fence_points` - List of geofence boundary points
- `max_altitude` - Maximum allowed altitude in meters
- `target_id` - Identifier of target to follow

---

### Tool 14: `drone_flight_control` ✅ COMPLETED

**Purpose**: Advanced flight modes, missions, and parameter tuning

**Operations**:
- `set_flight_mode` - Change to specific flight mode (LOITER, AUTO, GUIDED, etc.)
- `get_flight_modes` - List all available flight modes
- `start_mission` - Begin execution of uploaded mission plan
- `pause_mission` - Temporarily halt mission execution
- `resume_mission` - Continue paused mission
- `abort_mission` - Immediately terminate mission
- `upload_mission` - Send mission plan to drone
- `download_mission` - Retrieve current mission from drone
- `set_parameter` - Modify drone parameter value
- `get_parameters` - Retrieve all drone parameters

**Parameters**:
- `drone_id` - Unique drone identifier
- `mode` - Flight mode name for set_flight_mode
- `mission_id` - Identifier for mission operations
- `mission_plan` - Complete mission plan dictionary with waypoints and commands
- `param_name` - Parameter name for set_parameter
- `param_value` - New value for the specified parameter

---

## 🔧 Technical Architecture

### MCP Server Composition

**Main Server Structure**:
```python
from fastmcp import FastMCP, Client
from osc_mcp.server import server as osc_mcp_server
from unity3d_mcp.server import server as unity3d_mcp_server
from vrchat_mcp.server import server as vrchat_mcp_server

# Create main robotics server
robotics = FastMCP(name="Robotics-MCP")

# Mount existing MCP servers (live proxy mode)
robotics.mount(osc_mcp_server, prefix="osc", as_proxy=True)
robotics.mount(unity3d_mcp_server, prefix="unity", as_proxy=True)
robotics.mount(vrchat_mcp_server, prefix="vrchat", as_proxy=True)

# Now tools are available as:
# - osc_send_osc (from osc-mcp)
# - unity_spawn_object (from unity3d-mcp)
# - vrchat_send_osc (from vrchat-mcp)
```

**Unified Bot + Vbot Control Pattern**:
```python
@robotics.tool
async def robot_control(robot_id: str, action: str, **params):
    """Unified robot control (works for both physical bot and virtual bot)."""
    robot = state_manager.get_robot(robot_id)
    
    if robot is None:
        return {"error": f"Robot {robot_id} not found"}
    
    # Route to appropriate handler based on robot type
    if robot.type == "physical":
        # Physical robot: Use ROS bridge
        return await physical_robot_handler(robot, action, **params)
    elif robot.type == "virtual":
        # Virtual robot: Use unity3d-mcp or vrchat-mcp
        return await virtual_robot_handler(robot, action, **params)
    else:
        return {"error": f"Unknown robot type: {robot.type}"}

async def physical_robot_handler(robot, action, **params):
    """Handle physical robot commands via ROS 1.4."""
    if action == "move":
        # Use ROS bridge to publish /cmd_vel
        await ros_bridge.publish_cmd_vel(
            robot.id,
            linear=params.get("linear", 0.0),
            angular=params.get("angular", 0.0)
        )
    # ... other physical robot operations

async def virtual_robot_handler(robot, action, **params):
    """Handle virtual robot commands via unity3d-mcp, vrchat-mcp, or avatar-mcp."""
    async with Client(robotics) as client:
        if robot.platform == "unity":
            # Use unity3d-mcp for scene control, avatar-mcp for movement
            if action == "move":
                # Use avatar-mcp movement tools for smooth locomotion
                await client.call_tool("avatar_movement_walk",
                    avatar_id=robot.id,
                    direction="forward",
                    speed=params.get("linear", 0.0))
            elif action == "turn":
                await client.call_tool("avatar_movement_turn",
                    avatar_id=robot.id,
                    angle=params.get("angular", 0.0))
        elif robot.platform == "vrchat":
            # Use vrchat-mcp + osc-mcp
            if action == "move":
                await client.call_tool("vrchat_send_osc",
                    address=f"/robot/{robot.id}/move",
                    values=[params.get("linear", 0.0), params.get("angular", 0.0)])
    # ... other virtual robot operations
```

### Client Architecture

```python
# Base client interface
class RobotClient(ABC):
    @abstractmethod
    async def connect(self) -> Dict[str, Any]:
        """Connect to robot"""
    
    @abstractmethod
    async def get_status(self) -> Dict[str, Any]:
        """Get robot status"""
    
    @abstractmethod
    async def move(self, linear: float, angular: float) -> Dict[str, Any]:
        """Control movement"""

# Moorebot implementation
class MoorebotScoutClient(RobotClient):
    def __init__(self, ip_address: str, port: int = 9090, mock_mode: bool = False):
        self.ip_address = ip_address
        self.port = port
        self.mock_mode = mock_mode
        self.ros_bridge = None
        self.lidar_client = None  # YLIDAR SuperLight client
    
    async def connect(self):
        if self.mock_mode:
            return {"success": True, "mode": "mock"}
        # Connect to ROS 1 rosbridge WebSocket (rosbridge_suite)
        # Scout runs ROS 1.4 (Melodic), uses rosbridge_suite (not rosbridge_server)
        self.ros_bridge = await connect_rosbridge(
            f"ws://{self.ip_address}:{self.port}",
            ros_version="1"  # ROS 1.4 Melodic
        )
        # Initialize YDLIDAR client if enabled
        if self.config.lidar.enabled:
            self.lidar_client = YDLIDARClient(self.ros_bridge, topic="/scan", ros_version="1")
        return {"success": True, "mode": "real", "ros_version": "1.4 (Melodic)"}

# YDLIDAR SuperLight implementation (ROS 1.4)
class YDLIDARClient:
    def __init__(self, ros_bridge, topic: str = "/scan", ros_version: str = "1"):
        self.ros_bridge = ros_bridge
        self.topic = topic
        self.ros_version = ros_version  # "1" for ROS 1.4 Melodic
        self.latest_scan = None
        self.subscription = None
    
    async def subscribe(self):
        """Subscribe to LiDAR scan topic (ROS 1.4)"""
        # ROS 1 uses sensor_msgs/LaserScan (same message type as ROS 2)
        self.subscription = await self.ros_bridge.subscribe(
            self.topic,
            "sensor_msgs/LaserScan",  # ROS 1 message type
            self._scan_callback
        )
    
    def _scan_callback(self, msg):
        """Callback for incoming scan data"""
        self.latest_scan = {
            "ranges": msg.ranges,
            "angles": self._calculate_angles(msg),
            "intensities": msg.intensities,
            "timestamp": msg.header.stamp,
            "range_min": msg.range_min,
            "range_max": msg.range_max
        }
    
    def _calculate_angles(self, msg):
        """Calculate angle for each range measurement"""
        angles = []
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        for i in range(len(msg.ranges)):
            angles.append(angle_min + i * angle_increment)
        return angles
    
    async def get_scan(self) -> Dict[str, Any]:
        """Get latest LiDAR scan"""
        if self.latest_scan is None:
            return {"error": "No scan data available"}
        return self.latest_scan
    
    async def get_range(self, angle_deg: float) -> float:
        """Get distance at specific angle"""
        if self.latest_scan is None:
            return None
        # Find closest angle in scan
        angle_rad = math.radians(angle_deg)
        closest_idx = min(
            range(len(self.latest_scan["angles"])),
            key=lambda i: abs(self.latest_scan["angles"][i] - angle_rad)
        )
        return self.latest_scan["ranges"][closest_idx]

# Unitree implementation
class UnitreeClient(RobotClient):
    def __init__(self, robot_type: str, mock_mode: bool = False):
        self.robot_type = robot_type  # "go2" or "g1"
        self.mock_mode = mock_mode
        self.sdk = None
    
    async def connect(self):
        if self.mock_mode:
            return {"success": True, "mode": "mock"}
        # Initialize Unitree SDK
        self.sdk = UnitreeSDK2(robot_type=self.robot_type)
        return {"success": True, "mode": "real"}

# Virtual implementation
class VirtualRobotClient(RobotClient):
    def __init__(self, unity_host: str = "localhost", unity_port: int = 8080):
        self.uri = f"ws://{unity_host}:{unity_port}/robot"
        self.websocket = None
    
    async def connect(self):
        self.websocket = await websockets.connect(self.uri)
        return {"success": True, "mode": "virtual"}
```

### State Management

```python
# State manager for robot connections
class RobotStateManager:
    def __init__(self):
        self.robots: Dict[str, RobotClient] = {}
        self.config = load_config()
    
    def register_robot(self, robot_id: str, client: RobotClient):
        """Register a robot client"""
        self.robots[robot_id] = client
    
    def get_robot(self, robot_id: str) -> RobotClient:
        """Get robot client by ID"""
        if robot_id not in self.robots:
            raise ValueError(f"Robot {robot_id} not found")
        return self.robots[robot_id]
    
    async def initialize_robots(self):
        """Initialize all configured robots"""
        for robot_config in self.config.robots:
            client = create_client(robot_config)
            await client.connect()
            self.register_robot(robot_config.id, client)
```

---

## 📦 Dependencies

```toml
[project]
name = "robotics-mcp"
version = "0.1.0"
description = "Unified robotics control via MCP (bot + vbot)"
requires-python = ">=3.10"

dependencies = [
    "fastmcp>=2.13.0,<2.14.0",
    "pydantic>=2.0.0",
    "pyyaml>=6.0",
    "websockets>=12.0",
    "aiohttp>=3.9.0",
    "rosbridge-library>=0.11.0",  # ROS 1 bridge client (for rosbridge_suite)
    "unitree-sdk2>=1.0.0",  # Unitree SDK (if available)
    "numpy>=1.24.0",  # For sensor data processing
    "scipy>=1.10.0",  # For SLAM and path planning
    "opencv-python>=4.8.0",  # For map visualization
    "matplotlib>=3.7.0",  # For map plotting
]

# MCP Server Dependencies (for composition)
# These are imported at runtime via mount(), not direct dependencies
# - osc-mcp (for OSC communication)
# - unity3d-mcp (for Unity virtual robotics)
# - vrchat-mcp (for VRChat world control)
```

**Note**: `osc-mcp`, `unity3d-mcp`, and `vrchat-mcp` are mounted at runtime via `mount()`, not installed as direct dependencies. They must be available in the Python path or installed separately.

---

## ⚙️ Configuration

```yaml
# config.yaml.example
robotics:
  # Moorebot Scout
  moorebot_scout:
    enabled: true
    robot_id: "scout_01"
    ip_address: "192.168.1.100"
    port: 9090
    mock_mode: true  # Set false when hardware available
    
    # YDLIDAR SuperLight (95g) Configuration
    # Note: "SuperLight" may be model name or description
    # Could be: G2, X4, X4PRO, or other lightweight YDLIDAR model
    lidar:
      enabled: true
      type: "ydlidar_superlight"  # or "ydlidar_g2", "ydlidar_x4", etc.
      weight: 95  # grams
      ros_topic: "/scan"
      scan_rate: 10  # Hz
      range_min: 0.1  # meters
      range_max: 12.0  # meters (indoor)
      mounting:
        position: "top"  # Mounted on top of Scout
        height_offset: 0.08  # 8cm above Scout body
        rotation_offset: 0  # degrees (calibrate if needed)
      power:
        voltage: 5  # V
        current: 0.48  # A (480mA)
        check_scout_capacity: true  # Verify Scout can supply power
    
    location:
      home_base: {x: 0.0, y: 0.0}
      rooms:
        living_room: {x_min: 0, x_max: 5, y_min: 0, y_max: 4}
        bedroom: {x_min: 5, x_max: 8, y_min: 0, y_max: 3}
    
    patrols:
      default:
        - {x: 2.0, y: 2.0, room: "living_room"}
        - {x: 6.0, y: 1.5, room: "bedroom"}
    
    # Navigation and SLAM
    navigation:
      slam_enabled: true
      obstacle_avoidance: true
      safety_distance: 0.3  # meters (30cm minimum clearance)
      map_resolution: 0.05  # 5cm per pixel
      maps_directory: "maps/"
  
  # Unitree Go2
  unitree_go2:
    enabled: false
    robot_id: "go2_01"
    mock_mode: true
    # SDK configuration
  
  # Unitree G1
  unitree_g1:
    enabled: false
    robot_id: "g1_01"
    mock_mode: true
    # SDK configuration
  
  # Virtual Robotics (Vbot)
  virtual:
    enabled: true
    platform: "unity"  # "unity" or "vrchat"
    unity:
      host: "localhost"
      port: 8080
      use_unity3d_mcp: true  # Use mounted unity3d-mcp
    vrchat:
      enabled: false
      use_vrchat_mcp: true  # Use mounted vrchat-mcp
      osc_port: 9000  # VRChat OSC port
    environments:
      stroheckgasse_apartment:
        path: "environments/stroheckgasse.ply"
        type: "marble_export"
    # Virtual robot instances
    robots:
      vbot_scout_01:
        type: "scout"
        platform: "unity"
        model_path: "assets/models/scout/scout_base.fbx"
        scale: 1.0  # 1× = 11.5cm (actual Scout size)
        position: {x: 0.0, y: 0.0, z: 0.0}
      vbot_scout_go2_size:
        type: "scout"
        platform: "unity"
        model_path: "assets/models/scout/scout_base.fbx"
        scale: 6.1  # Scaled to Unitree Go2 size (70cm) for testing
        position: {x: 0.0, y: 0.0, z: 0.0}
  
  # Multi-robot coordination (bot + vbot)
  coordination:
    enabled: false
    strategy: "zone_based"  # or "time_based", "priority_based"
    zones:
      zone_1:
        robots: ["scout_01", "vbot_scout_01"]  # Physical + virtual
        area: {x_min: 0, x_max: 5, y_min: 0, y_max: 4}
  
  # MCP Server Integration
  mcp_integration:
    osc_mcp:
      enabled: true
      prefix: "osc"
      use_proxy: true
    unity3d_mcp:
      enabled: true
      prefix: "unity"
      use_proxy: true
    vrchat_mcp:
      enabled: true
      prefix: "vrchat"
      use_proxy: true
```

---

## 🤖 Pilot Labs Scout Repository

### GitHub Repository

**Official Scout Open Source SDK**:
- **URL**: `https://github.com/Pilot-Labs-Dev/Scout-open-source`
- **ROS Version**: ROS 1.4 (Melodic)
- **Workspace**: `roller_eye`
- **Purpose**: Complete ROS workspace for Scout robot

### Repository Structure

```
Scout-open-source/
├── roller_eye/              # Main ROS 1.4 workspace
│   ├── src/                 # Source code
│   │   ├── sensor/          # Sensor nodes (ToF, IMU, light)
│   │   ├── motor/           # Motor control node
│   │   ├── core/            # CoreNode (camera/audio)
│   │   └── supervisor/      # SupervisorNode
│   ├── msg/                 # Custom ROS messages
│   ├── srv/                 # Custom ROS services
│   └── launch/              # Launch files
└── docs/                    # Documentation
```

### Cloning and Setup

**Note**: Repository already cloned at `external/moorebot-scout-sdk/`

```bash
# Repository location (already exists)
cd D:\Dev\repos\external\moorebot-scout-sdk

# Review ROS 1.4 workspace structure
cd roller_eye
catkin_make  # Build ROS 1 workspace (if needed)
source devel/setup.bash

# Key ROS topics to reference:
# - /CoreNode/h264 (video)
# - /CoreNode/aac (audio)
# - /SensorNode/tof (ToF sensor)
# - /SensorNode/imu (IMU)
# - /SensorNode/light (light sensor)
# - /cmd_vel (movement commands)
# - /start_patrol (patrol service)
# - /return_home (docking service)
```

### ROS 1.4 Integration Notes

**Critical**: Scout runs ROS 1.4 (Melodic), NOT ROS 2!

**ROS 1.4 Requirements**:
- Use `catkin_make` (not `colcon build`)
- Use `roslaunch` (ROS 1 launch files)
- Use `rosbridge_suite` (ROS 1), not `rosbridge_server` (ROS 2)
- Message types: `sensor_msgs/LaserScan`, `geometry_msgs/Twist`, etc. (same as ROS 2, but ROS 1 implementation)

**rosbridge_suite Setup** (ROS 1):
```bash
# Install rosbridge_suite for ROS 1 Melodic
sudo apt install ros-melodic-rosbridge-suite

# Launch on Scout (or development machine)
roslaunch rosbridge_server rosbridge_websocket.launch port:=9090
```

**Integration with robotics-mcp**:
- Reference Scout repo for message types and service definitions
- Use ROS 1 compatible drivers (YDLIDAR ROS 1 driver)
- Ensure all ROS communication uses ROS 1 protocols

---

## 🔍 YDLIDAR SuperLight Integration Details

### Hardware Specifications

**YDLIDAR SuperLight (95g)** (Note: "SuperLight" may be a specific model name or a description - could be G2, X4, X4PRO, or another lightweight YDLIDAR model):
- **Weight**: 95g (lightweight, perfect for Scout's 350g body)
- **Type**: 2D LiDAR scanner (360° horizontal scan)
- **Range**: 
  - Indoor: ~10-12m typical
  - Outdoor: ~6m (reduced due to ambient light)
- **Scan Rate**: 5-12Hz adjustable (10Hz recommended)
- **Angular Resolution**: ~0.5-1.0° (depends on model)
- **Interface**: USB/UART
- **Power**: 5V, ~480mA (2.4W)
- **ROS Topic**: `/scan` (sensor_msgs/LaserScan message type)

### Mounting on Moorebot Scout

**Physical Mounting**:
- **Position**: Top center of Scout body
- **Height**: ~8cm above Scout base (ensures 360° unobstructed view)
- **Orientation**: Forward = 0° (calibrate if needed)
- **Mounting Method**: 
  - 3D printed bracket (design in `hardware/mounting/`)
  - Screw mount or adhesive (ensure secure attachment)
  - Consider vibration damping for smooth scans

**Power Considerations**:
- Scout power system: Check if 5V/500mA available
- May need USB power bank if Scout can't supply
- Total Scout + LiDAR power: ~350g + 95g = 445g (still lightweight!)

**Cable Management**:
- USB cable routing to avoid wheel interference
- Secure cable to prevent tangling during movement
- Consider flexible cable for mecanum wheel movement

### ROS Integration

**Driver Installation** (on Scout - ROS 1.4 Melodic):
```bash
# Install YDLIDAR ROS 1 driver (for ROS Melodic)
cd ~/catkin_ws/src
git clone https://github.com/YDLIDAR/ydlidar_ros_driver.git
cd ~/catkin_ws
catkin_make  # ROS 1 uses catkin_make (not colcon)
source devel/setup.bash

# Configure serial port permissions
cd src/ydlidar_ros_driver/startup
sudo chmod 777 ./*
sudo sh initenv.sh

# Launch LiDAR driver (ROS 1)
roslaunch ydlidar_ros_driver lidar_view.launch
```

**Note**: Scout runs ROS 1.4 (Melodic), NOT ROS 2. Use ROS 1 drivers and tools:
- `catkin_make` (not `colcon build`)
- `roslaunch` (ROS 1 launch files)
- `rosbridge_suite` (ROS 1 version, not rosbridge_server for ROS 2)

**ROS 1 Topics** (Scout runs ROS 1.4 Melodic):
- `/scan` - LaserScan messages (sensor_msgs/LaserScan) - ROS 1 message type
  - `ranges[]` - Distance measurements (meters)
  - `angle_min` - Minimum scan angle (radians)
  - `angle_max` - Maximum scan angle (radians)
  - `angle_increment` - Angular resolution (radians)
  - `range_min` - Minimum range (meters)
  - `range_max` - Maximum range (meters)
  - `intensities[]` - Reflectance values (optional)

**ROS 1 Launch File Configuration**:
```xml
<!-- ydlidar_superlight.launch (ROS 1 Melodic) -->
<launch>
  <node name="ydlidar_node" pkg="ydlidar_ros_driver" type="ydlidar_ros_driver_node" output="screen">
    <param name="port" value="/dev/ttyUSB0"/>
    <param name="baudrate" value="115200"/>
    <param name="frame_id" value="laser_frame"/>
    <param name="angle_min" value="-180"/>
    <param name="angle_max" value="180"/>
    <param name="range_min" value="0.1"/>
    <param name="range_max" value="12.0"/>
    <param name="frequency" value="10.0"/>
  </node>
  
  <!-- Optional: Transform from laser_frame to base_link (Scout body) -->
  <node pkg="tf" type="static_transform_publisher" name="laser_to_base"
        args="0.0 0.0 0.08 0 0 0 base_link laser_frame 100"/>
</launch>
```

**ROS 1 Bridge Setup** (rosbridge_suite for ROS 1):
```bash
# Install rosbridge_suite for ROS 1 Melodic
sudo apt install ros-melodic-rosbridge-suite

# Launch rosbridge WebSocket server (on Scout)
roslaunch rosbridge_server rosbridge_websocket.launch port:=9090
```

**Important**: Scout uses ROS 1.4 (Melodic), not ROS 2. All drivers and tools must be ROS 1 compatible.

### Calibration

**Mounting Offset Calibration**:
1. Place Scout at known position (e.g., 1m from wall)
2. Point LiDAR at wall (0° forward)
3. Measure actual distance vs LiDAR reading
4. Calculate offset: `offset = actual_distance - lidar_reading`
5. Store offset in configuration

**Angle Calibration**:
1. Point Scout forward (0°)
2. Rotate to face known landmark
3. Compare LiDAR angle to actual heading
4. Calculate rotation offset
5. Store in configuration

### Use Cases with YDLIDAR

**1. Obstacle Avoidance**:
- Real-time obstacle detection during movement
- Reactive navigation around furniture
- Safe patrol routes

**2. SLAM Mapping**:
- Create apartment map using LiDAR + odometry
- Store map for future navigation
- Localize robot in known map

**3. Doorway Detection**:
- Detect doorways for navigation
- Measure doorway width
- Plan routes through doorways

**4. Object Detection**:
- Detect large objects (furniture, boxes)
- Track moving objects (Benny the dog!)
- Monitor room occupancy

**5. Precision Navigation**:
- Navigate to exact positions
- Follow walls at precise distance
- Dock with high accuracy

### Integration with Existing Sensors

**Sensor Fusion**:
- **LiDAR + ToF**: LiDAR for wide-area, ToF for close-range precision
- **LiDAR + IMU**: LiDAR for mapping, IMU for orientation
- **LiDAR + Camera**: LiDAR for depth, camera for object recognition

**Complementary Capabilities**:
- **ToF sensor**: Single-point, close-range (3m max)
- **LiDAR**: 360° scan, longer range (12m), obstacle detection
- **Camera**: Visual information, object recognition
- **IMU**: Orientation, movement tracking

---

## 🧪 Testing Strategy

### Unit Tests
- Mock all robot clients
- Test each portmanteau tool operation
- Test error handling
- Test state management

### Integration Tests
- Test with mock ROS bridge
- Test with mock Unity WebSocket
- Test multi-robot coordination
- Test patrol route execution

### Hardware Tests (When Available)
- Real Moorebot Scout (XMas 2025)
- Real Unitree robots (if purchased)
- Virtual robotics validation

---

## 📚 Documentation Requirements

1. **README.md** - Quick start, installation, basic usage
2. **ARCHITECTURE.md** - System design, client architecture
3. **MOOREBOT_INTEGRATION.md** - Complete Moorebot guide
4. **UNITREE_INTEGRATION.md** - Unitree setup and usage
5. **VIRTUAL_ROBOTICS.md** - Virtual testing workflow
6. **API_REFERENCE.md** - Complete tool reference
7. **DEPLOYMENT.md** - Production deployment guide

---

## 🚀 Success Criteria

### Phase 1 Complete
- ✅ Server starts without errors
- ✅ Configuration system works
- ✅ Mock mode functional
- ✅ Basic tests pass

### Phase 2 Complete
- ✅ Moorebot Scout fully controllable
- ✅ All sensor data accessible
- ✅ Patrol routes working
- ✅ Camera streaming functional

### Phase 3 Complete
- ✅ Unitree Go2/G1 support
- ✅ Locomotion control
- ✅ Sensor integration
- ✅ Arm control (G1)

### Phase 4 Complete
- ✅ Virtual robot control
- ✅ Size testing capabilities
- ✅ Marble integration
- ✅ VRChat world control

### Phase 5 Complete
- ✅ Multi-robot coordination
- ✅ Zone-based patrol
- ✅ Safety features

### Phase 6 Complete
- ✅ Production-ready
- ✅ Complete documentation
- ✅ Monitoring and health checks
- ✅ Deployment guide

### Drone Integration Complete ✅
- ✅ 4 Portmanteau drone tools implemented (`drone_control`, `drone_streaming`, `drone_navigation`, `drone_flight_control`)
- ✅ PX4/ArduPilot MAVLink protocol support
- ✅ Core flight operations (takeoff, land, move, status)
- ✅ Video streaming (FPV, RTSP, WebRTC)
- ✅ GPS navigation and waypoints
- ✅ Advanced flight control (missions, parameters)
- ✅ Type safety and Pydantic schema validation
- ✅ Comprehensive documentation and examples

---

## 🔗 Integration Points

### Existing MCP Servers (Mounted via `mount()`)

**Primary Integrations**:
- **`osc-mcp`**: OSC protocol for robot communication
  - Used for: Robot control via OSC (if supported), VRChat/Unity communication
  - Mounted as: `osc_*` tools (e.g., `osc_send_osc`)
- **`unity3d-mcp`**: Unity automation integration + **World Labs Marble/Chisel**
  - Used for: Spawning virtual robots, scene control, physics, **Marble/Chisel environment loading**
  - Mounted as: `unity_*` tools (e.g., `unity_spawn_object`, `unity_import_marble_world`)
  - **Key Tool**: `unity_import_marble_world` - Import splats/meshes from Marble/Chisel
- **`vrchat-mcp`**: VRChat world control
  - Used for: Spawning robots in VRChat, world interactions, OSC avatar control
  - Mounted as: `vrchat_*` tools (e.g., `vrchat_send_osc`, `vrchat_world_control`)
- **`avatar-mcp`**: Avatar/robot movement and pose control
  - Used for: Robot locomotion (`movement.walk`, `movement.run`, `movement.turn`), pose control
  - Mounted as: `avatar_*` tools (e.g., `avatar_movement_walk`, `avatar_interactive_pose_control`)
  - **Key Tools**: Movement control, pose manipulation for robot behaviors

**Secondary Integrations** (Optional):
- **`blender-mcp`**: 3D model processing
  - Used for: Creating/editing Scout 3D model (if needed)
  - Reference: `mcp-central-docs/docs/robotics/SCOUT_3D_MODEL.md`
- **`tapo-camera-mcp`**: Reference Moorebot implementation
  - Used for: Reference code patterns (not mounted, just referenced)

### External Systems

**Physical Robots**:
- **ROS 1.4 (Melodic)**: Robot Operating System (Scout runs ROS 1.4, NOT ROS 2!)
- **rosbridge_suite**: WebSocket bridge to ROS 1.4 (NOT rosbridge_server for ROS 2)
- **Pilot Labs Scout SDK**: `external/moorebot-scout-sdk/roller_eye/` (ROS 1.4 workspace)

**Virtual Robotics**:
- **Unity3D**: Virtual robotics simulation (controlled via `unity3d-mcp`)
- **VRChat**: Social testing platform (controlled via `vrchat-mcp`)
- **Resonite**: Alternative VR platform (similar to VRChat)
- **World Labs Marble/Chisel**: Environment generation (Gaussian splats, meshes)
  - **Marble**: AI-generated 3D environments (splats/meshes)
  - **Chisel**: Edit and optimize Marble environments
  - **Integration**: Use `unity3d-mcp` `import_marble_world` tool
  - **Workflow**: Generate apartment with Marble → Edit with Chisel → Import to Unity → Test robot

### MCP Server Composition Architecture

```python
from fastmcp import FastMCP, Client

# Main robotics server
robotics = FastMCP(name="Robotics-MCP")

# Mount existing MCP servers
robotics.mount(osc_mcp, prefix="osc", as_proxy=True)
robotics.mount(unity3d_mcp, prefix="unity", as_proxy=True)  # Includes World Labs Marble/Chisel
robotics.mount(vrchat_mcp, prefix="vrchat", as_proxy=True)
robotics.mount(avatar_mcp, prefix="avatar", as_proxy=True)  # Robot movement/pose control

# Unified robot control (bot + vbot)
@robotics.tool
async def robot_move(robot_id: str, linear: float, angular: float):
    """Move robot (works for both physical bot and virtual bot)."""
    robot = get_robot(robot_id)
    
    if robot.type == "physical":
        # Use ROS bridge for physical robot
        await ros_bridge.publish_cmd_vel(robot_id, linear, angular)
    elif robot.type == "virtual":
        # Use unity3d-mcp for virtual robot
        async with Client(robotics) as client:
            await client.call_tool("unity_set_object_position",
                object_id=robot_id, x=linear, y=angular)
    
    return {"status": "moved", "robot_id": robot_id}
```

### Cross-Server Workflows

**Example: Spawn Virtual Robot in Unity with Marble Environment**
```python
@robotics.tool
async def spawn_vbot_in_marble_apartment(robot_type: str, marble_export_path: str):
    """Spawn virtual robot in Unity with World Labs Marble apartment."""
    async with Client(robotics) as client:
        # Import Marble/Chisel environment
        await client.call_tool("unity_import_marble_world",
            source_path=marble_export_path,
            include_colliders=True)
        
        # Load Scout 3D model
        await client.call_tool("unity_load_model",
            path="assets/models/scout/scout_base.fbx")
        
        # Spawn in scene
        vbot_id = await client.call_tool("unity_spawn_object",
            model="scout_base", position={"x": 0, "y": 0, "z": 0})
        
        # Register as virtual robot
        register_virtual_robot(vbot_id, robot_type, platform="unity")
        
        return {"vbot_id": vbot_id, "status": "spawned", "environment": "marble_apartment"}
```

**Example: Control Virtual Robot Movement via avatar-mcp**
```python
@robotics.tool
async def vbot_move(robot_id: str, linear: float, angular: float):
    """Move virtual robot using avatar-mcp movement tools."""
    async with Client(robotics) as client:
        # Use avatar-mcp for smooth locomotion
        if linear > 0:
            await client.call_tool("avatar_movement_walk",
                avatar_id=robot_id,
                direction="forward",
                speed=linear)
        if angular != 0:
            await client.call_tool("avatar_movement_turn",
                avatar_id=robot_id,
                angle=angular)
        
        return {"status": "moved", "robot_id": robot_id}
```

**Example: Control Robot in VRChat World**
```python
@robotics.tool
async def control_robot_in_vrchat(robot_id: str, action: str):
    """Control robot in VRChat using vrchat-mcp and osc-mcp."""
    async with Client(robotics) as client:
        # Send OSC command to VRChat
        await client.call_tool("vrchat_send_osc",
            address=f"/robot/{robot_id}/{action}",
            values=[1.0])
        
        return {"status": "controlled", "robot_id": robot_id}
```

---

## 📅 Timeline

| Phase | Duration | Start | End |
|-------|----------|-------|-----|
| Phase 1: Foundation | 1 week | Week 1 | Week 1 |
| Phase 2: Virtual Robotics (Vbot) ⚡ | 1 week | Week 2 | Week 2 |
| Phase 3: Unitree | 1 week | Week 3 | Week 3 |
| Phase 4: Moorebot Physical | 1 week | After XMas | After XMas |
| Phase 5: Multi-Robot | 1 week | Week 5 | Week 5 |
| Phase 6: Advanced | 1 week | Week 6 | Week 6 |
| **Total** | **6 weeks** | | |

**Note**: Phase 2 (Virtual Robotics) is prioritized since physical hardware isn't available yet. This allows full testing and development before hardware arrives.

---

## 🎯 Next Steps

1. **Review this plan** - Validate approach and priorities
2. **Set up project structure** - Initialize repository
3. **Begin Phase 1** - Foundation and basic server
4. **Iterate based on feedback** - Adjust as needed

---

**Status**: Drone Integration Complete - Ready for Physical Hardware Testing
**Priority**: High (complements existing robotics work)
**Dependencies**: FastMCP 2.13+, ROS bridge, Unity WebSocket server, PX4/ArduPilot firmware
**✅ Completed**: Core drone support with 4 portmanteau tools and comprehensive MCP integration

