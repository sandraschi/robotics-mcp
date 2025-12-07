# Tool Gaps Analysis - Robotics MCP

**Date**: 2025-12-07  
**Status**: Gap Analysis

## Current Tool Coverage

### ✅ Implemented Tools
1. **robot_model** - Model operations (create, import, export, convert)
2. **vbot_crud** - Virtual robot CRUD (create, read, update, delete, list)
3. **robotics_system** - System management (help, status, list_robots)
4. **robot_control** - Unified bot/vbot control (move, stop, get_status, sync_vbot)
5. **virtual_robotics** - Virtual robotics operations (spawn, move, get_lidar, load_environment, test_navigation, sync_with_physical)

## Identified Gaps

### 🔴 Critical Gaps (High Priority)

#### 1. **Robot Animation & Behavior**
**Status**: ❌ Missing  
**Impact**: High - Robots are static, no visual feedback  
**Needed Tools**:
- `robot_animation` portmanteau:
  - `animate_wheels` - Rotate wheels during movement (Scout mecanum wheels)
  - `animate_movement` - Play movement animations (walk, turn, etc.)
  - `set_pose` - Set robot pose (sitting, standing, etc. for Unitree)
  - `play_animation` - Play custom animations
  - `stop_animation` - Stop current animation

**Implementation**:
- Unity: Use Unity Animator/Animation system via `execute_unity_method`
- VRChat: Use avatar-mcp pose/animation tools
- Physical: ROS action servers for animations

#### 2. **Camera & Visual Feed**
**Status**: ❌ Missing (mentioned in PLAN but not implemented)  
**Impact**: High - No visual feedback from robots  
**Needed Tools**:
- `robot_camera` portmanteau:
  - `get_camera_feed` - Get live camera feed (physical Scout camera)
  - `get_virtual_camera` - Get Unity camera view from robot perspective
  - `set_camera_angle` - Adjust camera angle
  - `capture_image` - Capture still image
  - `start_streaming` - Start video stream
  - `stop_streaming` - Stop video stream

**Implementation**:
- Physical: ROS image topics (sensor_msgs/Image)
- Unity: Unity Camera component via `execute_unity_method`
- Streaming: WebRTC or MJPEG stream

#### 3. **Path Planning & Navigation**
**Status**: ⚠️ Stub (TODO in `test_navigation`)  
**Impact**: High - No actual navigation capability  
**Needed Tools**:
- `robot_navigation` portmanteau:
  - `plan_path` - Plan path from A to B (A* or RRT)
  - `follow_path` - Execute planned path
  - `set_waypoint` - Set navigation waypoint
  - `clear_waypoints` - Clear waypoint list
  - `get_path_status` - Check path execution status
  - `avoid_obstacle` - Dynamic obstacle avoidance

**Implementation**:
- Physical: ROS navigation stack (move_base, amcl)
- Unity: Unity NavMesh pathfinding via `execute_unity_method`
- Virtual: Unity NavMeshAgent component

#### 4. **Multi-Robot Coordination**
**Status**: ❌ Missing (mentioned in PLAN but no tools)  
**Impact**: Medium - Can't coordinate multiple robots  
**Needed Tools**:
- `multi_robot` portmanteau:
  - `coordinate_movement` - Move multiple robots together
  - `assign_roles` - Assign roles to robots (leader, follower, etc.)
  - `formation_control` - Control robot formations
  - `collision_avoidance` - Inter-robot collision avoidance
  - `synchronize` - Synchronize robot actions

**Implementation**:
- State manager tracks all robots
- Coordinate via shared state or ROS topics
- Unity: Multiple GameObjects with shared controller

#### 5. **Route Recording & Playback**
**Status**: ❌ Missing (patrol routes mentioned but no recording)  
**Impact**: Medium - Can't record and replay robot movements  
**Needed Tools**:
- `robot_routes` portmanteau:
  - `start_recording` - Start recording robot path
  - `stop_recording` - Stop recording and save route
  - `play_route` - Playback recorded route
  - `save_route` - Save route to file
  - `load_route` - Load route from file
  - `edit_route` - Edit waypoints in route

**Implementation**:
- Record position/orientation over time
- Save as JSON/YAML
- Playback via `robot_control` move commands

### 🟡 Important Gaps (Medium Priority)

#### 6. **Configuration Management**
**Status**: ❌ Missing  
**Impact**: Medium - No way to save/load robot configs  
**Needed Tools**:
- `robot_config` portmanteau:
  - `save_config` - Save robot configuration
  - `load_config` - Load robot configuration
  - `export_config` - Export config to file
  - `import_config` - Import config from file
  - `reset_config` - Reset to defaults
  - `validate_config` - Validate configuration

**Implementation**:
- YAML/JSON config files
- Store in state manager or file system

#### 7. **Sensor Simulation (Beyond LiDAR)**
**Status**: ⚠️ Partial (LiDAR exists, others missing)  
**Impact**: Medium - Limited sensor simulation  
**Needed Tools**:
- Extend `robot_sensors` portmanteau:
  - `get_imu` - Get IMU data (acceleration, gyro, magnetometer)
  - `get_odometry` - Get wheel odometry
  - `get_battery` - Get battery level
  - `get_temperature` - Get temperature sensors
  - `get_proximity` - Get proximity sensors
  - `simulate_sensor` - Simulate sensor failure/noise

**Implementation**:
- Physical: ROS sensor topics
- Unity: Simulate via Unity physics/components

#### 8. **Testing & Validation Tools**
**Status**: ❌ Missing  
**Impact**: Medium - No automated testing tools  
**Needed Tools**:
- `robot_testing` portmanteau:
  - `test_size_fit` - Test if robot fits in space (Marble apartment)
  - `test_doorway` - Test doorway clearance
  - `test_patrol_route` - Validate patrol route
  - `test_collision` - Test collision detection
  - `benchmark_performance` - Performance benchmarks
  - `generate_test_report` - Generate test results

**Implementation**:
- Unity: Physics queries, NavMesh checks
- Physical: ROS test nodes

#### 9. **State Persistence**
**Status**: ⚠️ Partial (in-memory only)  
**Impact**: Medium - Robot states lost on restart  
**Needed Tools**:
- Extend state manager:
  - `save_state` - Save all robot states to file
  - `load_state` - Load robot states from file
  - `export_state` - Export state snapshot
  - `import_state` - Import state snapshot

**Implementation**:
- JSON/YAML persistence
- SQLite for complex state

#### 10. **Robot-Specific Behaviors**
**Status**: ⚠️ Partial (generic control only)  
**Impact**: Low - Missing robot-specific features  
**Needed Tools**:
- `scout_control` portmanteau (Scout-specific):
  - `mecanum_move` - Mecanum wheel omnidirectional movement
  - `set_wheel_speeds` - Individual wheel speed control
  - `calibrate_wheels` - Wheel calibration
- `unitree_control` portmanteau (Unitree-specific):
  - `set_gait` - Set walking gait
  - `balance_mode` - Enable/disable balance
  - `manipulate` - Arm manipulation (G1)

**Implementation**:
- Physical: Robot-specific ROS nodes
- Virtual: Unity scripts for robot-specific behaviors

### 🟢 Nice-to-Have Gaps (Low Priority)

#### 11. **Visualization & Debugging**
**Status**: ❌ Missing  
**Impact**: Low - Limited debugging capabilities  
**Needed Tools**:
- `robot_visualization` portmanteau:
  - `show_path` - Visualize planned path
  - `show_sensors` - Visualize sensor ranges
  - `show_collision` - Visualize collision bounds
  - `debug_overlay` - Debug information overlay

**Implementation**:
- Unity: Gizmos, Debug.DrawLine
- ROS: RViz markers

#### 12. **Export/Import Workflows**
**Status**: ⚠️ Partial (models only)  
**Impact**: Low - Limited workflow portability  
**Needed Tools**:
- `robot_export` portmanteau:
  - `export_workflow` - Export complete workflow
  - `import_workflow` - Import workflow
  - `export_scene` - Export Unity scene with robots
  - `import_scene` - Import scene

## Recommended Implementation Order

### Phase 1: Critical Gaps (Next Sprint)
1. **robot_animation** - Animation & behavior
2. **robot_camera** - Camera & visual feeds
3. **robot_navigation** - Path planning (complete stub)

### Phase 2: Important Gaps (Following Sprint)
4. **multi_robot** - Multi-robot coordination
5. **robot_routes** - Route recording/playback
6. **robot_config** - Configuration management

### Phase 3: Nice-to-Have (Future)
7. **robot_testing** - Testing & validation
8. **robot_visualization** - Debugging tools
9. **robot_export** - Workflow export/import

## Integration Points

### Existing MCP Servers
- **unity3d-mcp**: Use `execute_unity_method` for Unity integration
- **avatar-mcp**: Use for animations and poses
- **blender-mcp**: Already integrated for models
- **gimp-mcp**: Already integrated for textures

### Missing Integrations
- **vrchat-mcp**: Not mounted (could add for VRChat-specific features)
- **avatar-mcp**: Mounted but underutilized (animation features)

## Notes

- Most gaps can be filled by extending existing portmanteau tools
- Unity integration via `execute_unity_method` is key for virtual robots
- Physical robot gaps require ROS integration (Phase 4 in PLAN)
- Consider creating Unity C# scripts for complex behaviors (like VbotSpawner.cs)

