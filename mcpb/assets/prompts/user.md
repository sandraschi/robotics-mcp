# Robotics MCP User Guide

## Quick Start

### Getting Help
Ask: "What tools are available in robotics-mcp?"
Use: `help()` tool to get comprehensive server information

### Checking Server Status
Ask: "What's the status of the robotics server?"
Use: `get_status()` to see server health, robots, and mounted servers

### Listing Robots
Ask: "Show me all registered robots"
Use: `list_robots()` to see all robots

Ask: "Show me only virtual robots"
Use: `list_robots(is_virtual=True)`

Ask: "Show me only Scout robots"
Use: `list_robots(robot_type="scout")`

## Virtual Robot Operations

### Spawning a Virtual Robot
Ask: "Spawn a Scout robot in Unity at position (0, 0, 0)"
Use: `virtual_robotics(robot_type="scout", action="spawn_robot", platform="unity", position={"x": 0.0, "y": 0.0, "z": 0.0})`

Ask: "Spawn a Scout robot in VRChat"
Use: `virtual_robotics(robot_type="scout", action="spawn_robot", platform="vrchat")`

### Loading Environments
Ask: "Load the apartment environment in Unity"
Use: `virtual_robotics(action="load_environment", environment="apartment", platform="unity")`

### Getting Robot Status
Ask: "What's the status of vbot_scout_01?"
Use: `virtual_robotics(robot_type="scout", action="get_status", robot_id="vbot_scout_01")`

### Scaling Robots
Ask: "Scale vbot_scout_01 to 1.5x size"
Use: `virtual_robotics(robot_type="scout", action="set_scale", robot_id="vbot_scout_01", scale=1.5)`

### Getting Virtual LiDAR
Ask: "Get LiDAR scan from vbot_scout_01"
Use: `virtual_robotics(robot_type="scout", action="get_lidar", robot_id="vbot_scout_01")`

## Robot Control Operations

### Getting Robot Status
Ask: "What's the status of scout_01?"
Use: `robot_control(robot_id="scout_01", action="get_status")`

### Moving a Robot
Ask: "Move scout_01 forward at 0.2 m/s"
Use: `robot_control(robot_id="scout_01", action="move", linear=0.2, angular=0.0)`

Ask: "Turn scout_01 left at 0.1 rad/s"
Use: `robot_control(robot_id="scout_01", action="move", linear=0.0, angular=0.1)`

Ask: "Move scout_01 forward and turn for 5 seconds"
Use: `robot_control(robot_id="scout_01", action="move", linear=0.2, angular=0.1, duration=5.0)`

### Stopping a Robot
Ask: "Stop scout_01 immediately"
Use: `robot_control(robot_id="scout_01", action="stop")`

### Returning to Dock
Ask: "Send scout_01 back to its charging dock"
Use: `robot_control(robot_id="scout_01", action="return_to_dock")`

### Unitree-Specific Actions
Ask: "Make go2_01 stand up"
Use: `robot_control(robot_id="go2_01", action="stand")`

Ask: "Make go2_01 sit down"
Use: `robot_control(robot_id="go2_01", action="sit")`

Ask: "Start walking gait for go2_01"
Use: `robot_control(robot_id="go2_01", action="walk")`

## Virtual Robot CRUD Operations

### Creating Virtual Robots
Ask: "Create a Scout robot in Unity"
Use: `vbot_crud(operation="create", robot_type="scout", platform="unity")`

Ask: "Create Robbie from Forbidden Planet in Unity at position (1, 0, 1)"
Use: `vbot_crud(operation="create", robot_type="robbie", platform="unity", position={"x": 1.0, "y": 0.0, "z": 1.0})`

Ask: "Create a Unitree Go2 in VRChat"
Use: `vbot_crud(operation="create", robot_type="go2", platform="vrchat")`

Ask: "Create a custom robot with my model"
Use: `vbot_crud(operation="create", robot_type="custom", platform="unity", model_path="D:/Models/my_robot.glb")`

### Reading Virtual Robot Details
Ask: "Get details of vbot_scout_01"
Use: `vbot_crud(operation="read", robot_id="vbot_scout_01")`

### Updating Virtual Robots
Ask: "Update vbot_scout_01 scale to 1.5x"
Use: `vbot_crud(operation="update", robot_id="vbot_scout_01", scale=1.5)`

Ask: "Move vbot_scout_01 to position (2, 0, 2)"
Use: `vbot_crud(operation="update", robot_id="vbot_scout_01", position={"x": 2.0, "y": 0.0, "z": 2.0})`

Ask: "Update vbot_scout_01 with custom metadata"
Use: `vbot_crud(operation="update", robot_id="vbot_scout_01", metadata={"color": "red", "speed": 0.5})`

### Deleting Virtual Robots
Ask: "Delete vbot_scout_01"
Use: `vbot_crud(operation="delete", robot_id="vbot_scout_01")`

### Listing Virtual Robots
Ask: "List all virtual robots"
Use: `vbot_crud(operation="list")`

Ask: "List all Scout virtual robots"
Use: `vbot_crud(operation="list", robot_type="scout")`

Ask: "List all Unity virtual robots"
Use: `vbot_crud(operation="list", platform="unity")`

## Common Use Cases

### Testing Before Hardware Arrives
1. Spawn virtual robot: `virtual_robotics(robot_type="scout", action="spawn_robot", platform="unity")`
2. Load test environment: `virtual_robotics(action="load_environment", environment="test_room", platform="unity")`
3. Test movement: `robot_control(robot_id="vbot_scout_01", action="move", linear=0.2, angular=0.0)`
4. Test navigation: `virtual_robotics(action="test_navigation", robot_id="vbot_scout_01")`

### Multi-Robot Coordination
1. List all robots: `list_robots()`
2. Control each robot: `robot_control(robot_id="scout_01", action="get_status")`
3. Filter by type: `list_robots(robot_type="scout")`
4. Filter by virtual: `list_robots(is_virtual=True)`

### Size Testing
1. Spawn robot: `virtual_robotics(robot_type="scout", action="spawn_robot", platform="unity")`
2. Test different scales: `virtual_robotics(action="set_scale", robot_id="vbot_scout_01", scale=1.0)`
3. Compare sizes: Spawn multiple robots with different scales

## Error Handling

If a tool returns an error:
- Check the `error_type` field to understand the error category
- Check the `message` field for human-readable error description
- Check the `details` field for additional context

Common errors:
- `not_found`: Robot doesn't exist - use `list_robots()` to see available robots
- `validation_error`: Invalid parameters - check tool documentation
- `connection_error`: Cannot connect to robot - check robot is powered on and connected

## Tips

1. Always check robot status before controlling: `robot_control(robot_id="...", action="get_status")`
2. Use virtual robots for testing before hardware arrives
3. Use `help()` to discover all available tools and their parameters
4. Use `get_status()` to check server health and mounted MCP servers
5. Filter robots with `list_robots()` to find specific robots quickly

