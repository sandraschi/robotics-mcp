# Gazebo Integration

**Last Updated**: 2025-02-08

Gazebo simulation support for robotics-mcp via rosbridge WebSocket. Control simulated robots (TurtleBot3, differential drive, etc.) using the same `robot_control` tool as physical robots.

## Architecture

- **Protocol**: rosbridge WebSocket (port 9090)
- **Topics**: Same as Yahboom: `/cmd_vel`, `/odom`, `/scan`, `/camera/image_raw`
- **Client**: `roslibpy` (optional dependency, `pip install robotics-mcp[ros]`)

## Configuration

Add to `~/.robotics-mcp/config.yaml`:

```yaml
robotics:
  gazebo:
    enabled: true
    robot_id: "gazebo_01"
    host: "localhost"
    port: 9090
    model: "turtlebot3"
    cmd_vel_topic: "/cmd_vel"
    odom_topic: "/odom"
    scan_topic: "/scan"
    camera_enabled: true
    mock_mode: false  # Set true to test without Gazebo
```

## Prerequisites

1. **ROS 1 (Melodic/Noetic)** or **ROS 2** with Gazebo
2. **rosbridge**: `sudo apt install ros-<distro>-rosbridge-suite`
3. **roslibpy**: `pip install roslibpy` or `pip install robotics-mcp[ros]`

## Quick Start (ROS 1 Melodic)

### Terminal 1: ROS core + rosbridge

```bash
roscore
# In another terminal:
roslaunch rosbridge_server rosbridge_websocket.launch
```

### Terminal 2: Gazebo + TurtleBot3

```bash
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

### Terminal 3: Robotics MCP

```bash
cd robotics-mcp
pip install -e ".[ros]"
robotics-mcp  # or python -m robotics_mcp.server
```

Then use MCP tool: `robot_control(robot_id="gazebo_01", action="get_status")` or `action="move", linear=0.2, angular=0.0`.

## Docker (Optional)

```powershell
cd docker
docker compose -f docker-compose.gazebo.yml up -d
```

This runs roscore + rosbridge on port 9090. Launch Gazebo in a separate terminal:

```powershell
docker exec -it robotics-gazebo bash
source /opt/ros/melodic/setup.bash
roslaunch gazebo_ros empty_world.launch
# Or for TurtleBot3: roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

Ensure port 9090 is exposed. Point `host` in config to `localhost` when robotics-mcp runs on the host.

## MCP Tool Usage

| Action       | Example |
|--------------|---------|
| get_status   | `robot_control(robot_id="gazebo_01", action="get_status")` |
| move         | `robot_control(robot_id="gazebo_01", action="move", linear=0.3, angular=0.0, duration=2.0)` |
| stop         | `robot_control(robot_id="gazebo_01", action="stop")` |

## Virtual Robotics (platform=gazebo)

The `virtual_robotics` tool supports `platform="gazebo"` for spawn and load_environment. Gazebo worlds/robots must be launched manually via roslaunch; the tool returns instructions.

## Web App

The web dashboard includes Gazebo support:

- **Robot Control Hub** (`/robot-control`): Gazebo card with link to dedicated control page
- **Gazebo Control Page** (`/robot-control/gazebo`): Movement controls (linear/angular velocity sliders), Move/Stop buttons, status display
- **Robot Dashboard**: Gazebo robots use generic move/stop controls when selected

Requires robotics-mcp server running with HTTP enabled (port 12230). Vite proxy forwards `/api/v1` to the server.

## Troubleshooting

- **Connection refused**: Ensure rosbridge is running and port 9090 is accessible.
- **No /cmd_vel**: Launch a robot model (e.g. TurtleBot3) that publishes/subscribes to cmd_vel.
- **roslibpy not found**: Install with `pip install roslibpy` or `pip install robotics-mcp[ros]`.
