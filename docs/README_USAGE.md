# Robotics MCP — Usage and API

[Project README](../README.md) · [Technical documentation](README_TECHNICAL.md)

MCP tool call examples, web UI notes, and HTTP endpoints. For installation and configuration, see [README_TECHNICAL.md](README_TECHNICAL.md).

## Yahboom arm manipulation example

Example sequence for a Yahboom ROSMASTER with arm and gripper (adjust joint targets to your calibration):

```python
# Example: pick up a small object (calibrate poses for your arm)
await robot_control(
    robot_id="yahboom_01",
    action="arm_move",
    joint_angles={"joint1": 0.0, "joint2": 0.5, "joint3": -0.3, "joint4": 0.0}
)

# Open gripper
await robot_control(robot_id="yahboom_01", action="gripper_control", gripper_action="open")

# Move arm to target
await robot_control(
    robot_id="yahboom_01",
    action="arm_move",
    joint_angles={"joint1": 0.2, "joint2": 0.7, "joint3": -0.1, "joint4": 0.1}
)

# Close gripper
await robot_control(robot_id="yahboom_01", action="gripper_control", gripper_action="close")

# Lift
await robot_control(
    robot_id="yahboom_01",
    action="arm_move",
    joint_angles={"joint1": 0.2, "joint2": 0.9, "joint3": 0.1, "joint4": 0.1}
)
```

**Hardware (indicative):** Yahboom ROSMASTER X3 or X3 Plus; arm kit; parallel gripper. See [Technical README](README_TECHNICAL.md#prerequisites-and-dependencies) for budget notes.

## MCP tool examples

### Robot control

**Dreame D20 Pro (robot vacuum):**

```python
await robot_control(robot_id="dreame_01", action="get_status")
await robot_control(robot_id="dreame_01", action="start_cleaning")
await robot_control(robot_id="dreame_01", action="clean_zone", zones=[[0, 0, 500, 300]])
await robot_control(robot_id="dreame_01", action="return_to_dock")
await robot_control(robot_id="dreame_01", action="get_map")  # LIDAR map
```

**Yahboom ROSMASTER (ROS 2):**

```python
await robot_control(robot_id="yahboom_01", action="get_status")
await robot_control(robot_id="yahboom_01", action="home_patrol")
await robot_control(robot_id="yahboom_01", action="camera_capture")
await robot_control(robot_id="yahboom_01", action="arm_move", joint_angles={"joint1": 0.0, "joint2": 0.5})
```

**Tdrone Mini (example PX4-style actions):**

```python
await robot_control(robot_id="tdrone_01", action="takeoff", altitude=3.0)
await robot_control(robot_id="tdrone_01", action="goto_waypoint", x=10, y=5, altitude=5)
await robot_control(robot_id="tdrone_01", action="set_mode", mode="position_hold")
await robot_control(robot_id="tdrone_01", action="return_home")
```

**Philips Hue Bridge Pro:**

```python
await robot_control(robot_id="hue_01", action="set_light_state",
                   light_id="1", state={"on": true, "brightness": 200})
await robot_control(robot_id="hue_01", action="activate_scene", scene="bright")
```

**Moorebot Scout (ROS 1):**

```python
await robot_control(robot_id="scout_01", action="move", linear=0.2, angular=0.0)
await robot_control(robot_id="scout_01", action="stop")
```

*Unitree Go2/G1: paths exist; behavior depends on hardware and implementation state.*

### Virtual robotics

```python
await virtual_robotics(
    robot_type="scout",
    action="spawn_robot",
    platform="unity",
    position={"x": 0, "y": 0, "z": 0}
)

await virtual_robotics(
    action="load_environment",
    environment="stroheckgasse_apartment",
    platform="unity"
)
```

### Robot model tools

```python
await robot_model_create(
    robot_type="scout",
    output_path="D:/Models/scout_model.fbx",
    format="fbx",
    dimensions={"length": 0.115, "width": 0.10, "height": 0.08},
    create_textures=True,
    texture_style="realistic"
)

await robot_model_import(
    robot_type="scout",
    model_path="D:/Models/scout_model.fbx",
    format="fbx",
    platform="unity",
    project_path="D:/Projects/UnityRobots"
)

await robot_model_convert(
    source_path="D:/Models/scout.fbx",
    source_format="fbx",
    target_format="glb",
    target_path="D:/Models/scout.glb"
)
```

### Drone control

```python
await drone_control(
    operation="get_status",
    drone_id="px4_quad_01"
)

await drone_control(
    operation="takeoff",
    drone_id="px4_quad_01",
    altitude=5.0
)

await drone_control(
    operation="move",
    drone_id="px4_quad_01",
    velocity_x=1.0,
    velocity_y=0.0,
    velocity_z=0.0,
    yaw_rate=0.1
)

await drone_control(
    operation="arm",
    drone_id="px4_quad_01"
)

await drone_control(
    operation="return_home",
    drone_id="px4_quad_01"
)
```

### Drone streaming

```python
await drone_streaming(
    operation="start_fpv",
    drone_id="px4_quad_01",
    quality="720p"
)

stream_info = await drone_streaming(
    operation="get_stream_url",
    drone_id="px4_quad_01",
    protocol="rtsp"
)
print(f"Stream URL: {stream_info['url']}")

await drone_streaming(
    operation="start_recording",
    drone_id="px4_quad_01",
    filename="flight_2025-01-17.mp4"
)

await drone_streaming(
    operation="take_snapshot",
    drone_id="px4_quad_01",
    filename="aerial_view.jpg"
)
```

### Drone navigation

```python
position = await drone_navigation(
    operation="get_position",
    drone_id="px4_quad_01"
)
print(f"Lat: {position['latitude']}, Lon: {position['longitude']}, Alt: {position['altitude']}")

await drone_navigation(
    operation="set_waypoint",
    drone_id="px4_quad_01",
    latitude=37.7749,
    longitude=-122.4194,
    altitude=10.0
)

await drone_navigation(
    operation="enable_follow_me",
    drone_id="px4_quad_01",
    target_id="operator_gps"
)

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

### Drone flight control

```python
await drone_flight_control(
    operation="set_flight_mode",
    drone_id="px4_quad_01",
    mode="AUTO"
)

modes = await drone_flight_control(
    operation="get_flight_modes",
    drone_id="px4_quad_01"
)
print(f"Available modes: {modes['modes']}")

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

await drone_flight_control(
    operation="start_mission",
    drone_id="px4_quad_01",
    mission_id="recon_mission_01"
)

await drone_flight_control(
    operation="set_parameter",
    drone_id="px4_quad_01",
    param_name="WPNAV_SPEED",
    param_value=500
)
```

## Web interface

Default URL (when the web stack is running and bound to this port):

```
http://localhost:8081
```

Features depend on your build: status, controls, logging. Layout of the Vite/React app: [web_sota/README.md](../web_sota/README.md).

## HTTP API

Base URL defaults to `http://localhost:12230` when HTTP mode is enabled.

### Health

```bash
curl http://localhost:12230/api/v1/health
```

### List robots

```bash
curl http://localhost:12230/api/v1/robots
```

### Control robot

```bash
curl -X POST http://localhost:12230/api/v1/robots/scout_01/control \
  -H "Content-Type: application/json" \
  -d "{\"action\": \"move\", \"linear\": 0.2, \"angular\": 0.0}"
```

### List tools

```bash
curl http://localhost:12230/api/v1/tools
```

### Call tool

```bash
curl -X POST http://localhost:12230/api/v1/tools/robot_control \
  -H "Content-Type: application/json" \
  -d "{\"robot_id\": \"scout_01\", \"action\": \"get_status\"}"
```

HTTP routes and payloads align with the FastAPI app under `src/robotics_mcp`; inspect there or extend [README_TECHNICAL.md](README_TECHNICAL.md) when you add stable API docs.
