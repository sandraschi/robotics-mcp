# Robotics MCP User Guide

## Getting Started

### Server Health and Discovery
Ask: "What tools are available in robotics-mcp?"
- Use: `robotics_system(operation="help")` - Get comprehensive server info, all registered tools, features, and mounted MCP servers

Ask: "What's the status of the robotics server?"
- Use: `robotics_system(operation="status")` - Server health, registered robots, mounted server connectivity, HTTP server status

Ask: "Show me all registered robots"
- Use: `robotics_system(operation="list_robots")` - List all robots with counts and filters

### Robot Discovery
Ask: "Show me only virtual robots"
- Use: `robotics_system(operation="list_robots", is_virtual=True)`

Ask: "Show me only Scout robots"
- Use: `robotics_system(operation="list_robots", robot_type="scout")`

Ask: "Show me only Dreame vacuums"
- Use: `robotics_system(operation="list_robots", robot_type="dreame")`

---

## Tutorial 1: Physical Robot Control

### Dreame D20 Pro Vacuum (Primary Platform)

Ask: "Start cleaning with dreame_01"
- Use: `robot_control(robot_id="dreame_01", action="start_cleaning")`

Ask: "Stop cleaning and send dreame_01 back to dock"
- Use: `robot_control(robot_id="dreame_01", action="stop_cleaning")`
- Then: `robot_control(robot_id="dreame_01", action="return_to_dock")`

Ask: "Clean a specific zone in the living room"
- Use: `robot_control(robot_id="dreame_01", action="clean_zone", zones=[[0, 0, 500, 300]])`
- Zones are rect coordinates [x1, y1, x2, y2] in cm from map origin

Ask: "Intensive spot clean at coordinates (200, 150)"
- Use: `robot_control(robot_id="dreame_01", action="clean_spot", spot_x=200, spot_y=150)`

Ask: "Set suction to max and reduce mop humidity"
- Use: `robot_control(robot_id="dreame_01", action="set_suction_level", suction_level=4)`
- Then: `robot_control(robot_id="dreame_01", action="set_mop_humidity", mop_humidity=1)`

Ask: "Get the map from dreame_01"
- Use: `robot_control(robot_id="dreame_01", action="get_map")`

Ask: "Start fast mapping mode"
- Use: `robot_control(robot_id="dreame_01", action="start_fast_mapping")`

Ask: "Rename room 1 to 'Living Room'"
- Use: `robot_control(robot_id="dreame_01", action="rename_room", room_id=1, room_name="Living Room")`

Ask: "Change cleaning order to rooms 2, 1, 3"
- Use: `robot_control(robot_id="dreame_01", action="set_cleaning_sequence", cleaning_sequence=[2, 1, 3])`

Ask: "Add a virtual wall around the TV area"
- Use: `robot_control(robot_id="dreame_01", action="set_restricted_zones", restricted_zones={"walls": [[100, 100, 200, 100]], "zones": []})`

Ask: "Get cleaning history for today"
- Use: `robot_control(robot_id="dreame_01", action="get_cleaning_history")`

Ask: "Clear the error on dreame_01"
- Use: `robot_control(robot_id="dreame_01", action="clear_error")`

Ask: "Export the Dreame map as an OBJ file"
- Use: `dreame_control(operation="export_map", format="obj", output_path="C:/Maps/living_room.obj")`

### Dreame Dedicated Tools

Ask: "Use the dedicated Dreame control tool to start cleaning"
- Use: `dreame_control(operation="start_cleaning")`

Ask: "What are all the Dreame control operations?"
- Use: `dreame_control(operation="get_status")` - Shows available Dreame operations

### Yahboom ROSMASTER Robot

Ask: "Get status of yahboom_01"
- Use: `robot_control(robot_id="yahboom_01", action="get_status")`

Ask: "Move yahboom_01 forward at 0.3 m/s"
- Use: `robot_control(robot_id="yahboom_01", action="move", linear=0.3, angular=0.0)`

Ask: "Turn yahboom_01 left at 0.2 rad/s for 3 seconds"
- Use: `robot_control(robot_id="yahboom_01", action="move", linear=0.0, angular=0.2, duration=3.0)`

Ask: "Emergency stop yahboom_01"
- Use: `robot_control(robot_id="yahboom_01", action="stop")`

Ask: "Start home security patrol with yahboom_01"
- Use: `robot_control(robot_id="yahboom_01", action="home_patrol", patrol_route="home_security")`

Ask: "Capture camera image from yahboom_01"
- Use: `robot_control(robot_id="yahboom_01", action="camera_capture")`

Ask: "Move the arm to specific joint angles"
- Use: `robot_control(robot_id="yahboom_01", action="arm_move", joint_angles={"shoulder": 45.0, "elbow": 90.0, "wrist": 30.0})`

Ask: "Close the gripper"
- Use: `robot_control(robot_id="yahboom_01", action="gripper_control", gripper_action="close")`

Ask: "Navigate yahboom_01 to position x=2, y=3"
- Use: `robot_control(robot_id="yahboom_01", action="navigate_to", x=2.0, y=3.0, theta=1.57)`

Ask: "Ask yahboom_01 what it sees"
- Use: `robot_control(robot_id="yahboom_01", action="ai_query", query="What is directly in front of me?", query_type="vision")`

### Unitree Quadruped Robot

Ask: "Make go2_01 stand up"
- Use: `robot_control(robot_id="go2_01", action="stand")`

Ask: "Sit go2_01 down"
- Use: `robot_control(robot_id="go2_01", action="sit")`

Ask: "Start walking with go2_01"
- Use: `robot_control(robot_id="go2_01", action="walk")`

Ask: "Check go2_01 status and battery"
- Use: `robot_control(robot_id="go2_01", action="get_status")`

### Elegoo Robot (Serial via ROS-on-PC)

Ask: "Connect and get status of elegoo_01"
- Use: `robot_control(robot_id="elegoo_01", action="get_status")`

Ask: "Move elegoo_01 forward slowly"
- Use: `robot_control(robot_id="elegoo_01", action="move", linear=0.1, angular=0.0)`

Ask: "Emergency stop elegoo_01"
- Use: `robot_control(robot_id="elegoo_01", action="emergency_stop")`

### Hue HomeAware Movement Detection

Ask: "Get movement events from hue_01"
- Use: `robot_control(robot_id="hue_01", action="hue_get_movement_events")`

Ask: "Check sensor status on hue_01"
- Use: `robot_control(robot_id="hue_01", action="hue_get_sensor_status")`

Ask: "List movement zones for hue_01"
- Use: `robot_control(robot_id="hue_01", action="hue_get_movement_zones")`

### Gazebo Simulation Robot

Ask: "Get status of gazebo_robot_01 in simulation"
- Use: `robot_control(robot_id="gazebo_robot_01", action="get_status")`

Ask: "Move the Gazebo robot forward"
- Use: `robot_control(robot_id="gazebo_robot_01", action="move", linear=0.5, angular=0.0, duration=2.0)`

---

## Tutorial 1b: Nori A3 Bimanual Manipulator

The Nori A3 is bridged over HTTP to a standalone `norirobotics-mcp` server, not driven
directly by robotics-mcp - `norirobotics-mcp` must be running (default
`http://127.0.0.1:11970`) before any of these calls will succeed. It defaults to a mock
session (no real hardware required) unless real credentials are configured, so every example
below is safe to try without owning the physical robot.

Ask: "Check the status of nori_01"
- Use: `robot_control(robot_id="nori_01", action="get_status")`
- The response includes `"connected"` and `"mock"` booleans, plus a nested
  `"connection_status"` (phase/failure/detail) once connected, and `"camera_layout"` (grid
  tile layout for the 4 onboard cameras) when a session is active.

Ask: "Connect to nori_01"
- Use: `robot_control(robot_id="nori_01", action="connect")`
- Opens a session against the Nori's control daemon. In mock mode this returns a synthetic
  session descriptor (joint names, ranges, camera list, capabilities) rather than a real
  hardware handshake - useful for testing the integration without hardware.

Ask: "Start recording a demonstration of picking up the red block"
- Use: `robot_control(robot_id="nori_01", action="episode_start", query="pick up the red block")`
- `query` here is the free-text task description stored alongside the recorded episode, not a
  question being asked to the robot - it becomes the LeRobot dataset's task label.

Ask: "Stop the current recording"
- Use: `robot_control(robot_id="nori_01", action="episode_stop")`
- Response includes `"episodes_kept"` (running count of saved episodes this session) and
  `"free_gb"` (remaining storage) so a caller can tell whether it's worth recording more
  without a separate status check.

Ask: "Emergency stop nori_01"
- Use: `robot_control(robot_id="nori_01", action="stop")`
- This is a real e-stop trigger, distinct from `disconnect` - use it to halt in-progress
  motion, not just to end the session cleanly.

Ask: "Disconnect from nori_01"
- Use: `robot_control(robot_id="nori_01", action="disconnect")`

Ask: "Record three demonstration episodes of a stacking task, back to back"
- Use: `robot_control(robot_id="nori_01", action="connect")`
- Then, three times: `robot_control(robot_id="nori_01", action="episode_start", query="stack the blue block on the green block")` followed by (after the demonstration) `robot_control(robot_id="nori_01", action="episode_stop")`
- Then: `robot_control(robot_id="nori_01", action="disconnect")`
- Each `episode_stop` response's `"episodes_kept"` should increment by one across the three
  passes - a quick sanity check that nothing silently failed mid-sequence.

---

## Tutorial 2: Robot Behavior Control

### Animation Control

Ask: "Spin the wheels on scout_01"
- Use: `robot_behavior(robot_id="scout_01", category="animation", action="set_wheel_speeds", wheel_speeds={"front_left": 1.0, "front_right": 1.0, "back_left": 1.0, "back_right": 1.0})`

Ask: "Play a walking animation on go2_01"
- Use: `robot_behavior(robot_id="go2_01", category="animation", action="play_animation", animation_name="walk", animation_speed=1.0, loop=True)`

Ask: "Make g1_01 stand up"
- Use: `robot_behavior(robot_id="g1_01", category="animation", action="set_pose", pose="stand")`

Ask: "Stop the current animation"
- Use: `robot_behavior(robot_id="go2_01", category="animation", action="stop_animation")`

### Camera Control

Ask: "Get camera feed from scout_01"
- Use: `robot_behavior(robot_id="scout_01", category="camera", action="get_camera_feed")`

Ask: "Capture a still image from the virtual robot camera"
- Use: `robot_behavior(robot_id="vbot_scout_01", category="camera", action="capture_image", output_path="C:/Screenshots/scout_view.png")`

Ask: "Tilt the camera up 15 degrees"
- Use: `robot_behavior(robot_id="scout_01", category="camera", action="set_camera_angle", angle_x=15.0, angle_y=0.0)`

Ask: "Start video streaming from scout_01"
- Use: `robot_behavior(robot_id="scout_01", category="camera", action="start_streaming", stream_url="rtsp://localhost:8554/scout")`

### Navigation

Ask: "Plan a path from (0,0,0) to (5,3,0)"
- Use: `robot_behavior(robot_id="scout_01", category="navigation", action="plan_path", start_position={"x": 0.0, "y": 0.0, "z": 0.0}, goal_position={"x": 5.0, "y": 3.0, "z": 0.0})`

Ask: "Execute path path_001"
- Use: `robot_behavior(robot_id="scout_01", category="navigation", action="follow_path", path_id="path_001")`

Ask: "Set a waypoint at (2, 1, 0)"
- Use: `robot_behavior(robot_id="scout_01", category="navigation", action="set_waypoint", waypoint={"x": 2.0, "y": 1.0, "z": 0.0})`

Ask: "Avoid an obstacle at (3, 2, 0)"
- Use: `robot_behavior(robot_id="scout_01", category="navigation", action="avoid_obstacle", obstacle_position={"x": 3.0, "y": 2.0, "z": 0.0})`

### Manipulation (Arm Control)

Ask: "Move the arm joints to specific angles"
- Use: `robot_behavior(robot_id="g1_01", category="manipulation", action="move_joints", joint_positions={"joint_1": 30.0, "joint_2": 45.0, "joint_3": 90.0})`

Ask: "Move the end effector to position (0.5, 0.3, 0.8)"
- Use: `robot_behavior(robot_id="g1_01", category="manipulation", action="move_end_effector", end_effector_pose={"x": 0.5, "y": 0.3, "z": 0.8, "roll": 0.0, "pitch": 0.0, "yaw": 0.0})`

Ask: "Open the gripper"
- Use: `robot_behavior(robot_id="g1_01", category="manipulation", action="control_gripper", gripper_position=0.0)`

Ask: "Close the gripper to 80%"
- Use: `robot_behavior(robot_id="g1_01", category="manipulation", action="control_gripper", gripper_position=0.8)`

---

## Tutorial 3: Virtual Robot Operations

### Creating and Managing Virtual Robots

Ask: "Create a Scout robot in Unity at position (0, 0, 0)"
- Use: `robot_virtual(operation="create", robot_type="scout", platform="unity", position={"x": 0.0, "y": 0.0, "z": 0.0})`

Ask: "Spawn a Robbie from Forbidden Planet in VRChat"
- Use: `robot_virtual(operation="create", robot_type="robbie", platform="vrchat")`

Ask: "Create a Go2 quadruped in Unity"
- Use: `robot_virtual(operation="create", robot_type="go2", platform="unity", position={"x": 1.0, "y": 0.0, "z": 1.0})`

Ask: "Create a Godzilla-scale robot at origin"
- Use: `robot_virtual(operation="create", robot_type="godzilla", platform="unity", position={"x": 0.0, "y": 0.0, "z": 0.0})`

Ask: "List all virtual robots"
- Use: `robot_virtual(operation="list")`

Ask: "Get details of vbot_scout_01"
- Use: `robot_virtual(operation="read", robot_id="vbot_scout_01")`

Ask: "Scale vbot_scout_01 to 2x size"
- Use: `robot_virtual(operation="update", robot_id="vbot_scout_01", scale=2.0)`

Ask: "Move vbot_scout_01 to position (3, 0, 5)"
- Use: `robot_virtual(operation="update", robot_id="vbot_scout_01", position={"x": 3.0, "y": 0.0, "z": 5.0})`

Ask: "Delete vbot_scout_01"
- Use: `robot_virtual(operation="delete", robot_id="vbot_scout_01")`

### Virtual Environment Management

Ask: "Load the apartment environment in Unity"
- Use: `robot_virtual(operation="load_environment", environment="apartment", platform="unity")`

Ask: "Load a warehouse environment for testing"
- Use: `robot_virtual(operation="load_environment", environment="warehouse", platform="unity")`

Ask: "Load a procedural outdoor environment"
- Use: `robot_virtual(operation="load_environment", environment="outdoor", platform="unity")`

### Virtual Robot Status and Sensing

Ask: "Get status of vbot_scout_01 in Unity"
- Use: `robot_virtual(operation="get_status", robot_id="vbot_scout_01", platform="unity")`

Ask: "Get virtual LiDAR scan from vbot_scout_01"
- Use: `robot_virtual(operation="get_lidar", robot_id="vbot_scout_01")`

### Virtual CRUD (Dedicated Tool)

Ask: "Create a virtual Scout robot"
- Use: `vbot_crud(operation="create", robot_type="scout", platform="unity")`

Ask: "List all Scout virtual robots"
- Use: `vbot_crud(operation="list", robot_type="scout")`

Ask: "List all Unity virtual robots"
- Use: `vbot_crud(operation="list", platform="unity")`

Ask: "Update vbot_01 position and metadata"
- Use: `vbot_crud(operation="update", robot_id="vbot_01", position={"x": 5.0, "y": 0.0, "z": 2.0}, metadata={"color": "red", "speed": 0.8})`

---

## Tutorial 4: Simulation Fleet Orchestration

Ask: "What simulation backends are available?"
- Use: `sim_fleet_status()`

Ask: "List all registered sim backends and their capabilities"
- Use: `sim_fleet_backends()`

Ask: "Route a sensor simulation task to the best backend"
- Use: `sim_fleet_route(task="Simulate LIDAR and depth camera in an indoor environment")`
- The router picks gazebo-mcp for sensor-rich tasks

Ask: "Route a locomotion training task"
- Use: `sim_fleet_route(task="Train walking gait with PPO for quadruped")`
- The router picks mujoco-mcp for locomotion/RL tasks

Ask: "Search for humanoid robot models in the marketplace"
- Use: `sim_marketplace_search_tool(query="humanoid", tags=["biped", "walking"])`

Ask: "Find all quadruped models compatible with MuJoCo"
- Use: `sim_marketplace_search_tool(tags=["quadruped"], sim_backend="mujoco")`

---

## Tutorial 5: Gazebo Simulation Models

Ask: "Search Gazebo Fuel for office furniture models"
- Use: `gazebo_models(operation="search", query="office chair", page=1, per_page=20)`

Ask: "Download a specific model from Gazebo Fuel"
- Use: `gazebo_models(operation="download", model_name="Bookshelf")`

Ask: "Spawn a bookshelf in the simulation at position (2, 0, 0.5)"
- Use: `gazebo_models(operation="spawn", model_name="Bookshelf", spawn_name="office_bookshelf", x=2.0, y=0.0, z=0.5)`

Ask: "List locally downloaded models"
- Use: `gazebo_models(operation="list_local")`

Ask: "Delete a local model"
- Use: `gazebo_models(operation="delete_local", model_name="Bookshelf")`

---

## Tutorial 6: Drone Operations

Ask: "Arm and take off drone_01 to 10 meters"
- Use: `drone_control(operation="takeoff", drone_id="drone_01", altitude=10.0)`

Ask: "Move drone_01 forward and to the right"
- Use: `drone_control(operation="move", drone_id="drone_01", velocity_x=2.0, velocity_y=0.0, velocity_z=0.0, yaw_rate=0.0)`

Ask: "Go to GPS coordinates"
- Use: `drone_control(operation="goto_global", drone_id="drone_01", lat=48.208, lon=16.373, alt=20.0)`

Ask: "Land and disarm drone_01"
- Use: `drone_control(operation="land", drone_id="drone_01")`
- Then: `drone_control(operation="disarm", drone_id="drone_01")`

Ask: "Set waypoints for an autonomous mission"
- Use: `drone_control(operation="set_waypoint", drone_id="drone_01", lat=48.210, lon=16.370, alt=15.0)`

Ask: "Start recording from the drone camera"
- Use: `drone_control(operation="start_recording", drone_id="drone_01")`

Ask: "What is the current battery level?"
- Use: `drone_control(operation="get_battery", drone_id="drone_01")`

---

## Tutorial 7: 3D Model Management

Ask: "Create a new robot model for a Scout robot"
- Use: `robot_model(operation="create", robot_type="scout")`

Ask: "Import a GLB model into Unity"
- Use: `robot_model(operation="import", robot_type="scout", model_path="D:/Models/scout_v2.glb")`

Ask: "Export a robot model to FBX format"
- Use: `robot_model(operation="export", robot_type="scout", model_path="D:/Models/scout_v2.blend", format="fbx", output_path="D:/Exports/scout_v2.fbx")`

Ask: "Convert OBJ to GLB"
- Use: `robot_model(operation="convert", model_path="D:/Models/scout.obj", format="glb", output_path="D:/Models/scout.glb")`

Ask: "Check an SPZ file integrity"
- Use: `robot_model(operation="spz_check", model_path="D:/Splats/scene.spz")`

Ask: "Convert SPZ to PLY point cloud"
- Use: `robot_model(operation="spz_convert", model_path="D:/Splats/scene.spz", output_path="D:/Splats/scene.ply", format="ply")`

Ask: "Install an SPZ into my Unity project"
- Use: `robot_model(operation="spz_install", model_path="D:/Splats/scene.spz", output_path="D:/UnityProject/Assets/Splats")`

Ask: "List available scripts"
- Use: `robot_model(operation="list_scripts")`

---

## Tutorial 8: Manufacturing Equipment

Ask: "Start printing a G-Code file on my 3D printer"
- Use: `robot_manufacturing(device_id="printer_01", device_type="3d_printer", category="control", action="print_file", file_path="D:/GCode/benchy.gcode")`

Ask: "Check print progress"
- Use: `robot_manufacturing(device_id="printer_01", device_type="3d_printer", category="monitor", action="get_progress")`

Ask: "Set nozzle temperature to 210C and bed to 60C"
- Use: `robot_manufacturing(device_id="printer_01", device_type="3d_printer", category="control", action="set_temperature", temperature={"nozzle": 210, "bed": 60})`

Ask: "Level the print bed"
- Use: `robot_manufacturing(device_id="printer_01", device_type="3d_printer", category="maintenance", action="level_bed")`

Ask: "Pause the current print"
- Use: `robot_manufacturing(device_id="printer_01", device_type="3d_printer", category="control", action="pause_print")`

Ask: "Start the CNC machine with a G-Code file"
- Use: `robot_manufacturing(device_id="cnc_01", device_type="cnc_machine", category="control", action="load_gcode", file_path="D:/GCode/part.gcode")`
- Then: `robot_manufacturing(device_id="cnc_01", device_type="cnc_machine", category="control", action="start_milling")`

Ask: "Set laser cutter power to 80%"
- Use: `robot_manufacturing(device_id="laser_01", device_type="laser_cutter", category="control", action="set_power", speed=80.0)`

---

## Tutorial 9: Workflow Management

Ask: "Create a new workflow for daily cleaning"
- Use: `workflow_management(operation="create", name="Daily Cleaning", template="vacuum_routine")`

Ask: "Execute a workflow"
- Use: `workflow_management(operation="execute", workflow_id="wf_001")`

Ask: "Check workflow execution status"
- Use: `workflow_management(operation="status", workflow_id="wf_001")`

Ask: "List all available workflow templates"
- Use: `workflow_management(operation="templates")`

Ask: "Schedule a workflow to run every day at 9 AM"
- Use: `workflow_management(operation="schedule", workflow_id="wf_001", cron="0 9 * * *")`

Ask: "Export a workflow as JSON"
- Use: `workflow_management(operation="export", workflow_id="wf_001", format="json")`

---

## Tutorial 10: Environmental Logistics

Ask: "Start a security patrol with route perimeter"
- Use: `environmental_logistics_handler(operation="security_patrol", route_id="perimeter", patrol_mode="autonomous", duration=3600)`

Ask: "Begin automated garden watering"
- Use: `environmental_logistics_handler(operation="garden_logistics", tool_id="watering_arm")`

Ask: "Paint a mural on the wall with design abstract_wave"
- Use: `environmental_logistics_handler(operation="mural_painting", design_id="abstract_wave", surface_dims={"width": 3.0, "height": 2.0}, tool_id="spray_arm")`

Ask: "Dispatch emergency response to coordinates"
- Use: `environmental_logistics_handler(operation="emergency_dispatch", emergency_type="medical", location={"x": 10.0, "y": 5.0})`

Ask: "Check on human health supervision"
- Use: `environmental_logistics_handler(operation="human_health_supervision", subject_id="elderly_01")`

Ask: "Interact with the companion robot"
- Use: `environmental_logistics_handler(operation="companion_logistics")`

---

## Tutorial 11: Fleet Art Bridges

Ask: "Check gimp-mcp status for sim art"
- Use: `robotics_sim_art(operation="gimp_status")`

Ask: "Generate Gazebo model icons from a directory"
- Use: `robotics_sim_art(operation="batch_gazebo_icons", input_dir="D:/Models/Gazebo", template_id="gazebo_icon_256")`

Ask: "Generate VRChat icons for avatar thumbnails"
- Use: `robotics_sim_art(operation="batch_vrchat_icons", input_dir="D:/Models/VRChat")`

Ask: "Create an avatar thumbnail"
- Use: `robotics_sim_art(operation="avatar_thumbnail", avatar_id="av_001", icon_path="D:/Icons/robot_thumb.png")`

Ask: "Check inkscape-mcp fab art status"
- Use: `robotics_fab_art(operation="inkscape_status")`

Ask: "Batch export DXF files for laser cutting"
- Use: `robotics_fab_art(operation="batch_dxf_export", input_dir="D:/Designs/SVG", output_dir="D:/Designs/DXF")`

Ask: "Generate a Gazebo schematic from SVG"
- Use: `robotics_fab_art(operation="gazebo_schematic", svg_path="D:/Designs/robot_base.svg", preset_id="gazebo_model_doc_192", output_dir="D:/Designs/Schematics")`

---

## Tutorial 12: Agentic Workflows

Ask: "Plan a multi-robot cleaning mission using AI sampling"
- Use: `robotics_agentic_workflow(thought_process="Need to coordinate multiple robots for efficient cleaning", objective="Clean the entire apartment with dreame_01 and scout patrol")`

Ask: "Orchestrate a security sweep using MCP sampling"
- Use: `robotics_agentic_workflow(thought_process="Multi-step security check needed", objective="Patrol perimeter, check all doors, report to human")`

---

## Tutorial 13: CAD and SPZ File Conversion

Ask: "Convert a STEP file to OBJ for Blender"
- Use: `cad_converter(operation="convert", input_path="D:/CAD/robot_arm.step", output_format="obj", output_path="D:/Models/robot_arm.obj")`

Ask: "Convert a STEP file to STL for 3D printing"
- Use: `cad_converter(operation="convert", input_path="D:/CAD/bracket.stp", output_format="stl", output_path="D:/Print/bracket.stl", scale=1.0)`

Ask: "List supported CAD formats"
- Use: `cad_converter(operation="list_supported_formats")`

Ask: "Check an SPZ file for the Gaussian splat project"
- Use: `spz_converter(operation="spz_check", spz_path="D:/Splats/capture.spz")`

Ask: "Convert SPZ to PLY format"
- Use: `spz_converter(operation="spz_convert", spz_path="D:/Splats/capture.spz", output_path="D:/Splats/capture.ply", output_format="ply")`

---

## Tutorial 14: Noetix Bumi Humanoid

Ask: "Tell me about the Noetix Bumi robot"
- Use: `noetix_info(operation="info")`
- Returns: 94 cm, 12 kg, 21 DOF, 0.5 m/s, ROS/ROS2, Python/C++, ~$1,370

Ask: "Get Noetix Bumi SDK links"
- Use: `noetix_info(operation="info")` - Check github field for SDK repos

---

## Tutorial 15: Multi-Robot Coordination

Ask: "List all robots to see what's available"
- Use: `robotics_system(operation="list_robots")`

Ask: "Get status of all Dreame and Scout robots"
- Use: `robotics_system(operation="list_robots", robot_type="dreame")`
- Then: `robotics_system(operation="list_robots", robot_type="scout")`

Ask: "Simultaneously check all robot batteries"
- Use: Call robot_control(action="get_status") for each robot_id found in list_robots

Ask: "Coordinate a multi-robot cleaning + patrol mission"
- Use: `robotics_agentic_workflow(thought_process="Need to clean floors while also patrolling", objective="dreame_01 cleans all rooms, yahboom_01 patrols perimeter simultaneously")`

---

## Tutorial 16: Virtual-First Development

Ask: "Test navigation before hardware arrives"
- Use: `robot_virtual(operation="create", robot_type="scout", platform="unity")`
- Then: `robot_virtual(operation="load_environment", environment="test_room", platform="unity")`
- Then: `robot_virtual(operation="get_status", robot_id="vbot_scout_01")`
- Then: `robot_virtual(operation="test_navigation", robot_id="vbot_scout_01")`

Ask: "Compare robot sizes in VR"
- Use: `robot_virtual(operation="create", robot_type="scout", platform="unity", position={"x": 0, "y": 0, "z": 0})`
- Then: `robot_virtual(operation="create", robot_type="go2", platform="unity", position={"x": 2, "y": 0, "z": 0})`
- Then: `robot_virtual(operation="create", robot_type="robbie", platform="unity", position={"x": 4, "y": 0, "z": 0})`
- Then: `robot_virtual(operation="create", robot_type="godzilla", platform="unity", position={"x": 6, "y": 0, "z": 0})`

---

## Tutorial 17: Combined Multi-Tool Workflows

These tutorials chain several portmanteau tools together for realistic end-to-end tasks that
a single call can't cover on its own.

### Physical-to-Virtual Digital Twin

Ask: "Map my apartment with the Dreame, then bring it into Unity for navigation testing"
- Use: `robot_control(robot_id="dreame_01", action="start_mapping")`
- Wait for mapping to complete, then: `robot_control(robot_id="dreame_01", action="get_map")`
  to confirm the map is ready
- Then: `dreame_control(operation="export_map", format="unity", output_path="D:/Maps/apartment_navmesh.json")`
- Then: `robot_virtual(operation="load_environment", environment="D:/Maps/apartment_navmesh.json", platform="unity")`
- Then: `robot_virtual(operation="create", robot_type="scout", platform="unity")` to spawn a
  test robot into the freshly-imported real floor plan
- Then: `robot_virtual(operation="test_navigation", robot_id="vbot_scout_01")`

### Manufacturing-to-Model Pipeline

Ask: "I have a CAD file for a robot bracket - get it ready for both 3D printing and a
virtual-twin model"
- Use: `cad_converter(operation="convert", input_path="D:/CAD/bracket.step", output_format="stl", output_path="D:/Print/bracket.stl")` for the printable version
- Then: `cad_converter(operation="convert", input_path="D:/CAD/bracket.step", output_format="obj", output_path="D:/Models/bracket.obj")` for the visual/virtual-twin version
- Then: `robot_manufacturing(device_id="printer_01", device_type="3d_printer", category="control", action="print_file", file_path="D:/Print/bracket.stl")`
- Then, once printed: `robot_model(operation="import", robot_type="custom", model_path="D:/Models/bracket.obj")`

### Sim-to-Real Validation

Ask: "I want to test a new patrol route in VR before running it on the real Yahboom"
- Use: `robot_virtual(operation="create", robot_type="yahboom", platform="unity")` to spawn a
  virtual twin
- Then: `robot_behavior(robot_id="vbot_yahboom_01", category="navigation", action="plan_path", start_position={"x": 0, "y": 0, "z": 0}, goal_position={"x": 10, "y": 0, "z": 5})`
- Then: `robot_behavior(robot_id="vbot_yahboom_01", category="navigation", action="follow_path", path_id="<returned path_id>")`
- Once the virtual run looks correct: `robot_control(robot_id="yahboom_01", action="home_patrol", patrol_route="<same route>")` on the physical robot

### Full Nori A3 Session with Error Recovery

Ask: "Set up a Nori A3 recording session, and handle it gracefully if norirobotics-mcp isn't
running yet"
- Use: `robot_control(robot_id="nori_01", action="get_status")` first as a cheap
  reachability check
- If the response is an error with `error_type="not_available"`, the message will say
  "norirobotics-mcp not reachable" - start that server separately and retry, rather than
  retrying the same connect call in a loop
- Once reachable: `robot_control(robot_id="nori_01", action="connect")`
- Then proceed with `episode_start`/`episode_stop` as in Tutorial 1b

### Fleet-Wide Status Check

Ask: "Give me a full status report across every robot and simulation backend"
- Use: `robotics_system(operation="status")` for the server's own health and mounted-server
  connectivity
- Then: `robotics_system(operation="list_robots")` for every registered robot
- Then: `sim_fleet_status()` for every simulation backend's availability
- Then, for each physical robot returned by list_robots: `robot_control(robot_id="<id>", action="get_status")` to get live per-robot telemetry, since list_robots itself only returns
  registration metadata, not live status

---

## Tutorial 18: Understanding Response Formats and Error Types

Every tool in this server returns a structured dict, either a success shape
(`{"success": true, "message": "...", "data": {...}}` from most tools, or the
`{"status": "success", ...}` shape used by `robotics_system` and `robot_control`'s Nori A3
handler) or an error shape carrying an `error_type` field that tells you WHY it failed, not
just THAT it failed - worth checking before assuming a generic retry will help.

Ask: "What does error_type 'not_available' mean?"
- It means a required mounted server or bridge (e.g. `norirobotics-mcp` for Nori A3,
  `unity3d-mcp`'s Editor bridge for virtual robots) isn't reachable right now. Retrying the
  exact same call won't help - the downstream service needs to actually be started first.
  `robotics_system(operation="status")`'s `mounted_servers`/component flags tell you which
  ones are currently connected.

Ask: "What does error_type 'not_found' mean?"
- The `robot_id` you passed isn't registered. Use
  `robotics_system(operation="list_robots")` to see what's actually available - a typo in the
  ID is the most common cause, followed by trying to control a robot that was never added to
  the config.

Ask: "What does error_type 'validation_error' mean?"
- A parameter is malformed or out of range for the action - e.g. an unsupported
  `device_type` for `robot_manufacturing`, or a `suction_level` outside 1-4 for Dreame. The
  error message names the specific field.

Ask: "What does error_type 'connection_error' mean, and how is it different from
'not_available'?"
- `connection_error` means the target robot/device itself refused or dropped the connection
  (network issue, robot powered off, wrong IP) - the bridge/mounted-server infrastructure IS
  reachable, but the specific robot behind it isn't. `not_available` means the
  infrastructure itself (the bridge/mounted server) isn't reachable at all. Distinguishing
  these tells you whether to check the robot's power/network, or check whether the
  peer MCP server process is even running.

Ask: "What does error_type 'timeout_error' mean, and should I retry immediately?"
- The operation started but didn't complete within the expected window - common for
  long-running physical operations (mapping, milling, printing) where a naive fixed timeout
  doesn't match real-world duration. Check the operation's own status/progress action first
  (`get_progress`, `get_print_status`, `get_map`) before retrying the original call, since
  the operation may actually still be in progress rather than genuinely stuck.

---

## Best Practices

1. Always check robot status before controlling: use robot_control(robot_id="...", action="get_status")
2. Use virtual-first development: test in Unity before hardware arrives
3. Use robotics_system(operation="help") to discover all available tools
4. Use robotics_system(operation="status") to check server health
5. Filter with list_robots to find specific robots quickly
6. Set duration on move operations to prevent runaway robots
7. Use emergency stop (action="stop") for any robot type
8. Check Sim Fleet status before routing simulation tasks
9. Dreame operations use DreameHome cloud - credentials required
10. Drone operations require GPS lock and adequate battery
11. Manufacturing operations: verify material compatibility before starting
12. Laser cutting requires ventilation and fire safety measures
13. SPZ files can exceed 100MB - plan output locations accordingly
14. Workflow templates provide reusable mission patterns
15. Use the agentic workflow tool for complex multi-step missions
16. Nori A3 sessions default to mock mode - always check the `"mock"` field in a `get_status`
    response before assuming a command reached real hardware
17. Use `episode_start`'s `query` parameter to give each recorded demonstration a clear,
    specific task description - it becomes the permanent label on that dataset entry, not
    just a transient log line
18. Prefer `stop` (a real e-stop) over `disconnect` when you need to halt in-progress Nori A3
    motion immediately - `disconnect` just closes the session, it doesn't guarantee the robot
    stops moving first
19. When chaining several robots in one multi-step task, check each one's status
    independently rather than assuming a prior success implies the next robot in the sequence
    is also ready
20. For CAD/model conversion pipelines, treat `output_path` as a hint from the caller, not
    a guarantee - if a downstream tool call fails to find the file, double-check the actual
    returned path in the conversion tool's response rather than assuming the path you passed
    in was used verbatim

## Error Recovery

- "Robot not found": Use robotics_system(operation="list_robots") to check registered robots
- "Connection error": Check robot is powered, on network, and config is correct
- "Not available": Check mounted server status with robotics_system(operation="status")
- "Dreame auth failed": Verify DREAME_USER and DREAME_PASSWORD environment variables
- "Drone pre-arm failed": Check GPS lock, battery level, and sensor calibration
- "Sim backend unavailable": Check sim_fleet_status() for which backends are running
- "File not found": Verify model/gcode/config path is correct and accessible
- "Timeout": Operation taking too long - check robot connectivity and retry
- "Unsupported device type" (manufacturing): device_type must be exactly `3d_printer`,
  `cnc_machine`, or `laser_cutter` - this returns a proper validation_error immediately
  rather than silently no-opping, so a typo here is easy to spot from the error message alone
- "Nori A3 status check failed" / "Nori A3 connect failed": the bridge reached
  norirobotics-mcp but it returned an error - check norirobotics-mcp's own logs, since the
  problem is on that side, not in the HTTP bridge itself
- "Nori A3 episode_start/episode_stop failed": usually means no session is open (call
  `connect` first) or the recording daemon on the norirobotics-mcp side hit a storage/state
  issue - check `free_gb` in the last successful status response before retrying
- "Unsupported action for Nori A3": only get_status/connect/disconnect/stop/episode_start/
  episode_stop are valid actions for `robot_type="nori_a3"` - other action names silently
  don't apply, even ones that are valid for other robot types like Yahboom or Dreame
- Virtual robot "created" but nothing visible in the platform: this server's own
  bookkeeping succeeding does not guarantee the platform-side spawn rendered - check the
  platform bridge's own connection status and, for Unity, the Editor's own scene hierarchy
  directly

---

## A Note on Keeping This Guide Current

These tutorials are drawn directly from the tool signatures and live-verified behavior in
`src/robotics_mcp/tools/`, not from an idealized spec - when a Nori A3 example or an error
message here stops matching what a tool actually returns, that's a real drift to fix, not a
stylistic choice to leave alone. If you're extending this server with a new robot type or
tool, add a matching Ask/Use pair here in the same change, following the pattern already
established for the fifteen-plus robot and device types already documented above: a short
Ask/Use pair for each realistic operation, real parameter names copied from the actual tool
signature, and honest notes about mock-mode defaults or platform-specific caveats where they
genuinely exist, rather than a generic template filled in with placeholder text.
