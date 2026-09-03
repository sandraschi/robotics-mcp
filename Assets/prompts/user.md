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

## Error Recovery

- "Robot not found": Use robotics_system(operation="list_robots") to check registered robots
- "Connection error": Check robot is powered, on network, and config is correct
- "Not available": Check mounted server status with robotics_system(operation="status")
- "Dreame auth failed": Verify DREAME_USER and DREAME_PASSWORD environment variables
- "Drone pre-arm failed": Check GPS lock, battery level, and sensor calibration
- "Sim backend unavailable": Check sim_fleet_status() for which backends are running
- "File not found": Verify model/gcode/config path is correct and accessible
- "Timeout": Operation taking too long - check robot connectivity and retry
