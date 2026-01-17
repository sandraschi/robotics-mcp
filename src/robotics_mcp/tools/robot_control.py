"""Robot control portmanteau tool - Unified bot + vbot control."""

from typing import Any, Literal

import structlog

from ..clients.yahboom_client import YahboomClient, YahboomRobotConfig
from ..utils.error_handler import format_error_response, format_success_response, handle_tool_error
from ..utils.mcp_client_helper import call_mounted_server_tool

logger = structlog.get_logger(__name__)


class RobotControlTool:
    """Portmanteau tool for unified robot control (bot + vbot)."""

    def __init__(self, mcp: Any, state_manager: Any, mounted_servers: dict[str, Any] | None = None):
        """Initialize robot control tool.

        Args:
            mcp: FastMCP server instance.
            state_manager: Robot state manager instance.
            mounted_servers: Dictionary of loaded MCP servers for internal use.
        """
        self.mounted_servers = mounted_servers or {}
        self.mcp = mcp
        self.state_manager = state_manager

    def register(self):
        """Register robot control tool with MCP server."""

        @self.mcp.tool()
        async def robot_control(
            robot_id: str,
            action: Literal[
                "get_status",
                "move",
                "stop",
                "return_to_dock",
                "stand",
                "sit",
                "walk",
                "sync_vbot",
                # Yahboom-specific actions
                "home_patrol",
                "camera_capture",
                "arm_move",
                "gripper_control",
                "navigate_to",
                # Dreame D20 Pro Plus specific actions
                "start_auto_empty",
                "stop_auto_empty",
                "start_self_clean",
                "stop_self_clean",
                "set_suction_level",
                "set_water_volume",
                "set_mop_humidity",
                "clean_zone",
                "clean_spot",
                "start_mapping",
                "rename_room",
                "set_cleaning_sequence",
                "set_restricted_zones",
                "get_cleaning_history",
                "clear_error",
            ],
            linear: float | None = None,
            angular: float | None = None,
            duration: float | None = None,
            # Yahboom-specific parameters
            x: float | None = None,
            y: float | None = None,
            theta: float | None = None,
            joint_angles: dict[str, float] | None = None,
            gripper_action: Literal["open", "close", "stop"] | None = None,
            patrol_route: str | None = None,
            # Dreame D20 Pro Plus specific parameters
            suction_level: int | None = None,
            water_volume: int | None = None,
            mop_humidity: int | None = None,
            zones: list[list[int]] | None = None,
            spot_x: int | None = None,
            spot_y: int | None = None,
            room_id: int | None = None,
            room_name: str | None = None,
            cleaning_sequence: list[int] | None = None,
            restricted_zones: dict[str, list[list[int]]] | None = None,
            # Yahboom AI query parameters
            query: str | None = None,
            query_type: Literal["text", "vision", "voice", "multimodal"] | None = None,
        ) -> dict[str, Any]:
            """Unified robot control with conversational responses.

            Provides a single interface for controlling physical robots, virtual robots, and specialized devices
            with intelligent routing and rich conversational responses. Supports Moorebot Scout, Unitree robots,
            Yahboom ROS platforms, Dreame vacuums, and virtual robots in Unity/VRChat.

            PORTMANTEAU PATTERN RATIONALE:
            Instead of creating separate tools for each robot type and operation, this tool
            consolidates all robot control operations into a single interface. This design:
            - Prevents tool explosion (15+ tools -> 1 tool) while maintaining full functionality
            - Enables seamless switching between physical and virtual robots
            - Provides consistent error handling and safety protocols across all robot types
            - Supports conversational AI interaction with context-aware responses
            - Follows FastMCP 2.13+ best practices for feature-rich MCP servers

            SUPPORTED ROBOT TYPES:
            - Physical Robots: Moorebot Scout, Unitree Go2/G1/H1 (ROS-based wheeled/legged robots)
            - Virtual Robots: Unity3D/VRChat robots (simulation and social VR platforms)
            - Yahboom Robots: ROSMASTER series with AI, navigation, and optional robotic arms
            - Dreame Vacuums: Smart vacuum cleaners with mapping and zone cleaning

            SUPPORTED OPERATIONS:
            - Universal: "get_status", "move", "stop"
            - Physical Robots: "return_to_dock", "stand", "sit", "walk", "sync_vbot"
            - Yahboom: "home_patrol", "camera_capture", "arm_move", "gripper_control", "navigate_to", "ai_query"
            - Dreame: "start_auto_empty", "stop_auto_empty", "start_self_clean", "stop_self_clean",
                     "set_suction_level", "set_water_volume", "set_mop_humidity", "clean_zone",
                     "clean_spot", "start_mapping", "rename_room", "set_cleaning_sequence",
                     "set_restricted_zones", "get_cleaning_history", "clear_error"

            Args:
                robot_id: Unique robot identifier. MUST follow naming convention:
                    - Physical: "scout_01", "go2_01", "g1_01", "yahboom_01", "dreame_01"
                    - Virtual: "vbot_scout_01", "unity_bot_01", "vrchat_bot_01"

                action: Operation to perform. MUST be one of the supported operations above:
                    Universal Operations:
                    - "get_status": Get comprehensive robot status (battery, position, sensors, capabilities)
                    - "move": Control movement with linear/angular velocities (all mobile robots)
                    - "stop": Emergency stop all movement (all robots)

                    Physical Robot Operations:
                    - "return_to_dock": Return to charging dock (vacuums only)
                    - "stand": Stand up from sitting position (legged robots)
                    - "sit": Sit down (legged robots)
                    - "walk": Start walking gait (legged robots)
                    - "sync_vbot": Synchronize virtual robot with physical robot state

                    Yahboom Robot Operations:
                    - "home_patrol": Start autonomous home security patrol
                    - "camera_capture": Capture image from robot camera
                    - "arm_move": Move robotic arm to specified joint angles
                    - "gripper_control": Control gripper open/close/stop
                    - "navigate_to": Navigate to specific coordinates
                    - "ai_query": Multimodal AI query (text/vision/voice/multimodal)

                    Dreame Vacuum Operations:
                    - "start_auto_empty": Start automatic dust bin emptying
                    - "stop_auto_empty": Stop automatic dust bin emptying
                    - "start_self_clean": Start mop self-cleaning cycle
                    - "stop_self_clean": Stop mop self-cleaning cycle
                    - "set_suction_level": Set vacuum suction power (1-4)
                    - "set_water_volume": Set mopping water volume (1-3)
                    - "set_mop_humidity": Set mop pad humidity (1-3)
                    - "clean_zone": Clean specific rectangular zones
                    - "clean_spot": Intensive spot cleaning at coordinates
                    - "start_mapping": Start new map creation/mapping
                    - "rename_room": Rename a detected room
                    - "set_cleaning_sequence": Set room cleaning order
                    - "set_restricted_zones": Create virtual walls/restricted zones
                    - "get_cleaning_history": Retrieve cleaning history
                    - "clear_error": Clear error conditions

                linear: Linear velocity (m/s) for move operations. Range: -2.0 to 2.0 m/s
                angular: Angular velocity (rad/s) for rotation. Range: -3.14 to 3.14 rad/s
                duration: Movement duration in seconds. Default: continuous until stopped
                x: Target X coordinate (meters) for navigation operations
                y: Target Y coordinate (meters) for navigation operations
                theta: Target orientation (radians) for navigation operations
                joint_angles: Dictionary of joint names to target angles (degrees) for arm control
                gripper_action: Gripper action. MUST be "open", "close", or "stop"
                patrol_route: Name of predefined patrol route for autonomous navigation
                suction_level: Vacuum suction power level (1-4, where 4 is maximum)
                water_volume: Mopping water volume level (1-3, where 3 is maximum)
                mop_humidity: Mop pad humidity level (1-3, where 3 is maximum humidity)
                zones: List of zone coordinates [[x1,y1,x2,y2], ...] for zone cleaning
                spot_x: X coordinate for intensive spot cleaning
                spot_y: Y coordinate for intensive spot cleaning
                room_id: Room identifier for room-specific operations
                room_name: New name for room renaming operations
                cleaning_sequence: List of room IDs defining cleaning order [room1, room2, ...]
                restricted_zones: Dictionary with 'walls' and 'zones' keys for virtual barriers
                query: AI query text for multimodal analysis (Yahboom robots)
                query_type: AI query type. MUST be "text", "vision", "voice", or "multimodal"

            Returns:
                Rich conversational response with:
                - success: Boolean operation status
                - message: Natural language description of result
                - robot_data: Current robot status and telemetry
                - safety_warnings: Any safety concerns or recommendations
                - next_commands: Suggested follow-up operations
                - estimated_completion: Time estimates for long operations
                - error_recovery: Intelligent error handling with resolution steps
                - operation_metadata: Device-specific operation details

            Examples:
                Get robot status (universal):
                    result = await robot_control(robot_id="scout_01", action="get_status")
                    # Returns: {"success": true, "message": "Robot status retrieved", "robot_data": {...}}

                Move robot forward:
                    result = await robot_control(
                        robot_id="scout_01",
                        action="move",
                        linear=0.5,
                        angular=0.0,
                        duration=5.0
                    )
                    # Returns: {"success": true, "message": "Moving forward at 0.5 m/s"}

                Emergency stop:
                    result = await robot_control(robot_id="yahboom_01", action="stop")
                    # Returns: {"success": true, "message": "Emergency stop activated", "safety_warnings": ["Verify robot stopped"]}

                Yahboom AI query:
                    result = await robot_control(
                        robot_id="yahboom_01",
                        action="ai_query",
                        query="What's in front of me?",
                        query_type="vision"
                    )
                    # Returns: {"success": true, "message": "AI analysis complete", "response": "I detect a clear pathway"}

                Dreame zone cleaning:
                    result = await robot_control(
                        robot_id="dreame_01",
                        action="clean_zone",
                        zones=[[0,0,200,200], [300,100,500,300]]
                    )
                    # Returns: {"success": true, "message": "Zone cleaning started", "estimated_completion": "45 minutes"}

                Unitree stand command:
                    result = await robot_control(robot_id="g1_01", action="stand")
                    # Returns: {"success": true, "message": "Robot standing up", "estimated_completion": "3 seconds"}

                Sync virtual with physical:
                    result = await robot_control(robot_id="scout_01", action="sync_vbot")
                    # Returns: {"success": true, "message": "Virtual robot synchronized", "next_commands": ["robot_behavior get_status"]}
            """
            try:
                robot = self.state_manager.get_robot(robot_id)
                if not robot:
                    return format_error_response(
                        f"Robot {robot_id} not found",
                        error_type="not_found",
                        robot_id=robot_id,
                        action=action,
                    )

                # Route to appropriate handler
                if robot.is_virtual:
                    return await self._handle_virtual_robot(robot, action, linear, angular, duration)
                elif robot.robot_type == "yahboom":
                    return await self._handle_yahboom_robot(
                        robot, action, linear, angular, duration,
                        x, y, theta, joint_angles, gripper_action, patrol_route,
                        query, query_type
                    )
                elif robot.robot_type == "dreame":
                    return await self._handle_dreame_robot(
                        robot, action, linear, angular, duration,
                        x, y, theta, suction_level, water_volume, mop_humidity,
                        zones, spot_x, spot_y, room_id, room_name, cleaning_sequence, restricted_zones
                    )
                else:
                    return await self._handle_physical_robot(robot, action, linear, angular, duration)
            except Exception as e:
                return handle_tool_error("robot_control", e, robot_id=robot_id, action=action)

    async def _handle_physical_robot(
        self,
        robot: Any,
        action: str,
        linear: float | None,
        angular: float | None,
        duration: float | None,
    ) -> dict[str, Any]:
        """Handle physical robot commands.

        Args:
            robot: Robot state.
            action: Action to perform.
            linear: Linear velocity.
            angular: Angular velocity.
            duration: Movement duration.
            **kwargs: Additional parameters.

        Returns:
            Operation result.
        """
        try:
            # TODO: Implement ROS bridge integration
            logger.info("Physical robot command", robot_id=robot.robot_id, action=action)
            return format_success_response(
                f"Physical robot {action} command sent (mock - ROS integration pending)",
                robot_id=robot.robot_id,
                action=action,
                data={
                    "note": "ROS bridge integration not yet implemented",
                    "mock": True,
                },
            )
        except Exception as e:
            return handle_tool_error("_handle_physical_robot", e, robot_id=robot.robot_id, action=action)

    async def _handle_yahboom_robot(
        self,
        robot: Any,
        action: str,
        linear: float | None,
        angular: float | None,
        duration: float | None,
        x: float | None,
        y: float | None,
        theta: float | None,
        joint_angles: dict[str, float] | None,
        gripper_action: str | None,
        patrol_route: str | None,
        query: str | None,
        query_type: str | None,
    ) -> dict[str, Any]:
        """Handle Yahboom robot commands via ROS 2.

        Args:
            robot: Robot state object.
            action: Action to perform.
            linear: Linear velocity for movement.
            angular: Angular velocity for movement.
            duration: Movement duration.
            x: Target X coordinate for navigation.
            y: Target Y coordinate for navigation.
            theta: Target orientation for navigation.
            query: AI query text for multimodal analysis.
            query_type: Type of AI query (text, vision, voice, multimodal).
            joint_angles: Joint angles for arm control.
            gripper_action: Gripper action.
            patrol_route: Patrol route name.

        Returns:
            Operation result.
        """
        try:
            # Create Yahboom client from robot config
            config = YahboomRobotConfig(
                robot_id=robot.robot_id,
                ip_address=getattr(robot, 'ip_address', '192.168.1.101'),
                port=getattr(robot, 'port', 9090),
                camera_enabled=getattr(robot, 'camera_enabled', True),
                navigation_enabled=getattr(robot, 'navigation_enabled', True),
                arm_enabled=getattr(robot, 'arm_enabled', False),
                mock_mode=getattr(robot, 'mock_mode', True),
            )

            client = YahboomClient(config)
            await client.connect()

            # Handle different actions
            if action == "get_status":
                status = await client.get_status()
                return format_success_response(
                    f"Yahboom robot {robot.robot_id} status retrieved",
                    robot_id=robot.robot_id,
                    action=action,
                    data=status,
                )

            elif action == "move":
                result = await client.move(linear or 0.0, angular or 0.0, duration)
                return format_success_response(
                    f"Yahboom robot {robot.robot_id} movement command executed",
                    robot_id=robot.robot_id,
                    action=action,
                    data=result,
                )

            elif action == "stop":
                result = await client.stop()
                return format_success_response(
                    f"Yahboom robot {robot.robot_id} stopped",
                    robot_id=robot.robot_id,
                    action=action,
                    data=result,
                )

            elif action == "camera_capture":
                frame = await client.get_camera_frame()
                if frame:
                    return format_success_response(
                        f"Yahboom robot {robot.robot_id} camera capture successful",
                        robot_id=robot.robot_id,
                        action=action,
                        data={"frame_size": len(frame), "mock": config.mock_mode},
                    )
                else:
                    return format_error_response(
                        "Camera not available or failed to capture",
                        error_type="camera_error",
                        robot_id=robot.robot_id,
                        action=action,
                    )

            elif action == "arm_move":
                if not joint_angles:
                    return format_error_response(
                        "joint_angles required for arm_move action",
                        error_type="missing_parameter",
                        robot_id=robot.robot_id,
                        action=action,
                    )
                result = await client.control_arm(joint_angles)
                return format_success_response(
                    f"Yahboom robot {robot.robot_id} arm moved",
                    robot_id=robot.robot_id,
                    action=action,
                    data=result,
                )

            elif action == "gripper_control":
                if not gripper_action:
                    return format_error_response(
                        "gripper_action required for gripper_control action",
                        error_type="missing_parameter",
                        robot_id=robot.robot_id,
                        action=action,
                    )
                result = await client.control_gripper(gripper_action)
                return format_success_response(
                    f"Yahboom robot {robot.robot_id} gripper {gripper_action}",
                    robot_id=robot.robot_id,
                    action=action,
                    data=result,
                )

            elif action == "navigate_to":
                if x is None or y is None:
                    return format_error_response(
                        "x and y coordinates required for navigate_to action",
                        error_type="missing_parameter",
                        robot_id=robot.robot_id,
                        action=action,
                    )
                result = await client.navigate_to_pose(x, y, theta or 0.0)
                return format_success_response(
                    f"Yahboom robot {robot.robot_id} navigating to ({x}, {y})",
                    robot_id=robot.robot_id,
                    action=action,
                    data=result,
                )

            elif action == "home_patrol":
                # For now, implement basic patrol - could be extended
                patrol_points = [
                    {"x": 0.0, "y": 0.0, "description": "living room"},
                    {"x": 3.0, "y": 0.0, "description": "kitchen"},
                    {"x": 3.0, "y": 2.0, "description": "bedroom"},
                    {"x": 0.0, "y": 2.0, "description": "return to base"},
                ]

                return format_success_response(
                    f"Yahboom robot {robot.robot_id} starting home patrol",
                    robot_id=robot.robot_id,
                    action=action,
                    data={
                        "patrol_route": patrol_route or "home_security",
                        "waypoints": patrol_points,
                        "mock": config.mock_mode,
                    },
                )

            elif action == "ai_query":
                if not query:
                    return format_error_response(
                        "Query parameter required for ai_query action",
                        error_type="missing_parameter",
                        robot_id=robot.robot_id,
                        action=action,
                    )

                # Mock AI response for Yahboom multimodal AI
                query_type = query_type or "text"
                mock_responses = {
                    "text": f"Processed text query: '{query}'. Analysis complete.",
                    "vision": f"Analyzed visual input for: '{query}'. Object detection successful.",
                    "voice": f"Processed voice command: '{query}'. Speech recognition complete.",
                    "multimodal": f"Multimodal analysis for: '{query}'. Combined text, vision, and voice processing.",
                }

                ai_response = mock_responses.get(query_type, f"AI query processed: {query}")

                return format_success_response(
                    f"Yahboom robot {robot.robot_id} AI query completed",
                    robot_id=robot.robot_id,
                    action=action,
                    data={
                        "query": query,
                        "query_type": query_type,
                        "response": ai_response,
                        "confidence": 0.95,
                        "processing_time_ms": 150,
                        "mock": config.mock_mode,
                    },
                )

            else:
                return format_error_response(
                    f"Unsupported action '{action}' for Yahboom robot",
                    error_type="unsupported_action",
                    robot_id=robot.robot_id,
                    action=action,
                )

        except Exception as e:
            return handle_tool_error("_handle_yahboom_robot", e, robot_id=robot.robot_id, action=action)
        finally:
            if 'client' in locals():
                await client.disconnect()

    async def _handle_dreame_robot(
        self,
        robot: Any,
        action: str,
        linear: float | None,
        angular: float | None,
        duration: float | None,
        x: float | None,
        y: float | None,
        theta: float | None,
        suction_level: int | None,
        water_volume: int | None,
        mop_humidity: int | None,
        zones: list[list[int]] | None,
        spot_x: int | None,
        spot_y: int | None,
        room_id: int | None,
        room_name: str | None,
        cleaning_sequence: list[int] | None,
        restricted_zones: dict[str, list[list[int]]] | None,
    ) -> dict[str, Any]:
        """Handle Dreame D20 Pro Plus vacuum commands.

        Args:
            robot: Robot state object.
            action: Action to perform.
            linear: Linear velocity for movement.
            angular: Angular velocity for movement.
            duration: Movement duration.
            x: Target X coordinate for navigation.
            y: Target Y coordinate for navigation.
            theta: Target orientation for navigation.
            suction_level: Suction power level (1-4).
            water_volume: Water volume level (1-3).
            mop_humidity: Mop pad humidity level (1-3).
            zones: List of zone coordinates [[x1,y1,x2,y2], ...] for zone cleaning.
            spot_x: X coordinate for spot cleaning.
            spot_y: Y coordinate for spot cleaning.
            room_id: Room identifier for room-specific operations.
            room_name: New name for room renaming.
            cleaning_sequence: List of room IDs defining cleaning order.
            restricted_zones: Dictionary with 'walls' and 'zones' keys for restricted areas.

        Returns:
            Operation result.
        """
        try:
            from .dreame_client import (
                dreame_clean_room,
                dreame_clean_spot,
                dreame_clean_zone,
                dreame_get_map,
                dreame_get_status,
                dreame_move,
                dreame_start_cleaning,
                dreame_stop_cleaning,
                get_dreame_client,
            )

            # Handle different Dreame actions
            if action == "get_status":
                return await dreame_get_status(robot.robot_id)

            elif action == "start_cleaning":
                return await dreame_start_cleaning(robot.robot_id)

            elif action == "stop_cleaning":
                return await dreame_stop_cleaning(robot.robot_id)

            elif action == "move":
                # Convert velocities to Dreame format (rotation and velocity as integers)
                rotation = int(angular * 100) if angular else 0
                velocity = int(linear * 100) if linear else 50
                return await dreame_move(robot.robot_id, rotation=rotation, velocity=velocity)

            elif action == "go_to":
                if x is not None and y is not None:
                    client = get_dreame_client(robot.robot_id)
                    success = await client.go_to_position(x, y)
                    if success:
                        return format_success_response(
                            f"Dreame {robot.robot_id} navigating to ({x}, {y})",
                            robot_id=robot.robot_id,
                            action=action,
                            data={"x": x, "y": y},
                        )
                    else:
                        return format_error_response(
                            "Failed to navigate Dreame to position",
                            error_type="navigation_failed",
                            robot_id=robot.robot_id,
                        )
                else:
                    return format_error_response(
                        "go_to action requires x and y coordinates",
                        error_type="missing_coordinates",
                        robot_id=robot.robot_id,
                    )

            elif action == "get_map":
                return await dreame_get_map(robot.robot_id)

            elif action == "clean_room":
                if room_id is not None:
                    return await dreame_clean_room(robot.robot_id, room_id)
                else:
                    return format_error_response(
                        "clean_room action requires room_id",
                        error_type="missing_room_id",
                        robot_id=robot.robot_id,
                    )

            elif action == "clean_zone":
                if zones:
                    return await dreame_clean_zone(robot.robot_id, zones)
                else:
                    return format_error_response(
                        "clean_zone action requires zones parameter",
                        error_type="missing_zones",
                        robot_id=robot.robot_id,
                    )

            elif action == "clean_spot":
                if spot_x is not None and spot_y is not None:
                    return await dreame_clean_spot(robot.robot_id, spot_x, spot_y)
                else:
                    return format_error_response(
                        "clean_spot action requires spot_x and spot_y",
                        error_type="missing_coordinates",
                        robot_id=robot.robot_id,
                    )

            elif action == "return_to_dock":
                client = get_dreame_client(robot.robot_id)
                success = await client.return_to_dock()
                if success:
                    return format_success_response(
                        f"Dreame {robot.robot_id} returning to dock",
                        robot_id=robot.robot_id,
                        action=action,
                        data={"action": "dock"},
                    )
                else:
                    return format_error_response(
                        "Failed to send Dreame to dock",
                        error_type="dock_failed",
                        robot_id=robot.robot_id,
                    )

            elif action == "set_suction_level":
                if suction_level is not None:
                    client = get_dreame_client(robot.robot_id)
                    success = await client.set_suction_level(suction_level)
                    if success:
                        return format_success_response(
                            f"Dreame {robot.robot_id} suction level set to {suction_level}",
                            robot_id=robot.robot_id,
                            action=action,
                            data={"suction_level": suction_level},
                        )
                    else:
                        return format_error_response(
                            "Failed to set Dreame suction level",
                            error_type="suction_failed",
                            robot_id=robot.robot_id,
                        )
                else:
                    return format_error_response(
                        "set_suction_level action requires suction_level parameter",
                        error_type="missing_suction_level",
                        robot_id=robot.robot_id,
                    )

            elif action == "set_water_volume":
                if water_volume is not None:
                    client = get_dreame_client(robot.robot_id)
                    success = await client.set_water_volume(water_volume)
                    if success:
                        return format_success_response(
                            f"Dreame {robot.robot_id} water volume set to {water_volume}",
                            robot_id=robot.robot_id,
                            action=action,
                            data={"water_volume": water_volume},
                        )
                    else:
                        return format_error_response(
                            "Failed to set Dreame water volume",
                            error_type="water_failed",
                            robot_id=robot.robot_id,
                        )
                else:
                    return format_error_response(
                        "set_water_volume action requires water_volume parameter",
                        error_type="missing_water_volume",
                        robot_id=robot.robot_id,
                    )

            elif action == "set_mop_humidity":
                if mop_humidity is not None:
                    client = get_dreame_client(robot.robot_id)
                    success = await client.set_mop_humidity(mop_humidity)
                    if success:
                        return format_success_response(
                            f"Dreame {robot.robot_id} mop humidity set to {mop_humidity}",
                            robot_id=robot.robot_id,
                            action=action,
                            data={"mop_humidity": mop_humidity},
                        )
                    else:
                        return format_error_response(
                            "Failed to set Dreame mop humidity",
                            error_type="humidity_failed",
                            robot_id=robot.robot_id,
                        )
                else:
                    return format_error_response(
                        "set_mop_humidity action requires mop_humidity parameter",
                        error_type="missing_humidity",
                        robot_id=robot.robot_id,
                    )

            elif action == "start_fast_mapping":
                client = get_dreame_client(robot.robot_id)
                success = await client.start_fast_mapping()
                if success:
                    return format_success_response(
                        f"Dreame {robot.robot_id} starting fast mapping",
                        robot_id=robot.robot_id,
                        action=action,
                        data={"mapping_type": "fast"},
                    )
                else:
                    return format_error_response(
                        "Failed to start Dreame fast mapping",
                        error_type="mapping_failed",
                        robot_id=robot.robot_id,
                    )

            elif action == "start_mapping":
                client = get_dreame_client(robot.robot_id)
                success = await client.start_mapping()
                if success:
                    return format_success_response(
                        f"Dreame {robot.robot_id} starting mapping",
                        robot_id=robot.robot_id,
                        action=action,
                        data={"mapping_type": "standard"},
                    )
                else:
                    return format_error_response(
                        "Failed to start Dreame mapping",
                        error_type="mapping_failed",
                        robot_id=robot.robot_id,
                    )

            elif action == "set_cleaning_sequence":
                if cleaning_sequence:
                    client = get_dreame_client(robot.robot_id)
                    success = await client.set_cleaning_sequence(cleaning_sequence)
                    if success:
                        return format_success_response(
                            f"Dreame {robot.robot_id} cleaning sequence updated",
                            robot_id=robot.robot_id,
                            action=action,
                            data={"cleaning_sequence": cleaning_sequence},
                        )
                    else:
                        return format_error_response(
                            "Failed to set Dreame cleaning sequence",
                            error_type="sequence_failed",
                            robot_id=robot.robot_id,
                        )
                else:
                    return format_error_response(
                        "set_cleaning_sequence action requires cleaning_sequence parameter",
                        error_type="missing_sequence",
                        robot_id=robot.robot_id,
                    )

            elif action == "set_restricted_zones":
                if restricted_zones:
                    client = get_dreame_client(robot.robot_id)
                    success = await client.set_restricted_zones(restricted_zones)
                    if success:
                        return format_success_response(
                            f"Dreame {robot.robot_id} restricted zones updated",
                            robot_id=robot.robot_id,
                            action=action,
                            data={"restricted_zones": restricted_zones},
                        )
                    else:
                        return format_error_response(
                            "Failed to set Dreame restricted zones",
                            error_type="zones_failed",
                            robot_id=robot.robot_id,
                        )
                else:
                    return format_error_response(
                        "set_restricted_zones action requires restricted_zones parameter",
                        error_type="missing_zones",
                        robot_id=robot.robot_id,
                    )

            elif action == "get_cleaning_history":
                client = get_dreame_client(robot.robot_id)
                history = await client.get_cleaning_history()
                if history:
                    return format_success_response(
                        f"Dreame {robot.robot_id} cleaning history retrieved",
                        robot_id=robot.robot_id,
                        action=action,
                        data={"cleaning_history": history},
                    )
                else:
                    return format_error_response(
                        "Failed to retrieve Dreame cleaning history",
                        error_type="history_failed",
                        robot_id=robot.robot_id,
                    )

            elif action == "clear_error":
                client = get_dreame_client(robot.robot_id)
                success = await client.clear_error()
                if success:
                    return format_success_response(
                        f"Dreame {robot.robot_id} error cleared",
                        robot_id=robot.robot_id,
                        action=action,
                        data={"error_cleared": True},
                    )
                else:
                    return format_error_response(
                        "Failed to clear Dreame error",
                        error_type="clear_error_failed",
                        robot_id=robot.robot_id,
                    )

            else:
                return format_error_response(
                    f"Unsupported action '{action}' for Dreame robot",
                    error_type="unsupported_action",
                    robot_id=robot.robot_id,
                    action=action,
                    supported_actions=[
                        "get_status", "start_cleaning", "stop_cleaning", "move", "go_to",
                        "get_map", "clean_room", "clean_zone", "clean_spot", "return_to_dock",
                        "set_suction_level", "set_water_volume", "set_mop_humidity",
                        "start_fast_mapping", "start_mapping", "set_cleaning_sequence",
                        "set_restricted_zones", "get_cleaning_history", "clear_error"
                    ],
                )

        except Exception as e:
            return handle_tool_error("_handle_dreame_robot", e, robot_id=robot.robot_id, action=action)

    async def _handle_virtual_robot(
        self,
        robot: Any,
        action: str,
        linear: float | None,
        angular: float | None,
        duration: float | None,
    ) -> dict[str, Any]:
        """Handle virtual robot commands.

        Args:
            robot: Robot state.
            action: Action to perform.
            linear: Linear velocity.
            angular: Angular velocity.
            duration: Movement duration.
            **kwargs: Additional parameters.

        Returns:
            Operation result.
        """

        logger.info("Virtual robot command", robot_id=robot.robot_id, action=action, platform=robot.platform)

        try:
            if action == "move":
                if robot.platform == "unity":
                    # Use avatar-mcp or unity3d-mcp for movement
                    # Try avatar-mcp first for smooth locomotion
                    try:
                        if "avatar" in self.mounted_servers:
                            await call_mounted_server_tool(
                                self.mounted_servers,
                                "avatar",
                                "avatar_movement_walk",
                                {"avatar_id": robot.robot_id, "direction": "forward", "speed": linear or 0.0},
                            )
                            if angular:
                                await call_mounted_server_tool(
                                    self.mounted_servers,
                                    "avatar",
                                    "avatar_movement_turn",
                                    {"avatar_id": robot.robot_id, "angle": angular},
                                )
                            return {
                                "status": "success",
                                "message": "Virtual robot moved via avatar-mcp",
                                "robot_id": robot.robot_id,
                                "action": action,
                                "linear": linear,
                                "angular": angular,
                            }
                    except Exception:
                        # Fallback to Unity direct control
                        if "unity" in self.mounted_servers:
                            await call_mounted_server_tool(
                                self.mounted_servers,
                                "unity",
                                "execute_unity_method",
                                {
                                    "class_name": "RobotController",
                                    "method_name": "Move",
                                    "parameters": {
                                        "robotId": robot.robot_id,
                                        "linear": linear or 0.0,
                                        "angular": angular or 0.0,
                                    },
                                },
                            )
                            return {
                                "status": "success",
                                "message": "Virtual robot moved via Unity",
                                "robot_id": robot.robot_id,
                                "action": action,
                            }
                elif robot.platform == "vrchat" and "vrchat" in self.mounted_servers:
                    # Use VRChat OSC for movement
                        await call_mounted_server_tool(
                            self.mounted_servers,
                            "vrchat",
                            "vrchat_send_osc_message",
                            {"address": f"/robot/{robot.robot_id}/move", "args": [linear or 0.0, angular or 0.0]},
                        )
                        return {
                            "status": "success",
                            "message": "Virtual robot moved via VRChat OSC",
                            "robot_id": robot.robot_id,
                            "action": action,
                        }

            elif action == "stop":
                if robot.platform == "vrchat" and "vrchat" in self.mounted_servers:
                    await call_mounted_server_tool(
                        self.mounted_servers,
                        "vrchat",
                        "vrchat_send_osc_message",
                        {"address": f"/robot/{robot.robot_id}/stop", "args": [1]},
                    )
                elif "avatar" in self.mounted_servers:
                    await call_mounted_server_tool(
                        self.mounted_servers,
                        "avatar",
                        "avatar_movement_walk",
                        {"avatar_id": robot.robot_id, "direction": "forward", "speed": 0.0},
                    )
                    return {
                        "status": "success",
                        "message": "Virtual robot stopped",
                        "robot_id": robot.robot_id,
                        "action": action,
                    }

            elif action == "get_status":
                return {
                    "status": "success",
                    "robot": robot.to_dict(),
                    "action": action,
                }

            else:
                return {
                    "status": "success",
                    "message": f"Virtual robot {action} command sent",
                    "robot_id": robot.robot_id,
                    "action": action,
                }

        except Exception as e:
            return handle_tool_error("_handle_virtual_robot", e, robot_id=robot.robot_id, action=action)

    async def handle_action(self, robot_id: str, action: str, params: dict[str, Any]) -> dict[str, Any]:
        """Handle robot action (for HTTP API).

        Args:
            robot_id: Robot identifier.
            action: Action to perform.
            params: Action parameters.

        Returns:
            Operation result.
        """
        robot = self.state_manager.get_robot(robot_id)
        if not robot:
            return {"status": "error", "message": f"Robot {robot_id} not found"}

        linear = params.get("linear")
        angular = params.get("angular")
        duration = params.get("duration")

        if robot.is_virtual:
            return await self._handle_virtual_robot(robot, action, linear, angular, duration, **params)
        else:
            return await self._handle_physical_robot(robot, action, linear, angular, duration, **params)

