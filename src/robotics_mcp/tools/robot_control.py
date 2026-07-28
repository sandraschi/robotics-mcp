"""Robot control portmanteau tool - Unified bot + vbot control."""

from typing import Any, Literal

import structlog
from fastmcp import Context

from ..clients.elegoo_client import ElegooClient, ElegooRobotConfig
from ..clients.gazebo_client import GazeboClient, GazeboRobotConfig
from ..clients.yahboom_mcp_client import YahboomMcpClient, mcp_call_succeeded, yahboom_mcp_url
from ..utils.error_handler import (
    format_error_response,
    format_success_response,
    format_unavailable_error,
    handle_tool_error,
)
from ..utils.mcp_client_helper import call_mounted_server_tool
from .hue_client import (
    hue_get_movement_events,
    hue_get_movement_zones,
    hue_get_sensor_status,
)

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
        self.elegoo_clients = {}  # robot_id -> ElegooClient

    def register(self):
        """Register robot control tool with MCP server."""

        @self.mcp.tool()
        async def robot_control(
            ctx: Context,
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
                "ai_query",
                # Dreame D20 Pro Plus specific actions
                "start_cleaning",
                "stop_cleaning",
                "start_auto_empty",
                "stop_auto_empty",
                "start_self_clean",
                "stop_self_clean",
                "set_suction_level",
                "set_water_volume",
                "set_mop_humidity",
                "clean_zone",
                "clean_spot",
                "clean_room",
                "go_to",
                "get_map",
                "start_mapping",
                "start_fast_mapping",
                "rename_room",
                "set_cleaning_sequence",
                "set_restricted_zones",
                "get_cleaning_history",
                "clear_error",
                # Elegoo robot specific actions
                "emergency_stop",
                # Hue HomeAware actions
                "hue_get_movement_events",
                "hue_get_sensor_status",
                "hue_get_movement_zones",
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
            with intelligent routing and rich conversational responses. Primary: Dreame D20 Pro vacuum. Also supports
            Yahboom ROSMASTER, Moorebot Scout, Unitree (when hardware available), and virtual robots in Unity/VRChat.

            PORTMANTEAU PATTERN RATIONALE:
            Instead of creating separate tools for each robot type and operation, this tool
            consolidates all robot control operations into a single interface. This design:
            - Prevents tool explosion (15+ tools -> 1 tool) while maintaining full functionality
            - Enables seamless switching between physical and virtual robots
            - Provides consistent error handling and safety protocols across all robot types
            - Supports conversational AI interaction with context-aware responses
            - Follows FastMCP 2.13+ best practices for feature-rich MCP servers

            SUPPORTED ROBOT TYPES:
            - Dreame D20 Pro: Primary platform - LIDAR vacuum with mapping, zone cleaning, mop, auto-empty
            - Yahboom Robots: ROSMASTER series with AI, navigation, and optional robotic arms
            - Moorebot Scout: ROS1 wheeled robot
            - Elegoo Robots: ROS-on-PC robots with serial communication
            - Unitree Go2/G1: Quadrupedal robots (when hardware available)
            - Virtual Robots: Unity3D/VRChat robots
            - Hue HomeAware: Philips Hue Bridge Pro with RF-based movement detection

            SUPPORTED OPERATIONS:
            - Universal: "get_status", "move", "stop"
            - Physical Robots: "return_to_dock", "stand", "sit", "walk", "sync_vbot"
            - Yahboom: "home_patrol", "camera_capture", "arm_move", "gripper_control", "navigate_to", "ai_query"
            - Elegoo: "emergency_stop" (ROS-on-PC serial communication)
            - Dreame: "start_auto_empty", "stop_auto_empty", "start_self_clean", "stop_self_clean",
                     "set_suction_level", "set_water_volume", "set_mop_humidity", "clean_zone",
                     "clean_spot", "start_mapping", "rename_room", "set_cleaning_sequence",
                     "set_restricted_zones", "get_cleaning_history", "clear_error"
            - Hue HomeAware: "hue_get_movement_events", "hue_get_sensor_status", "hue_get_movement_zones"

            Args:
                robot_id: Unique robot identifier. MUST follow naming convention:
                    - Physical: "scout_01", "go2_01", "g1_01", "yahboom_01", "elegoo_01", "dreame_01", "hue_01"
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
                        robot,
                        action,
                        linear,
                        angular,
                        duration,
                        x,
                        y,
                        theta,
                        joint_angles,
                        gripper_action,
                        patrol_route,
                        query,
                        query_type,
                    )
                elif robot.robot_type == "dreame":
                    return await self._handle_dreame_robot(
                        robot,
                        action,
                        linear,
                        angular,
                        duration,
                        x,
                        y,
                        theta,
                        suction_level,
                        water_volume,
                        mop_humidity,
                        zones,
                        spot_x,
                        spot_y,
                        room_id,
                        room_name,
                        cleaning_sequence,
                        restricted_zones,
                    )
                elif robot.robot_type == "hue":
                    return await self._handle_hue_robot(robot, action)
                elif robot.robot_type == "elegoo":
                    return await self._handle_elegoo_robot(robot, action, linear, angular)
                elif robot.robot_type == "gazebo":
                    return await self._handle_gazebo_robot(robot, action, linear, angular, duration)
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
        """Handle generic physical robot commands.

        For robot types without a dedicated client (Yahboom, Dreame, Elegoo, Gazebo),
        this handler returns an honest error indicating the robot type needs a specific
        client implementation.

        Args:
            robot: Robot state.
            action: Action to perform.
            linear: Linear velocity.
            angular: Angular velocity.
            duration: Movement duration.

        Returns:
            Operation result.
        """
        try:
            logger.info(
                ":ROBOT: Physical robot command received for unimplemented type",
                robot_id=robot.robot_id,
                robot_type=robot.robot_type,
                action=action,
            )
            return format_error_response(
                f"Robot type '{robot.robot_type}' does not have a dedicated client. "
                f"Supported types with real hardware clients: yahboom, dreame, elegoo, gazebo, hue. "
                f"Register this robot with a supported type or implement a client for '{robot.robot_type}'.",
                error_type="not_implemented",
                robot_id=robot.robot_id,
                action=action,
            )
        except Exception as e:
            return handle_tool_error("_handle_physical_robot", e, robot_id=robot.robot_id, action=action)

    def _yahboom_mcp_client(self, robot: Any) -> YahboomMcpClient:
        meta = robot.metadata or {}
        base_url = meta.get("yahboom_mcp_url") or yahboom_mcp_url()
        return YahboomMcpClient(base_url=base_url)

    async def _yahboom_mcp_preflight(
        self, client: YahboomMcpClient, robot_id: str, action: str
    ) -> dict[str, Any] | None:
        """Return an error response if yahboom-mcp is unreachable; else None."""
        health = await client.health()
        if not client.is_reachable(health):
            return format_error_response(
                f"yahboom-mcp not reachable at {client.base_url}",
                error_type="not_available",
                robot_id=robot_id,
                action=action,
                details={"health": health, "hint": "Start yahboom-mcp (default http://127.0.0.1:10892)"},
            )
        return None

    async def _yahboom_via_mounted_mcp(self, operation: str, params: dict[str, Any]) -> dict[str, Any] | None:
        if "yahboom" not in self.mounted_servers:
            return None
        raw = await call_mounted_server_tool(self.mounted_servers, "yahboom", "yahboom_tool", params)
        if isinstance(raw, dict):
            return raw
        return {
            "success": False,
            "error": "Unexpected response from mounted yahboom-mcp",
            "message": "Unexpected response from mounted yahboom-mcp",
        }

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
        """Handle Yahboom robot commands via yahboom-mcp (HTTP or mounted MCP)."""
        try:
            client = self._yahboom_mcp_client(robot)
            preflight = await self._yahboom_mcp_preflight(client, robot.robot_id, action)
            if preflight:
                return preflight

            if action == "get_status":
                telemetry = await client.telemetry()
                if not client.is_live_telemetry(telemetry):
                    return format_error_response(
                        telemetry.get("message") or "Yahboom robot bridge disconnected",
                        error_type="not_connected",
                        robot_id=robot.robot_id,
                        action=action,
                        details={"telemetry": telemetry},
                    )
                health = await client.health()
                status = client.build_status_payload(health, telemetry)
                return format_success_response(
                    f"Yahboom robot {robot.robot_id} status retrieved via yahboom-mcp",
                    robot_id=robot.robot_id,
                    action=action,
                    data=status,
                )

            if action == "move":
                logger.info(
                    ":MOVE: Yahboom via yahboom-mcp",
                    robot_id=robot.robot_id,
                    linear=linear,
                    angular=angular,
                    duration=duration,
                )
                result = await client.move(linear or 0.0, angular or 0.0)
                if not mcp_call_succeeded(result):
                    return format_error_response(
                        result.get("error", "Yahboom move failed"),
                        error_type=result.get("error_type", "motion_error"),
                        robot_id=robot.robot_id,
                        action=action,
                        details=result,
                    )
                return format_success_response(
                    f"Yahboom robot {robot.robot_id} movement command sent via yahboom-mcp",
                    robot_id=robot.robot_id,
                    action=action,
                    data=result,
                )

            if action == "stop":
                logger.warning(":SECURITY: EMERGENCY STOP via yahboom-mcp", robot_id=robot.robot_id)
                result = await client.stop_all()
                if not mcp_call_succeeded(result):
                    return format_error_response(
                        result.get("error", "Yahboom stop failed"),
                        error_type="motion_error",
                        robot_id=robot.robot_id,
                        action=action,
                        details=result,
                    )
                return format_success_response(
                    f"Yahboom robot {robot.robot_id} stopped via yahboom-mcp",
                    robot_id=robot.robot_id,
                    action=action,
                    data=result,
                )

            if action == "camera_capture":
                result = await client.snapshot()
                frame = result.get("frame")
                if frame:
                    return format_success_response(
                        f"Yahboom robot {robot.robot_id} camera capture via yahboom-mcp",
                        robot_id=robot.robot_id,
                        action=action,
                        data={"frame_size": len(frame), "source": "yahboom-mcp"},
                    )
                return format_error_response(
                    result.get("error", "Camera not available or failed to capture"),
                    error_type="camera_error",
                    robot_id=robot.robot_id,
                    action=action,
                    details=result,
                )

            if action in ("arm_move", "gripper_control"):
                return format_error_response(
                    f"Action '{action}' is not supported on ROSMaster-M1 via robotics-mcp — use yahboom-mcp tools directly",
                    error_type="unsupported_action",
                    robot_id=robot.robot_id,
                    action=action,
                )

            if action == "navigate_to":
                if x is None or y is None:
                    return format_error_response(
                        "x and y coordinates required for navigate_to action",
                        error_type="missing_parameter",
                        robot_id=robot.robot_id,
                        action=action,
                    )
                goal = f"navigate to map position x={x} y={y} theta={theta or 0.0}"
                result = await client.agent_mission(goal)
                if not mcp_call_succeeded(result):
                    return format_error_response(
                        result.get("error", "Navigation mission planning failed"),
                        error_type="navigation_error",
                        robot_id=robot.robot_id,
                        action=action,
                        details=result,
                    )
                return format_success_response(
                    f"Yahboom robot {robot.robot_id} navigation mission planned via yahboom-mcp",
                    robot_id=robot.robot_id,
                    action=action,
                    data={"target_pose": {"x": x, "y": y, "theta": theta or 0.0}, "mission": result},
                )

            if action == "home_patrol":
                route = patrol_route or "home_security"
                goal = (
                    f"patrol route '{route}': living room, kitchen, bedroom, return to base; "
                    "report obstacles and battery"
                )
                result = await client.agent_mission(goal, speak=False)
                if not mcp_call_succeeded(result):
                    return format_error_response(
                        result.get("error", "Patrol mission planning failed"),
                        error_type="navigation_error",
                        robot_id=robot.robot_id,
                        action=action,
                        details=result,
                    )
                plan = result.get("plan") or {}
                waypoints = plan.get("waypoints") or [
                    {"x": 0.0, "y": 0.0, "description": "living room"},
                    {"x": 3.0, "y": 0.0, "description": "kitchen"},
                    {"x": 3.0, "y": 2.0, "description": "bedroom"},
                    {"x": 0.0, "y": 2.0, "description": "return to base"},
                ]
                return format_success_response(
                    f"Yahboom robot {robot.robot_id} home patrol mission via yahboom-mcp",
                    robot_id=robot.robot_id,
                    action=action,
                    data={"patrol_route": route, "waypoints": waypoints, "mission": result},
                )

            if action == "ai_query":
                if not query:
                    return format_error_response(
                        "Query parameter required for ai_query action",
                        error_type="missing_parameter",
                        robot_id=robot.robot_id,
                        action=action,
                    )
                goal = query if query_type in (None, "text") else f"[{query_type}] {query}"
                result = await client.agent_mission(goal, speak=False)
                if not mcp_call_succeeded(result):
                    return format_error_response(
                        result.get("error", "AI mission/query failed"),
                        error_type="agent_error",
                        robot_id=robot.robot_id,
                        action=action,
                        details={"query": query, "query_type": query_type, "mission": result},
                    )
                return format_success_response(
                    f"Yahboom robot {robot.robot_id} AI query handled via yahboom-mcp",
                    robot_id=robot.robot_id,
                    action=action,
                    query=query,
                    data={"query": query, "query_type": query_type or "text", "mission": result},
                )

            return format_error_response(
                f"Unsupported action '{action}' for Yahboom robot",
                error_type="unsupported_action",
                robot_id=robot.robot_id,
                action=action,
            )

        except Exception as e:
            return handle_tool_error("_handle_yahboom_robot", e, robot_id=robot.robot_id, action=action)

    async def _handle_gazebo_robot(
        self,
        robot: Any,
        action: str,
        linear: float | None,
        angular: float | None,
        duration: float | None,
    ) -> dict[str, Any]:
        """Handle Gazebo simulated robot commands via rosbridge.

        Args:
            robot: Robot state object.
            action: Action to perform (get_status, move, stop).
            linear: Linear velocity for movement.
            angular: Angular velocity for movement.
            duration: Movement duration.

        Returns:
            Operation result.
        """
        meta = robot.metadata or {}
        config = GazeboRobotConfig(
            robot_id=robot.robot_id,
            host=meta.get("host", "localhost"),
            port=meta.get("port", 9090),
            model=meta.get("model", "turtlebot3"),
            cmd_vel_topic=meta.get("cmd_vel_topic", "/cmd_vel"),
            odom_topic=meta.get("odom_topic", "/odom"),
            scan_topic=meta.get("scan_topic", "/scan"),
            camera_enabled=meta.get("camera_enabled", True),
            mock_mode=meta.get("mock_mode", True),
        )
        client = GazeboClient(config)
        try:
            await client.connect()
            if action == "get_status":
                status = await client.get_status()
                return format_success_response(
                    f"Gazebo robot {robot.robot_id} status retrieved",
                    robot_id=robot.robot_id,
                    action=action,
                    data=status,
                )
            elif action == "move":
                result = await client.move(
                    linear or 0.0,
                    angular or 0.0,
                    duration,
                )
                return format_success_response(
                    f"Gazebo robot {robot.robot_id} movement command executed",
                    robot_id=robot.robot_id,
                    action=action,
                    data=result,
                )
            elif action == "stop":
                result = await client.stop()
                return format_success_response(
                    f"Gazebo robot {robot.robot_id} stopped",
                    robot_id=robot.robot_id,
                    action=action,
                    data=result,
                )
            else:
                return format_error_response(
                    f"Unsupported action '{action}' for Gazebo robot",
                    error_type="unsupported_action",
                    robot_id=robot.robot_id,
                    action=action,
                )
        except Exception as e:
            return handle_tool_error("_handle_gazebo_robot", e, robot_id=robot.robot_id, action=action)
        finally:
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

            config = getattr(robot, "metadata", {}) or {}
            get_dreame_client(robot.robot_id, config)

            # Handle different Dreame actions
            if action == "get_status":
                return await dreame_get_status(robot.robot_id, config=config)

            elif action == "start_cleaning":
                return await dreame_start_cleaning(robot.robot_id, config=config)

            elif action == "stop_cleaning":
                return await dreame_stop_cleaning(robot.robot_id, config=config)

            elif action == "move":
                rotation = int(angular * 100) if angular else 0
                velocity = int(linear * 100) if linear else 50
                return await dreame_move(robot.robot_id, rotation=rotation, velocity=velocity, config=config)

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
                return await dreame_get_map(robot.robot_id, config=config)

            elif action == "clean_room":
                if room_id is not None:
                    return await dreame_clean_room(robot.robot_id, room_id, config=config)
                else:
                    return format_error_response(
                        "clean_room action requires room_id",
                        error_type="missing_room_id",
                        robot_id=robot.robot_id,
                    )

            elif action == "clean_zone":
                if zones:
                    return await dreame_clean_zone(robot.robot_id, zones, config=config)
                else:
                    return format_error_response(
                        "clean_zone action requires zones parameter",
                        error_type="missing_zones",
                        robot_id=robot.robot_id,
                    )

            elif action == "clean_spot":
                if spot_x is not None and spot_y is not None:
                    return await dreame_clean_spot(robot.robot_id, spot_x, spot_y, config=config)
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
                        "get_status",
                        "start_cleaning",
                        "stop_cleaning",
                        "move",
                        "go_to",
                        "get_map",
                        "clean_room",
                        "clean_zone",
                        "clean_spot",
                        "return_to_dock",
                        "set_suction_level",
                        "set_water_volume",
                        "set_mop_humidity",
                        "start_fast_mapping",
                        "start_mapping",
                        "set_cleaning_sequence",
                        "set_restricted_zones",
                        "get_cleaning_history",
                        "clear_error",
                    ],
                )

        except Exception as e:
            return handle_tool_error("_handle_dreame_robot", e, robot_id=robot.robot_id, action=action)

    async def _handle_hue_robot(
        self,
        robot: Any,
        action: str,
    ) -> dict[str, Any]:
        """Handle Philips Hue Bridge Pro HomeAware commands.

        Args:
            robot: Robot state object.
            action: Action to perform.

        Returns:
            Operation result.
        """
        try:
            # Route to appropriate Hue HomeAware handler
            if action == "hue_get_movement_events":
                return await hue_get_movement_events(robot.robot_id)

            elif action == "hue_get_sensor_status":
                return await hue_get_sensor_status(robot.robot_id)

            elif action == "hue_get_movement_zones":
                return await hue_get_movement_zones(robot.robot_id)

            else:
                return format_error_response(
                    f"Unsupported Hue action: {action}",
                    error_type="unsupported_action",
                    robot_id=robot.robot_id,
                    action=action,
                )

        except Exception as e:
            return handle_tool_error("_handle_hue_robot", e, robot_id=robot.robot_id, action=action)

    async def _handle_elegoo_robot(
        self,
        robot: Any,
        action: str,
        linear: float | None,
        angular: float | None,
    ) -> dict[str, Any]:
        """Handle Elegoo robot commands.

        Args:
            robot: Robot state.
            action: Action to perform.
            linear: Linear velocity (-1.0 to 1.0).
            angular: Angular velocity (-1.0 to 1.0).

        Returns:
            Operation result.
        """
        try:
            # Get or create Elegoo client
            if robot.robot_id not in self.elegoo_clients:
                # Use default serial port, can be configured later
                config = ElegooRobotConfig(robot_id=robot.robot_id)
                client = ElegooClient(config)
                connected = await client.connect()
                if not connected:
                    return format_error_response(
                        f"Failed to connect to Elegoo robot {robot.robot_id}",
                        error_type="connection_failed",
                        robot_id=robot.robot_id,
                        action=action,
                    )
                self.elegoo_clients[robot.robot_id] = client

            client = self.elegoo_clients[robot.robot_id]

            if action == "get_status":
                status = client.get_status()
                sensor_data = await client.get_sensor_data()
                return format_success_response(
                    f"Elegoo robot {robot.robot_id} status retrieved",
                    robot_id=robot.robot_id,
                    connected=client.connected,
                    sensor_data=sensor_data,
                    status=status,
                )

            elif action == "move":
                if linear is None:
                    linear = 0.0
                if angular is None:
                    angular = 0.0

                # Convert to differential drive
                left_speed = linear - angular * 0.5  # Adjust multiplier as needed
                right_speed = linear + angular * 0.5

                success = await client.set_motors(left_speed, right_speed)
                if success:
                    return format_success_response(
                        f"Elegoo robot moving: linear={linear:.2f}, angular={angular:.2f}",
                        robot_id=robot.robot_id,
                        left_speed=left_speed,
                        right_speed=right_speed,
                    )
                else:
                    return format_error_response(
                        "Failed to set Elegoo robot motors",
                        error_type="motor_control_failed",
                        robot_id=robot.robot_id,
                        action=action,
                    )

            elif action == "stop" or action == "emergency_stop":
                success = await client.emergency_stop()
                if success:
                    return format_success_response(
                        "Elegoo robot emergency stop activated",
                        robot_id=robot.robot_id,
                        safety_warnings=["Verify robot has stopped completely"],
                    )
                else:
                    return format_error_response(
                        "Failed to stop Elegoo robot",
                        error_type="stop_failed",
                        robot_id=robot.robot_id,
                        action=action,
                    )

            else:
                return format_error_response(
                    f"Unsupported action '{action}' for Elegoo robot",
                    error_type="unsupported_action",
                    robot_id=robot.robot_id,
                    action=action,
                    supported_actions=["get_status", "move", "stop", "emergency_stop"],
                )

        except Exception as e:
            return handle_tool_error("_handle_elegoo_robot", e, robot_id=robot.robot_id, action=action)

    async def _handle_virtual_robot(
        self,
        robot: Any,
        action: str,
        linear: float | None,
        angular: float | None,
        duration: float | None,
        *,
        head_yaw: float | None = None,
        head_pitch: float | None = None,
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
                                {
                                    "avatar_id": robot.robot_id,
                                    "direction": "forward",
                                    "speed": linear or 0.0,
                                },
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
                    return format_unavailable_error(
                        "avatar-mcp or unity3d-mcp",
                        "move virtual robot",
                        robot_id=robot.robot_id,
                        action=action,
                        platform=robot.platform,
                    )
                elif robot.platform == "vrchat" and "vrchat" in self.mounted_servers:
                    # Use VRChat OSC for movement
                    await call_mounted_server_tool(
                        self.mounted_servers,
                        "vrchat",
                        "vrchat_send_osc_message",
                        {
                            "address": f"/robot/{robot.robot_id}/move",
                            "args": [linear or 0.0, angular or 0.0],
                        },
                    )
                    return {
                        "status": "success",
                        "message": "Virtual robot moved via VRChat OSC",
                        "robot_id": robot.robot_id,
                        "action": action,
                    }
                elif robot.platform == "resonite":
                    # Use built-in OSC bridge for Resonite
                    from ..osc_bridge import osc_bridge

                    await osc_bridge.send_message(f"/robot/{robot.robot_id}/move", [linear or 0.0, angular or 0.0])
                    return {
                        "status": "success",
                        "message": "Virtual robot moved via Resonite OSC",
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
                elif robot.platform == "resonite":
                    from ..osc_bridge import osc_bridge

                    await osc_bridge.send_message(f"/robot/{robot.robot_id}/stop", [1])
                    return {
                        "status": "success",
                        "message": "Virtual robot stopped via Resonite OSC",
                        "robot_id": robot.robot_id,
                        "action": action,
                    }
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

            elif action == "head":
                if robot.platform == "resonite":
                    from ..osc_bridge import osc_bridge

                    await osc_bridge.send_message(
                        f"/robot/{robot.robot_id}/head",
                        [head_yaw or 0.0, head_pitch or 0.0],
                    )
                    return {
                        "status": "success",
                        "message": "Virtual robot head via Resonite OSC",
                        "robot_id": robot.robot_id,
                        "action": action,
                        "yaw": head_yaw,
                        "pitch": head_pitch,
                    }

            elif action == "spawn":
                if robot.platform == "resonite":
                    from ..osc_bridge import osc_bridge

                    meta = robot.metadata or {}
                    pos = meta.get("position") or {"x": 0.0, "y": 0.0, "z": 0.0}
                    scale = float(meta.get("scale", 1.0))
                    await osc_bridge.spawn_vbot(
                        robot.robot_id,
                        robot.robot_type,
                        x=float(pos.get("x", 0.0)),
                        y=float(pos.get("y", 0.0)),
                        z=float(pos.get("z", 0.0)),
                        scale=scale,
                    )
                    return {
                        "status": "success",
                        "message": "Virtual robot spawned via Resonite OSC",
                        "robot_id": robot.robot_id,
                        "action": action,
                    }

            elif action == "emergency_stop":
                from ..osc_bridge import VBOOMY_ESTOP_ADDRESS, osc_bridge

                await osc_bridge.send_message(VBOOMY_ESTOP_ADDRESS, [1.0])
                await osc_bridge.send_message(f"/robot/{robot.robot_id}/stop", [1])
                return {
                    "status": "success",
                    "message": "Fleet e-stop sent via OSC",
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
            return format_error_response(
                f"Robot {robot_id} not found",
                error_type="not_found",
                robot_id=robot_id,
                action=action,
            )

        linear = params.get("linear") or params.get("linear_x")
        angular = params.get("angular") or params.get("angular_z")
        duration = params.get("duration")
        x = params.get("x")
        y = params.get("y")
        theta = params.get("theta")
        joint_angles = params.get("joint_angles")
        gripper_action = params.get("gripper_action")
        patrol_route = params.get("patrol_route")
        query = params.get("query")
        query_type = params.get("query_type")
        head_yaw = params.get("yaw") if params.get("yaw") is not None else params.get("head_yaw")
        head_pitch = params.get("pitch") if params.get("pitch") is not None else params.get("head_pitch")

        if robot.is_virtual:
            return await self._handle_virtual_robot(
                robot,
                action,
                linear,
                angular,
                duration,
                head_yaw=head_yaw,
                head_pitch=head_pitch,
            )
        elif robot.robot_type == "yahboom":
            return await self._handle_yahboom_robot(
                robot,
                action,
                linear,
                angular,
                duration,
                x,
                y,
                theta,
                joint_angles,
                gripper_action,
                patrol_route,
                query,
                query_type,
            )
        elif robot.robot_type == "dreame":
            return await self._handle_dreame_robot(
                robot,
                action,
                linear,
                angular,
                duration,
                x,
                y,
                theta,
                params.get("suction_level"),
                params.get("water_volume"),
                params.get("mop_humidity"),
                params.get("zones"),
                params.get("spot_x"),
                params.get("spot_y"),
                params.get("room_id"),
                params.get("room_name"),
                params.get("cleaning_sequence"),
                params.get("restricted_zones"),
            )
        elif robot.robot_type == "hue":
            return await self._handle_hue_robot(robot, action)
        elif robot.robot_type == "elegoo":
            return await self._handle_elegoo_robot(robot, action, linear, angular)
        elif robot.robot_type == "gazebo":
            return await self._handle_gazebo_robot(robot, action, linear, angular, duration)
        else:
            return await self._handle_physical_robot(robot, action, linear, angular, duration)
