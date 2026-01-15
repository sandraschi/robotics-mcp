"""Robot control portmanteau tool - Unified bot + vbot control."""

from typing import Any, Dict, Literal, Optional

import structlog

from ..clients.yahboom_client import YahboomClient, YahboomRobotConfig
from ..utils.error_handler import format_error_response, format_success_response, handle_tool_error
from ..utils.mcp_client_helper import call_mounted_server_tool

logger = structlog.get_logger(__name__)


class RobotControlTool:
    """Portmanteau tool for unified robot control (bot + vbot)."""

    def __init__(self, mcp: Any, state_manager: Any, mounted_servers: Optional[Dict[str, Any]] = None):
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
            ],
            linear: Optional[float] = None,
            angular: Optional[float] = None,
            duration: Optional[float] = None,
            # Yahboom-specific parameters
            x: Optional[float] = None,
            y: Optional[float] = None,
            theta: Optional[float] = None,
            joint_angles: Optional[Dict[str, float]] = None,
            gripper_action: Optional[Literal["open", "close", "stop"]] = None,
            patrol_route: Optional[str] = None,
        ) -> Dict[str, Any]:
            """Unified robot control (works for both physical bot and virtual bot).

            This portmanteau tool provides a unified interface for controlling both
            physical robots (via ROS) and virtual robots (via Unity/VRChat). The tool
            automatically routes commands to the appropriate handler based on robot type.

            Args:
                robot_id: Robot identifier (e.g., "scout_01", "vbot_scout_01", "yahboom_01").
                action: Operation to perform:
                    - "get_status": Get robot status (battery, position, state)
                    - "move": Control movement (linear/angular velocity)
                    - "stop": Emergency stop
                    - "return_to_dock": Return to charging dock (physical bot only)
                    - "stand": Stand up (Unitree G1, physical bot only)
                    - "sit": Sit down (Unitree G1, physical bot only)
                    - "walk": Walking gait (Unitree, physical bot only)
                    - "sync_vbot": Sync virtual bot with physical bot state
                    - "home_patrol": Start autonomous home patrol (Yahboom)
                    - "camera_capture": Capture camera image (Yahboom)
                    - "arm_move": Move robotic arm joints (Yahboom with arm)
                    - "gripper_control": Control gripper open/close (Yahboom with arm)
                    - "navigate_to": Navigate to specific coordinates (Yahboom)
                linear: Linear velocity (m/s) for move action.
                angular: Angular velocity (rad/s) for move action.
                duration: Movement duration (seconds).
                x: Target X coordinate for navigation.
                y: Target Y coordinate for navigation.
                theta: Target orientation for navigation.
                joint_angles: Dictionary of joint angles for arm control.
                gripper_action: Gripper action ("open", "close", "stop").
                patrol_route: Name of patrol route to execute.
                **kwargs: Additional action-specific parameters.

            Returns:
                Dictionary containing operation result.

            Examples:
                Get robot status (any robot type):
                    result = await robot_control(robot_id="scout_01", action="get_status")
                    result = await robot_control(robot_id="yahboom_01", action="get_status")

                Move robot forward:
                    result = await robot_control(
                        robot_id="scout_01",
                        action="move",
                        linear=0.2,
                        angular=0.0
                    )

                Stop robot:
                    result = await robot_control(robot_id="scout_01", action="stop")

                Yahboom-specific actions:
                    # Start home patrol
                    result = await robot_control(robot_id="yahboom_01", action="home_patrol")

                    # Navigate to coordinates
                    result = await robot_control(
                        robot_id="yahboom_01",
                        action="navigate_to",
                        x=2.0, y=1.5, theta=0.0
                    )

                    # Control arm (when equipped)
                    result = await robot_control(
                        robot_id="yahboom_01",
                        action="arm_move",
                        joint_angles={"joint1": 0.5, "joint2": 0.3}
                    )

                    # Control gripper
                    result = await robot_control(
                        robot_id="yahboom_01",
                        action="gripper_control",
                        gripper_action="open"
                    )
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
                        x, y, theta, joint_angles, gripper_action, patrol_route
                    )
                else:
                    return await self._handle_physical_robot(robot, action, linear, angular, duration)
            except Exception as e:
                return handle_tool_error("robot_control", e, robot_id=robot_id, action=action)

    async def _handle_physical_robot(
        self,
        robot: Any,
        action: str,
        linear: Optional[float],
        angular: Optional[float],
        duration: Optional[float],
    ) -> Dict[str, Any]:
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
        linear: Optional[float],
        angular: Optional[float],
        duration: Optional[float],
        x: Optional[float],
        y: Optional[float],
        theta: Optional[float],
        joint_angles: Optional[Dict[str, float]],
        gripper_action: Optional[str],
        patrol_route: Optional[str],
    ) -> Dict[str, Any]:
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

    async def _handle_virtual_robot(
        self,
        robot: Any,
        action: str,
        linear: Optional[float],
        angular: Optional[float],
        duration: Optional[float],
    ) -> Dict[str, Any]:
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
        from fastmcp import Client

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
                                "message": f"Virtual robot moved via avatar-mcp",
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
                                "message": f"Virtual robot moved via Unity",
                                "robot_id": robot.robot_id,
                                "action": action,
                            }
                elif robot.platform == "vrchat":
                    # Use VRChat OSC for movement
                    if "vrchat" in self.mounted_servers:
                        await call_mounted_server_tool(
                            self.mounted_servers,
                            "vrchat",
                            "vrchat_send_osc_message",
                            {"address": f"/robot/{robot.robot_id}/move", "args": [linear or 0.0, angular or 0.0]},
                        )
                        return {
                            "status": "success",
                            "message": f"Virtual robot moved via VRChat OSC",
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
                        "message": f"Virtual robot stopped",
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

    async def handle_action(self, robot_id: str, action: str, params: Dict[str, Any]) -> Dict[str, Any]:
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

