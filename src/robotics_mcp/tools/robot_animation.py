"""Robot animation portmanteau tool - Animation and behavior control.

Provides unified animation control for both physical and virtual robots.
"""

from typing import Any, Literal

import structlog

from ..utils.error_handler import (
    format_error_response,
    format_not_implemented_error,
    format_success_response,
    format_unavailable_error,
    handle_tool_error,
)
from ..utils.mcp_client_helper import call_mounted_server_tool

logger = structlog.get_logger(__name__)


class RobotAnimationTool:
    """Portmanteau tool for robot animation and behavior control."""

    def __init__(self, mcp: Any, state_manager: Any, mounted_servers: dict[str, Any] | None = None):
        """Initialize robot animation tool.

        Args:
            mcp: FastMCP server instance.
            state_manager: Robot state manager instance.
            mounted_servers: Dictionary of mounted MCP servers.
        """
        self.mcp = mcp
        self.state_manager = state_manager
        self.mounted_servers = mounted_servers or {}

    def register(self):
        """Register robot animation tool with MCP server."""

        @self.mcp.tool()
        async def robot_animation(
            robot_id: str,
            action: Literal[
                "animate_wheels",
                "animate_movement",
                "set_pose",
                "play_animation",
                "stop_animation",
                "get_animation_state",
            ],
            wheel_speeds: dict[str, float] | None = None,
            animation_name: str | None = None,
            pose: str | None = None,
            speed: float | None = None,
            loop: bool = False,
        ) -> dict[str, Any]:
            """Robot animation and behavior control portmanteau.

            PORTMANTEAU PATTERN: Consolidates animation operations into a single tool.

            SUPPORTED OPERATIONS:
            - animate_wheels: Rotate wheels during movement (Scout mecanum wheels)
            - animate_movement: Play movement animations (walk, turn, etc.)
            - set_pose: Set robot pose (sitting, standing, etc. for Unitree)
            - play_animation: Play custom animations
            - stop_animation: Stop current animation
            - get_animation_state: Get current animation state

            Args:
                robot_id: Robot identifier (e.g., "scout_01", "vbot_scout_01").
                action: Animation operation to perform.
                wheel_speeds: Wheel speeds for animate_wheels (front_left, front_right, back_left, back_right).
                animation_name: Animation name for play_animation (e.g., "walk", "turn", "idle").
                pose: Pose name for set_pose (e.g., "sit", "stand", "crouch").
                speed: Animation speed multiplier (1.0 = normal speed).
                loop: Whether to loop animation (for play_animation).

            Returns:
                Dictionary containing operation result.

            Examples:
                Animate Scout wheels:
                    result = await robot_animation(
                        robot_id="scout_01",
                        action="animate_wheels",
                        wheel_speeds={"front_left": 1.0, "front_right": 1.0, "back_left": 1.0, "back_right": 1.0}
                    )

                Play walk animation:
                    result = await robot_animation(
                        robot_id="scout_01",
                        action="play_animation",
                        animation_name="walk",
                        speed=1.0,
                        loop=True
                    )

                Set Unitree pose:
                    result = await robot_animation(
                        robot_id="g1_01",
                        action="set_pose",
                        pose="sit"
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
                    return await self._handle_virtual_animation(
                        robot, action, wheel_speeds, animation_name, pose, speed, loop
                    )
                else:
                    return await self._handle_physical_animation(
                        robot, action, wheel_speeds, animation_name, pose, speed, loop
                    )

            except Exception as e:
                return handle_tool_error("robot_animation", e, robot_id=robot_id, action=action)

    async def _handle_virtual_animation(
        self,
        robot: Any,
        action: str,
        wheel_speeds: dict[str, float] | None,
        animation_name: str | None,
        pose: str | None,
        speed: float | None,
        loop: bool,
    ) -> dict[str, Any]:
        """Handle virtual robot animation."""
        try:
            if robot.platform == "unity" and "unity" in self.mounted_servers:
                if action == "animate_wheels":
                    result = await call_mounted_server_tool(
                        self.mounted_servers,
                        "unity",
                        "execute_unity_method",
                        {
                            "class_name": "RobotAnimator",
                            "method_name": "AnimateWheels",
                            "parameters": {
                                "robotId": robot.robot_id,
                                "wheelSpeeds": wheel_speeds or {},
                            },
                        },
                    )
                    return format_success_response(
                        f"Wheel animation started for {robot.robot_id}",
                        robot_id=robot.robot_id,
                        action=action,
                        data=result,
                    )

                elif action == "animate_movement":
                    result = await call_mounted_server_tool(
                        self.mounted_servers,
                        "unity",
                        "execute_unity_method",
                        {
                            "class_name": "RobotAnimator",
                            "method_name": "AnimateMovement",
                            "parameters": {
                                "robotId": robot.robot_id,
                                "animationName": animation_name or "walk",
                                "speed": speed or 1.0,
                                "loop": loop,
                            },
                        },
                    )
                    return format_success_response(
                        f"Movement animation started for {robot.robot_id}",
                        robot_id=robot.robot_id,
                        action=action,
                        data=result,
                    )

                elif action == "set_pose":
                    result = await call_mounted_server_tool(
                        self.mounted_servers,
                        "unity",
                        "execute_unity_method",
                        {
                            "class_name": "RobotAnimator",
                            "method_name": "SetPose",
                            "parameters": {
                                "robotId": robot.robot_id,
                                "pose": pose or "idle",
                            },
                        },
                    )
                    return format_success_response(
                        f"Pose set for {robot.robot_id}",
                        robot_id=robot.robot_id,
                        action=action,
                        data=result,
                    )

                elif action == "play_animation":
                    result = await call_mounted_server_tool(
                        self.mounted_servers,
                        "unity",
                        "execute_unity_method",
                        {
                            "class_name": "RobotAnimator",
                            "method_name": "PlayAnimation",
                            "parameters": {
                                "robotId": robot.robot_id,
                                "animationName": animation_name or "idle",
                                "speed": speed or 1.0,
                                "loop": loop,
                            },
                        },
                    )
                    return format_success_response(
                        f"Animation playing for {robot.robot_id}",
                        robot_id=robot.robot_id,
                        action=action,
                        data=result,
                    )

                elif action == "stop_animation":
                    result = await call_mounted_server_tool(
                        self.mounted_servers,
                        "unity",
                        "execute_unity_method",
                        {
                            "class_name": "RobotAnimator",
                            "method_name": "StopAnimation",
                            "parameters": {
                                "robotId": robot.robot_id,
                            },
                        },
                    )
                    return format_success_response(
                        f"Animation stopped for {robot.robot_id}",
                        robot_id=robot.robot_id,
                        action=action,
                        data=result,
                    )

                elif action == "get_animation_state":
                    result = await call_mounted_server_tool(
                        self.mounted_servers,
                        "unity",
                        "execute_unity_method",
                        {
                            "class_name": "RobotAnimator",
                            "method_name": "GetAnimationState",
                            "parameters": {
                                "robotId": robot.robot_id,
                            },
                        },
                    )
                    return format_success_response(
                        f"Animation state retrieved for {robot.robot_id}",
                        robot_id=robot.robot_id,
                        action=action,
                        data=result,
                    )

            elif robot.platform == "vrchat" and "avatar" in self.mounted_servers:
                # Use avatar-mcp for VRChat animations (mounted server pattern)
                if action == "set_pose":
                    result = await call_mounted_server_tool(
                        self.mounted_servers,
                        "avatar",
                        "avatar_set_pose",
                        {
                            "avatar_id": robot.robot_id,
                            "pose": pose or "idle",
                        },
                    )
                    return format_success_response(
                        f"Pose set for {robot.robot_id} (VRChat)",
                        robot_id=robot.robot_id,
                        action=action,
                        data=result,
                    )
                else:
                    # Fallback to OSC for other animations (mounted server pattern)
                    result = await call_mounted_server_tool(
                        self.mounted_servers,
                        "osc",
                        "osc_send_osc",
                        {
                            "host": "127.0.0.1",
                            "port": 9000,
                            "address": f"/robot/{robot.robot_id}/animation",
                            "values": [action, animation_name or "", speed or 1.0],
                        },
                    )
                    return format_success_response(
                        f"Animation command sent for {robot.robot_id} (VRChat OSC)",
                        robot_id=robot.robot_id,
                        action=action,
                        data=result,
                    )

            else:
                return format_unavailable_error(
                    "Unity MCP or VRChat OSC",
                    f"run animation action '{action}'",
                    robot_id=robot.robot_id,
                    action=action,
                    platform=robot.platform,
                )

        except Exception as e:
            return handle_tool_error("_handle_virtual_animation", e, robot_id=robot.robot_id, action=action)

    async def _handle_physical_animation(
        self,
        robot: Any,
        action: str,
        wheel_speeds: dict[str, float] | None,
        animation_name: str | None,
        pose: str | None,
        speed: float | None,
        loop: bool,
    ) -> dict[str, Any]:
        """Handle physical robot animation."""
        return format_not_implemented_error(
            f"Physical robot animation action '{action}'",
            robot_id=robot.robot_id,
            action=action,
            details={"requires": "ROS animation integration"},
        )
