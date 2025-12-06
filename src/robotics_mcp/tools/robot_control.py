"""Robot control portmanteau tool - Unified bot + vbot control."""

from typing import Any, Dict, Literal, Optional

import structlog

logger = structlog.get_logger(__name__)


class RobotControlTool:
    """Portmanteau tool for unified robot control (bot + vbot)."""

    def __init__(self, mcp: Any, state_manager: Any):
        """Initialize robot control tool.

        Args:
            mcp: FastMCP server instance.
            state_manager: Robot state manager instance.
        """
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
            ],
            linear: Optional[float] = None,
            angular: Optional[float] = None,
            duration: Optional[float] = None,
            **kwargs: Any,
        ) -> Dict[str, Any]:
            """Unified robot control (works for both physical bot and virtual bot).

            This portmanteau tool provides a unified interface for controlling both
            physical robots (via ROS) and virtual robots (via Unity/VRChat). The tool
            automatically routes commands to the appropriate handler based on robot type.

            Args:
                robot_id: Robot identifier (e.g., "scout_01", "vbot_scout_01").
                action: Operation to perform:
                    - "get_status": Get robot status (battery, position, state)
                    - "move": Control movement (linear/angular velocity)
                    - "stop": Emergency stop
                    - "return_to_dock": Return to charging dock (physical bot only)
                    - "stand": Stand up (Unitree G1, physical bot only)
                    - "sit": Sit down (Unitree G1, physical bot only)
                    - "walk": Walking gait (Unitree, physical bot only)
                    - "sync_vbot": Sync virtual bot with physical bot state
                linear: Linear velocity (m/s) for move action.
                angular: Angular velocity (rad/s) for move action.
                duration: Movement duration (seconds).
                **kwargs: Additional action-specific parameters.

            Returns:
                Dictionary containing operation result.

            Examples:
                Get robot status:
                    result = await robot_control(robot_id="scout_01", action="get_status")

                Move robot forward:
                    result = await robot_control(
                        robot_id="scout_01",
                        action="move",
                        linear=0.2,
                        angular=0.0
                    )

                Stop robot:
                    result = await robot_control(robot_id="scout_01", action="stop")
            """
            robot = self.state_manager.get_robot(robot_id)
            if not robot:
                return {"status": "error", "message": f"Robot {robot_id} not found"}

            # Route to appropriate handler
            if robot.is_virtual:
                return await self._handle_virtual_robot(robot, action, linear, angular, duration, **kwargs)
            else:
                return await self._handle_physical_robot(robot, action, linear, angular, duration, **kwargs)

    async def _handle_physical_robot(
        self,
        robot: Any,
        action: str,
        linear: Optional[float],
        angular: Optional[float],
        duration: Optional[float],
        **kwargs: Any,
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
        # TODO: Implement ROS bridge integration
        logger.info("Physical robot command", robot_id=robot.robot_id, action=action)
        return {
            "status": "success",
            "message": f"Physical robot {action} command sent (mock)",
            "robot_id": robot.robot_id,
            "action": action,
        }

    async def _handle_virtual_robot(
        self,
        robot: Any,
        action: str,
        linear: Optional[float],
        angular: Optional[float],
        duration: Optional[float],
        **kwargs: Any,
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
                    async with Client(self.mcp) as client:
                        # Try avatar-mcp first for smooth locomotion
                        try:
                            await client.call_tool(
                                "avatar_movement_walk",
                                avatar_id=robot.robot_id,
                                direction="forward",
                                speed=linear or 0.0,
                            )
                            if angular:
                                await client.call_tool(
                                    "avatar_movement_turn",
                                    avatar_id=robot.robot_id,
                                    angle=angular,
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
                            await client.call_tool(
                                "unity_execute_method",
                                class_name="RobotController",
                                method_name="Move",
                                parameters={
                                    "robotId": robot.robot_id,
                                    "linear": linear or 0.0,
                                    "angular": angular or 0.0,
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
                    async with Client(self.mcp) as client:
                        await client.call_tool(
                            "vrchat_send_osc_message",
                            address=f"/robot/{robot.robot_id}/move",
                            args=[linear or 0.0, angular or 0.0],
                        )
                        return {
                            "status": "success",
                            "message": f"Virtual robot moved via VRChat OSC",
                            "robot_id": robot.robot_id,
                            "action": action,
                        }

            elif action == "stop":
                async with Client(self.mcp) as client:
                    if robot.platform == "vrchat":
                        await client.call_tool(
                            "vrchat_send_osc_message",
                            address=f"/robot/{robot.robot_id}/stop",
                            args=[1],
                        )
                    else:
                        await client.call_tool(
                            "avatar_movement_walk",
                            avatar_id=robot.robot_id,
                            direction="forward",
                            speed=0.0,
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
            logger.error("Virtual robot command failed", error=str(e), robot_id=robot.robot_id, action=action)
            return {"status": "error", "message": str(e)}

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

