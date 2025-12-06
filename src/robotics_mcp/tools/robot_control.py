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
        # Use mounted MCP servers for virtual robot control
        logger.info("Virtual robot command", robot_id=robot.robot_id, action=action, platform=robot.platform)

        if action == "move" and robot.platform == "unity":
            # Use avatar-mcp movement tools
            try:
                # This would call the mounted avatar-mcp server
                # For now, return mock response
                return {
                    "status": "success",
                    "message": f"Virtual robot {action} command sent (mock)",
                    "robot_id": robot.robot_id,
                    "action": action,
                    "platform": robot.platform,
                }
            except Exception as e:
                return {"status": "error", "message": str(e)}

        return {
            "status": "success",
            "message": f"Virtual robot {action} command sent (mock)",
            "robot_id": robot.robot_id,
            "action": action,
        }

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

