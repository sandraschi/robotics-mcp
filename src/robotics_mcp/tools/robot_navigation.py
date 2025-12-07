"""Robot navigation portmanteau tool - Path planning and navigation.

Provides unified navigation control for both physical and virtual robots.
"""

from typing import Any, Dict, List, Literal, Optional

import structlog
from fastmcp import Client

from ..utils.error_handler import format_error_response, format_success_response, handle_tool_error

logger = structlog.get_logger(__name__)


class RobotNavigationTool:
    """Portmanteau tool for robot navigation and path planning."""

    def __init__(self, mcp: Any, state_manager: Any, mounted_servers: Optional[Dict[str, Any]] = None):
        """Initialize robot navigation tool.

        Args:
            mcp: FastMCP server instance.
            state_manager: Robot state manager instance.
            mounted_servers: Dictionary of mounted MCP servers.
        """
        self.mcp = mcp
        self.state_manager = state_manager
        self.mounted_servers = mounted_servers or {}

    def register(self):
        """Register robot navigation tool with MCP server."""

        @self.mcp.tool()
        async def robot_navigation(
            robot_id: str,
            action: Literal[
                "plan_path",
                "follow_path",
                "set_waypoint",
                "clear_waypoints",
                "get_path_status",
                "avoid_obstacle",
                "get_current_path",
            ],
            start_position: Optional[Dict[str, float]] = None,
            goal_position: Optional[Dict[str, float]] = None,
            waypoint: Optional[Dict[str, float]] = None,
            obstacle_position: Optional[Dict[str, float]] = None,
            path_id: Optional[str] = None,
        ) -> Dict[str, Any]:
            """Robot navigation and path planning portmanteau.

            PORTMANTEAU PATTERN: Consolidates navigation operations into a single tool.

            SUPPORTED OPERATIONS:
            - plan_path: Plan path from A to B (A* or RRT)
            - follow_path: Execute planned path
            - set_waypoint: Set navigation waypoint
            - clear_waypoints: Clear waypoint list
            - get_path_status: Check path execution status
            - avoid_obstacle: Dynamic obstacle avoidance
            - get_current_path: Get current path being followed

            Args:
                robot_id: Robot identifier (e.g., "scout_01", "vbot_scout_01").
                action: Navigation operation to perform.
                start_position: Start position (x, y, z) for plan_path.
                goal_position: Goal position (x, y, z) for plan_path.
                waypoint: Waypoint position (x, y, z) for set_waypoint.
                obstacle_position: Obstacle position (x, y, z) for avoid_obstacle.
                path_id: Path identifier for follow_path or get_path_status.

            Returns:
                Dictionary containing operation result with path data or status.

            Examples:
                Plan path:
                    result = await robot_navigation(
                        robot_id="scout_01",
                        action="plan_path",
                        start_position={"x": 0, "y": 0, "z": 0},
                        goal_position={"x": 5, "y": 0, "z": 0}
                    )

                Follow path:
                    result = await robot_navigation(
                        robot_id="scout_01",
                        action="follow_path",
                        path_id="path_123"
                    )

                Set waypoint:
                    result = await robot_navigation(
                        robot_id="scout_01",
                        action="set_waypoint",
                        waypoint={"x": 2, "y": 0, "z": 0}
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
                    return await self._handle_virtual_navigation(
                        robot, action, start_position, goal_position, waypoint, obstacle_position, path_id
                    )
                else:
                    return await self._handle_physical_navigation(
                        robot, action, start_position, goal_position, waypoint, obstacle_position, path_id
                    )

            except Exception as e:
                return handle_tool_error("robot_navigation", e, robot_id=robot_id, action=action)

    async def _handle_virtual_navigation(
        self,
        robot: Any,
        action: str,
        start_position: Optional[Dict[str, float]],
        goal_position: Optional[Dict[str, float]],
        waypoint: Optional[Dict[str, float]],
        obstacle_position: Optional[Dict[str, float]],
        path_id: Optional[str],
    ) -> Dict[str, Any]:
        """Handle virtual robot navigation."""
        try:
            if robot.platform == "unity" and "unity" in self.mounted_servers:
                async with Client(self.mcp) as client:
                    if action == "plan_path":
                        # Call Unity RobotNavigator.PlanPath()
                        result = await client.call_tool(
                            "execute_unity_method",
                            {
                                "class_name": "RobotNavigator",
                                "method_name": "PlanPath",
                                "parameters": {
                                    "robotId": robot.robot_id,
                                    "startPosition": start_position or {},
                                    "goalPosition": goal_position or {},
                                },
                            },
                        )
                        return format_success_response(
                            f"Path planned for {robot.robot_id}",
                            robot_id=robot.robot_id,
                            action=action,
                            data=result,
                        )

                    elif action == "follow_path":
                        # Call Unity RobotNavigator.FollowPath()
                        result = await client.call_tool(
                            "execute_unity_method",
                            {
                                "class_name": "RobotNavigator",
                                "method_name": "FollowPath",
                                "parameters": {
                                    "robotId": robot.robot_id,
                                    "pathId": path_id or "",
                                },
                            },
                        )
                        return format_success_response(
                            f"Path following started for {robot.robot_id}",
                            robot_id=robot.robot_id,
                            action=action,
                            data=result,
                        )

                    elif action == "set_waypoint":
                        # Call Unity RobotNavigator.SetWaypoint()
                        result = await client.call_tool(
                            "execute_unity_method",
                            {
                                "class_name": "RobotNavigator",
                                "method_name": "SetWaypoint",
                                "parameters": {
                                    "robotId": robot.robot_id,
                                    "waypoint": waypoint or {},
                                },
                            },
                        )
                        return format_success_response(
                            f"Waypoint set for {robot.robot_id}",
                            robot_id=robot.robot_id,
                            action=action,
                            data=result,
                        )

                    elif action == "clear_waypoints":
                        # Call Unity RobotNavigator.ClearWaypoints()
                        result = await client.call_tool(
                            "execute_unity_method",
                            {
                                "class_name": "RobotNavigator",
                                "method_name": "ClearWaypoints",
                                "parameters": {
                                    "robotId": robot.robot_id,
                                },
                            },
                        )
                        return format_success_response(
                            f"Waypoints cleared for {robot.robot_id}",
                            robot_id=robot.robot_id,
                            action=action,
                            data=result,
                        )

                    elif action == "get_path_status":
                        # Call Unity RobotNavigator.GetPathStatus()
                        result = await client.call_tool(
                            "execute_unity_method",
                            {
                                "class_name": "RobotNavigator",
                                "method_name": "GetPathStatus",
                                "parameters": {
                                    "robotId": robot.robot_id,
                                    "pathId": path_id or "",
                                },
                            },
                        )
                        return format_success_response(
                            f"Path status retrieved for {robot.robot_id}",
                            robot_id=robot.robot_id,
                            action=action,
                            data=result,
                        )

                    elif action == "avoid_obstacle":
                        # Call Unity RobotNavigator.AvoidObstacle()
                        result = await client.call_tool(
                            "execute_unity_method",
                            {
                                "class_name": "RobotNavigator",
                                "method_name": "AvoidObstacle",
                                "parameters": {
                                    "robotId": robot.robot_id,
                                    "obstaclePosition": obstacle_position or {},
                                },
                            },
                        )
                        return format_success_response(
                            f"Obstacle avoidance triggered for {robot.robot_id}",
                            robot_id=robot.robot_id,
                            action=action,
                            data=result,
                        )

                    elif action == "get_current_path":
                        # Call Unity RobotNavigator.GetCurrentPath()
                        result = await client.call_tool(
                            "execute_unity_method",
                            {
                                "class_name": "RobotNavigator",
                                "method_name": "GetCurrentPath",
                                "parameters": {
                                    "robotId": robot.robot_id,
                                },
                            },
                        )
                        return format_success_response(
                            f"Current path retrieved for {robot.robot_id}",
                            robot_id=robot.robot_id,
                            action=action,
                            data=result,
                        )

            else:
                # Mock navigation for testing
                logger.info("Mock navigation", robot_id=robot.robot_id, action=action)
                mock_path = {
                    "path_id": path_id or "mock_path_123",
                    "waypoints": [
                        start_position or {"x": 0, "y": 0, "z": 0},
                        goal_position or {"x": 5, "y": 0, "z": 0},
                    ],
                    "status": "planned",
                }
                return format_success_response(
                    f"Mock navigation: {action} for {robot.robot_id}",
                    robot_id=robot.robot_id,
                    action=action,
                    data={"note": "Mock mode - Unity not available", "path": mock_path},
                )

        except Exception as e:
            return handle_tool_error("_handle_virtual_navigation", e, robot_id=robot.robot_id, action=action)

    async def _handle_physical_navigation(
        self,
        robot: Any,
        action: str,
        start_position: Optional[Dict[str, float]],
        goal_position: Optional[Dict[str, float]],
        waypoint: Optional[Dict[str, float]],
        obstacle_position: Optional[Dict[str, float]],
        path_id: Optional[str],
    ) -> Dict[str, Any]:
        """Handle physical robot navigation."""
        # TODO: Implement ROS-based navigation (move_base, amcl)
        logger.info("Physical robot navigation", robot_id=robot.robot_id, action=action)
        return format_success_response(
            f"Physical robot navigation: {action} for {robot.robot_id}",
            robot_id=robot.robot_id,
            action=action,
            data={"note": "Physical robot navigation not yet implemented (requires ROS navigation stack)"},
        )

