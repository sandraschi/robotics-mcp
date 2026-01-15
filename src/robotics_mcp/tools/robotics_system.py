"""Robotics system portmanteau tool - System management operations.

Consolidates help, status, and robot listing into a single portmanteau tool.
"""

from typing import Any, Dict, Literal, Optional

import structlog

from ..utils.error_handler import (
    format_error_response,
    format_success_response,
    handle_tool_error,
)
from ..utils.response_builders import (
    build_success_response,
    build_error_response,
    build_hardware_error_response,
    build_network_error_response,
    build_configuration_error_response,
    build_robotics_error_response,
)

logger = structlog.get_logger(__name__)


class RoboticsSystemTool:
    """Portmanteau tool for system management operations."""

    def __init__(
        self,
        mcp: Any,
        state_manager: Any,
        config: Any,
        config_loader: Any,
        mounted_servers: Dict[str, Any],
    ):
        """Initialize robotics system tool.

        Args:
            mcp: FastMCP server instance.
            state_manager: Robot state manager instance.
            config: Server configuration.
            config_loader: Configuration loader instance.
            mounted_servers: Dictionary of mounted MCP servers.
        """
        self.mcp = mcp
        self.state_manager = state_manager
        self.config = config
        self.config_loader = config_loader
        self.mounted_servers = mounted_servers

    def register(self):
        """Register robotics system tool with MCP server."""

        @self.mcp.tool()
        async def robotics_system(
            operation: Literal["help", "status", "list_robots"],
            robot_type: Optional[str] = None,
            is_virtual: Optional[bool] = None,
        ) -> Dict[str, Any]:
            """System management portmanteau for Robotics MCP.

            PORTMANTEAU PATTERN RATIONALE:
            Instead of creating 3 separate tools (help, status, list_robots), this tool
            consolidates related system operations into a single interface. This design:
            - Prevents tool explosion (3 tools -> 1 tool) while maintaining full functionality
            - Improves discoverability by grouping related operations together
            - Reduces cognitive load when working with system management tasks
            - Enables consistent system interface across all operations
            - Follows FastMCP 2.13+ best practices for feature-rich MCP servers

            SUPPORTED OPERATIONS:
            - help: Get comprehensive help information about the server and its tools
            - status: Get server status with connectivity tests and robot counts
            - list_robots: List all registered robots with optional filtering

            Args:
                operation: The system operation to perform. MUST be one of:
                    - "help": Get help information (no additional parameters)
                    - "status": Get server status (no additional parameters)
                    - "list_robots": List robots (optional: robot_type, is_virtual filters)

                robot_type: Optional filter for list_robots operation.
                    Valid values: "scout", "go2", "g1", "dreame", or any custom robot type.
                    If None, returns all robot types.

                is_virtual: Optional filter for list_robots operation.
                    - True: Only virtual robots (vbots)
                    - False: Only physical robots (bots)
                    - None: Both virtual and physical robots

            Returns:
                Dictionary containing operation-specific results:
                - help: Server info, tool list, features, mounted servers
                - status: Server health, robot counts, connectivity tests, HTTP status
                - list_robots: Robot list with filtering applied

            Examples:
                Get help information:
                    result = await robotics_system(operation="help")

                Get server status:
                    result = await robotics_system(operation="status")

                List all robots:
                    result = await robotics_system(operation="list_robots")

                List only Scout robots:
                    result = await robotics_system(operation="list_robots", robot_type="scout")

                List only virtual robots:
                    result = await robotics_system(operation="list_robots", is_virtual=True)
            """
            try:
                if operation == "help":
                    return await self._handle_help()
                elif operation == "status":
                    return await self._handle_status()
                elif operation == "list_robots":
                    return await self._handle_list_robots(robot_type, is_virtual)
                else:
                    return format_error_response(
                        f"Unknown operation: {operation}",
                        error_type="validation_error",
                        operation=operation,
                    )
            except Exception as e:
                return handle_tool_error("robotics_system", e, operation=operation)

    async def _handle_help(self) -> Dict[str, Any]:
        """Handle help operation."""
        try:
            # Get all registered tools
            tools_info = []
            # FastMCP stores tools in _tools dict - get description from docstring
            for tool_name, tool_func in getattr(self.mcp, "_tools", {}).items():
                description = ""
                if hasattr(tool_func, "__doc__") and tool_func.__doc__:
                    # Get first line of docstring as description
                    description = tool_func.__doc__.split("\n")[0].strip()
                tools_info.append(
                    {
                        "name": tool_name,
                        "description": description,
                    }
                )

            return format_success_response(
                "Help information retrieved",
                data={
                    "server_name": "Robotics-MCP",
                    "version": "0.1.0",
                    "description": (
                        "Unified robotics control via MCP - Physical and virtual robots (bot + vbot). "
                        "Provides comprehensive control for Moorebot Scout, Unitree robots, Dreame vacuums, and virtual "
                        "robots in Unity/VRChat. Integrates with osc-mcp, unity3d-mcp, vrchat-mcp, and "
                        "avatar-mcp for seamless virtual robotics testing."
                    ),
                    "features": [
                        "Physical robot control (ROS 1.4 via rosbridge)",
                        "Virtual robot control (Unity3D/VRChat/Resonite)",
                        "YDLIDAR SuperLight (95g) LiDAR integration",
                        "World Labs Marble/Chisel environment generation",
                        "Multi-robot coordination",
                        "Dual transport (stdio + HTTP)",
                    ],
                    "tools": tools_info,
                    "mounted_servers": list(self.mounted_servers.keys()),
                    "configuration": {
                        "http_enabled": self.config.enable_http,
                        "http_port": self.config.http_port
                        if self.config.enable_http
                        else None,
                        "config_path": str(self.config_loader.config_path),
                    },
                },
            )
        except Exception as e:
            logger.error("Failed to generate help", error=str(e), exc_info=True)
            return build_error_response(
                error="Failed to generate help information",
                error_code="HELP_GENERATION_FAILED",
                message="Unable to generate comprehensive help information due to an internal error",
                recovery_options=[
                    "Try the help operation again",
                    "Check server logs for detailed error information",
                    "Restart the Robotics MCP server if issues persist"
                ],
                suggestions=[
                    "Use individual tool operations directly if help generation fails",
                    "Check the README.md file for basic usage information"
                ]
            )

    async def _handle_status(self) -> Dict[str, Any]:
        """Handle status operation."""
        try:
            robots = self.state_manager.list_robots()

            # Test mounted server connectivity
            mounted_servers_status: Dict[str, Any] = {}
            for server_name, server_instance in self.mounted_servers.items():
                try:
                    if hasattr(server_instance, "list_tools"):
                        tools = server_instance.list_tools()
                        mounted_servers_status[server_name] = {
                            "available": True,
                            "tools_count": len(tools) if isinstance(tools, dict) else 0,
                        }
                    else:
                        mounted_servers_status[server_name] = {
                            "available": True,
                            "tools_count": "unknown",
                        }
                except Exception as e:
                    logger.warning(
                        "Mounted server connectivity test failed",
                        server=server_name,
                        error=str(e),
                    )
                    mounted_servers_status[server_name] = {
                        "available": False,
                        "error": str(e),
                    }

            # Test HTTP server if enabled
            http_status = None
            if self.config.enable_http:
                try:
                    import socket

                    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    sock.settimeout(1)
                    result = sock.connect_ex(
                        (self.config.http_host, self.config.http_port)
                    )
                    sock.close()
                    http_status = {
                        "enabled": True,
                        "host": self.config.http_host,
                        "port": self.config.http_port,
                        "reachable": result == 0,
                    }
                except Exception as e:
                    logger.warning("HTTP server status check failed", error=str(e))
                    http_status = {
                        "enabled": True,
                        "host": self.config.http_host,
                        "port": self.config.http_port,
                        "reachable": False,
                        "error": str(e),
                    }

            # Build conversational status response
            robots_count = len(robots)
            mounted_count = len([s for s in mounted_servers_status.values() if s.get("available")])

            status_data = {
                "version": "0.1.0",
                "status": "healthy",
                "robots": [r.to_dict() for r in robots],
                "robots_count": robots_count,
                "mounted_servers": mounted_servers_status,
                "http": http_status,
                "config": {
                    "http_enabled": self.config.enable_http,
                    "log_level": self.config.log_level,
                },
            }

            recommendations = []
            next_steps = []

            if robots_count > 0:
                recommendations.append("Use 'robot_behavior' tool to control robot movements and actions")
                recommendations.append("Use 'robot_virtual' tool to spawn virtual robots in Unity/VRChat")
                next_steps.append("Try controlling a robot with 'robot_behavior get_status' action")
            else:
                recommendations.append("No robots currently registered - spawn virtual robots or connect physical robots")
                next_steps.append("Use 'robot_virtual spawn' to create virtual robots for testing")
                next_steps.append("Check physical robot connectivity if available")

            if mounted_count > 0:
                recommendations.append(f"{mounted_count} MCP servers mounted - full robotics ecosystem available")
            else:
                recommendations.append("No external MCP servers mounted - limited functionality available")
                next_steps.append("Check MCP server mounting configuration for full features")

            return build_success_response(
                operation="server_status",
                summary=f"Robotics MCP healthy with {robots_count} robots and {mounted_count} mounted servers",
                result=status_data,
                recommendations=recommendations,
                next_steps=next_steps
            )
        except Exception as e:
            return handle_tool_error("robotics_system", e, operation="status")

    async def _handle_list_robots(
        self, robot_type: Optional[str], is_virtual: Optional[bool]
    ) -> Dict[str, Any]:
        """Handle list_robots operation."""
        try:
            robots = self.state_manager.list_robots()

            # Apply filters
            filtered_robots = robots
            if robot_type:
                filtered_robots = [
                    r for r in filtered_robots if r.robot_type == robot_type
                ]
            if is_virtual is not None:
                filtered_robots = [
                    r for r in filtered_robots if r.is_virtual == is_virtual
                ]

            robot_count = len(filtered_robots)
            filter_desc = []
            if robot_type:
                filter_desc.append(f"type: {robot_type}")
            if is_virtual is not None:
                filter_desc.append(f"virtual: {is_virtual}")

            filter_summary = f" ({', '.join(filter_desc)})" if filter_desc else ""

            robots_data = {
                "count": robot_count,
                "robots": [r.to_dict() for r in filtered_robots],
                "filters": {
                    "robot_type": robot_type,
                    "is_virtual": is_virtual,
                },
            }

            recommendations = []
            next_steps = []

            if robot_count > 0:
                recommendations.append("Use 'robot_behavior' tool to control robot movements and sensors")
                recommendations.append("Use 'robot_virtual' tool for Unity/VRChat integration")
                next_steps.append("Try 'robot_behavior get_status' to check robot state")
                next_steps.append("Use 'robot_behavior move' for basic movement control")
            else:
                recommendations.append("No robots match the specified filters")
                if robot_type or is_virtual is not None:
                    recommendations.append("Try removing filters to see all available robots")
                    next_steps.append("Use 'robotics_system list_robots' without filters")
                else:
                    recommendations.append("No robots currently registered in the system")
                    next_steps.append("Use 'robot_virtual spawn' to create virtual robots")
                    next_steps.append("Check physical robot connectivity and configuration")

            return build_success_response(
                operation="list_robots",
                summary=f"Found {robot_count} robot{'s' if robot_count != 1 else ''}{filter_summary}",
                result=robots_data,
                recommendations=recommendations,
                next_steps=next_steps
            )
        except Exception as e:
            logger.error("Failed to list robots", error=str(e), exc_info=True)

            # Intelligent error analysis for robot listing issues
            error_str = str(e).lower()
            recovery_options = []

            if "database" in error_str or "sqlite" in error_str:
                recovery_options = [
                    "Check robot state database file integrity",
                    "Restart Robotics MCP server to reinitialize state",
                    "Check file system permissions for database access",
                    "Verify database is not corrupted (backup and restore if needed)"
                ]
            elif "memory" in error_str or "state" in error_str:
                recovery_options = [
                    "Restart Robotics MCP server to reinitialize state manager",
                    "Check system memory availability",
                    "Verify state manager initialization completed successfully"
                ]
            else:
                recovery_options = [
                    "Try the list_robots operation again",
                    "Check server logs for detailed error information",
                    "Restart Robotics MCP server if state corruption is suspected"
                ]

            return build_error_response(
                error="Failed to retrieve robot list",
                error_code="LIST_ROBOTS_FAILED",
                message="Unable to list robots due to an internal error",
                recovery_options=recovery_options,
                suggestions=[
                    "Use 'robotics_system status' to check server health first",
                    "Try individual robot operations to test connectivity",
                    "Restart the Robotics MCP server if issues persist"
                ]
            )
