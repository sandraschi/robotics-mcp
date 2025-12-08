"""Robot virtual portmanteau tool - Virtual robot lifecycle and operations.

Consolidates virtual robot CRUD operations and virtual robotics operations into a single unified tool.
"""

from typing import Any, Dict, Literal, Optional

import structlog
from fastmcp import Client

from ..utils.error_handler import format_error_response, format_success_response, handle_tool_error
from ..utils.mcp_client_helper import call_mounted_server_tool

logger = structlog.get_logger(__name__)

SUPPORTED_ROBOT_TYPES = ["scout", "scout_e", "go2", "g1", "robbie", "custom"]


class RobotVirtualTool:
    """Portmanteau tool for virtual robot lifecycle and operations."""

    def __init__(self, mcp: Any, state_manager: Any, mounted_servers: Optional[Dict[str, Any]] = None):
        """Initialize robot virtual tool.

        Args:
            mcp: FastMCP server instance.
            state_manager: Robot state manager instance.
            mounted_servers: Dictionary of mounted MCP servers.
        """
        self.mcp = mcp
        self.state_manager = state_manager
        self.mounted_servers = mounted_servers or {}

    def register(self):
        """Register robot virtual tool with MCP server."""

        @self.mcp.tool()
        async def robot_virtual(
            operation: Literal[
                # CRUD operations
                "create",
                "read",
                "update",
                "delete",
                "list",
                # Virtual robot operations
                "spawn",
                "load_environment",
                "get_status",
                "get_lidar",
                "set_scale",
                "test_navigation",
                "sync_with_physical",
            ],
            robot_type: Optional[str] = None,
            robot_id: Optional[str] = None,
            platform: Literal["unity", "vrchat"] = "unity",
            position: Optional[Dict[str, float]] = None,
            scale: Optional[float] = None,
            metadata: Optional[Dict[str, Any]] = None,
            model_path: Optional[str] = None,
            environment: Optional[str] = None,
            environment_path: Optional[str] = None,
            project_path: Optional[str] = None,
            include_colliders: bool = True,
        ) -> Dict[str, Any]:
            """Virtual robot lifecycle and operations portmanteau.

            PORTMANTEAU PATTERN: Consolidates virtual robot CRUD operations and virtual
            robotics operations into a single unified tool. This reduces tool explosion
            while maintaining full functionality for virtual robot management.

            CRUD OPERATIONS:
            - create: Create/spawn and register a new virtual robot
            - read: Get details of an existing virtual robot
            - update: Modify virtual robot properties (scale, position, metadata, etc.)
            - delete: Remove and unregister a virtual robot
            - list: List all virtual robots with optional filtering

            VIRTUAL ROBOT OPERATIONS:
            - spawn: Spawn robot in Unity/VRChat scene (alias for create)
            - load_environment: Load Marble/Chisel environment into scene
            - get_status: Get virtual robot status
            - get_lidar: Get virtual LiDAR scan (Unity physics raycast)
            - set_scale: Scale robot size (for size testing)
            - test_navigation: Test pathfinding in environment
            - sync_with_physical: Sync vbot state with physical bot

            Args:
                operation: Operation to perform (see CRUD and Virtual Robot Operations above).
                robot_type: Type of robot (required for create/spawn).
                    Examples: "scout", "go2", "g1", "robbie", "custom"
                robot_id: Virtual robot identifier (required for read, update, delete, get_status, etc.).
                    Auto-generated for create/spawn if not provided.
                platform: Target platform ("unity" or "vrchat"). Default: "unity".
                position: Spawn/update position (x, y, z) for create/spawn/update.
                scale: Size multiplier for create/spawn/update/set_scale.
                metadata: Additional metadata dictionary for create/update.
                model_path: Path to 3D model file (.glb, .fbx, .vrm) for custom robot_type.
                environment: Environment name (Marble-generated) for load_environment.
                environment_path: Path to environment file for load_environment.
                project_path: Unity project path (optional, auto-detected if not provided).
                include_colliders: Whether to import collider meshes (default: True).

            Returns:
                Dictionary containing operation result with robot details.

            Examples:
                # Create a Scout vbot
                result = await robot_virtual(
                    operation="create",
                    robot_type="scout",
                    platform="unity",
                    position={"x": 0.0, "y": 0.0, "z": 0.0},
                    scale=1.0
                )

                # Spawn robot (alias for create)
                result = await robot_virtual(
                    operation="spawn",
                    robot_type="scout",
                    platform="unity"
                )

                # Read vbot details
                result = await robot_virtual(
                    operation="read",
                    robot_id="vbot_scout_01"
                )

                # Update vbot
                result = await robot_virtual(
                    operation="update",
                    robot_id="vbot_scout_01",
                    scale=1.5,
                    position={"x": 2.0, "y": 0.0, "z": 2.0}
                )

                # Load environment
                result = await robot_virtual(
                    operation="load_environment",
                    environment="stroheckgasse_apartment",
                    platform="unity"
                )

                # Get LiDAR scan
                result = await robot_virtual(
                    operation="get_lidar",
                    robot_id="vbot_scout_01"
                )

                # List all vbots
                result = await robot_virtual(operation="list")
            """
            try:
                # Route to appropriate handler
                if operation in ["create", "spawn"]:
                    return await self._handle_create(robot_type, robot_id, platform, position, scale, metadata, model_path)
                elif operation == "read":
                    return await self._handle_read(robot_id)
                elif operation == "update":
                    return await self._handle_update(robot_id, position, scale, metadata)
                elif operation == "delete":
                    return await self._handle_delete(robot_id)
                elif operation == "list":
                    return await self._handle_list(robot_type, platform)
                elif operation == "load_environment":
                    return await self._handle_load_environment(environment, platform, environment_path, project_path, include_colliders)
                elif operation == "get_status":
                    return await self._handle_get_status(robot_id)
                elif operation == "get_lidar":
                    return await self._handle_get_lidar(robot_id)
                elif operation == "set_scale":
                    return await self._handle_set_scale(robot_id, scale)
                elif operation == "test_navigation":
                    return await self._handle_test_navigation(robot_id, environment)
                elif operation == "sync_with_physical":
                    return await self._handle_sync_with_physical(robot_id)
                else:
                    return format_error_response(f"Unknown operation: {operation}", error_type="validation_error")
            except Exception as e:
                return handle_tool_error("robot_virtual", e, operation=operation, robot_type=robot_type, robot_id=robot_id)

    # CRUD handlers (from vbot_crud.py)
    async def _handle_create(
        self,
        robot_type: Optional[str],
        robot_id: Optional[str],
        platform: str,
        position: Optional[Dict[str, float]],
        scale: Optional[float],
        metadata: Optional[Dict[str, Any]],
        model_path: Optional[str],
    ) -> Dict[str, Any]:
        """Create/spawn a new virtual robot."""
        if not robot_type:
            return format_error_response("robot_type is required for create/spawn operation", error_type="validation_error")
        if robot_type not in SUPPORTED_ROBOT_TYPES:
            return format_error_response(
                f"Unsupported robot_type: {robot_type}. Supported: {', '.join(SUPPORTED_ROBOT_TYPES)}",
                error_type="validation_error",
            )
        if robot_type == "custom" and not model_path:
            return format_error_response("model_path is required for custom robot_type", error_type="validation_error")
        if not robot_id:
            robot_id = f"vbot_{robot_type}_{len(self.state_manager.list_robots(is_virtual=True)) + 1:02d}"
        if self.state_manager.get_robot(robot_id):
            return format_error_response(f"Robot {robot_id} already exists", error_type="validation_error")
        position = position or {"x": 0.0, "y": 0.0, "z": 0.0}
        scale = scale or 1.0
        vbot_metadata = {
            "spawned": True,
            "platform": platform,
            "position": position,
            "scale": scale,
            "model_path": model_path,
            **(metadata or {}),
        }
        try:
            robot = self.state_manager.register_robot(robot_id, robot_type, platform=platform, metadata=vbot_metadata)
        except ValueError as e:
            return format_error_response(str(e), error_type="validation_error")
        spawn_result = await self._spawn_in_platform(robot_id, robot_type, platform, position, scale, model_path)
        if spawn_result.get("status") != "success":
            self.state_manager.unregister_robot(robot_id)
            return spawn_result
        return format_success_response(
            f"Virtual robot {robot_id} created successfully",
            data={"robot_id": robot_id, "robot_type": robot_type, "platform": platform, "position": position, "scale": scale},
            robot_id=robot_id,
        )

    async def _handle_read(self, robot_id: Optional[str]) -> Dict[str, Any]:
        """Read/get details of an existing virtual robot."""
        if not robot_id:
            return format_error_response("robot_id is required for read operation", error_type="validation_error")
        robot = self.state_manager.get_robot(robot_id)
        if not robot:
            return format_error_response(f"Virtual robot {robot_id} not found", error_type="not_found", robot_id=robot_id)
        if not robot.is_virtual:
            return format_error_response(f"Robot {robot_id} is not a virtual robot", error_type="validation_error")
        return format_success_response(f"Virtual robot {robot_id} details retrieved", data=robot.to_dict(), robot_id=robot_id)

    async def _handle_update(
        self,
        robot_id: Optional[str],
        position: Optional[Dict[str, float]],
        scale: Optional[float],
        metadata: Optional[Dict[str, Any]],
    ) -> Dict[str, Any]:
        """Update properties of an existing virtual robot."""
        if not robot_id:
            return format_error_response("robot_id is required for update operation", error_type="validation_error")
        robot = self.state_manager.get_robot(robot_id)
        if not robot:
            return format_error_response(f"Virtual robot {robot_id} not found", error_type="not_found", robot_id=robot_id)
        if not robot.is_virtual:
            return format_error_response(f"Robot {robot_id} is not a virtual robot", error_type="validation_error")
        updates = {}
        if position is not None:
            robot.metadata["position"] = position
            updates["position"] = position
        if scale is not None:
            robot.metadata["scale"] = scale
            updates["scale"] = scale
        if metadata is not None:
            robot.metadata.update(metadata)
            updates["metadata"] = metadata
        if position is not None or scale is not None:
            update_result = await self._update_in_platform(robot_id, robot.platform, position, scale)
            if update_result.get("status") != "success":
                return update_result
        return format_success_response(
            f"Virtual robot {robot_id} updated successfully",
            data={"robot_id": robot_id, "updates": updates, "robot": robot.to_dict()},
            robot_id=robot_id,
        )

    async def _handle_delete(self, robot_id: Optional[str]) -> Dict[str, Any]:
        """Delete/remove a virtual robot."""
        if not robot_id:
            return format_error_response("robot_id is required for delete operation", error_type="validation_error")
        robot = self.state_manager.get_robot(robot_id)
        if not robot:
            return format_error_response(f"Virtual robot {robot_id} not found", error_type="not_found", robot_id=robot_id)
        if not robot.is_virtual:
            return format_error_response(f"Robot {robot_id} is not a virtual robot", error_type="validation_error")
        delete_result = await self._delete_from_platform(robot_id, robot.platform)
        if delete_result.get("status") != "success":
            logger.warning("Failed to delete from platform, but continuing with unregister", robot_id=robot_id)
        self.state_manager.unregister_robot(robot_id)
        return format_success_response(f"Virtual robot {robot_id} deleted successfully", data={"robot_id": robot_id}, robot_id=robot_id)

    async def _handle_list(self, robot_type: Optional[str], platform: Optional[str]) -> Dict[str, Any]:
        """List all virtual robots with optional filtering."""
        robots = self.state_manager.list_robots(is_virtual=True)
        if robot_type:
            robots = [r for r in robots if r.robot_type == robot_type]
        if platform:
            robots = [r for r in robots if r.platform == platform]
        return format_success_response(
            f"Found {len(robots)} virtual robot(s)",
            data={"count": len(robots), "robots": [r.to_dict() for r in robots], "filters": {"robot_type": robot_type, "platform": platform}},
        )

    # Virtual robot operation handlers (from virtual_robotics.py)
    async def _handle_load_environment(
        self, environment: str, platform: str, environment_path: Optional[str], project_path: Optional[str], include_colliders: bool
    ) -> Dict[str, Any]:
        """Load Marble/Chisel environment."""
        logger.info("Loading environment", environment=environment, platform=platform, environment_path=environment_path)
        try:
            if platform == "unity" and "unity" in self.mounted_servers:
                source_path = environment_path or environment
                if source_path and source_path.lower().endswith('.spz'):
                    return format_error_response(
                        ".spz files are not supported by Unity. Use robot_model(operation='spz_convert') or re-export from Marble as .ply/.fbx/.glb.",
                        error_type="unsupported_format",
                    )
                result = await call_mounted_server_tool(
                    self.mounted_servers,
                    "unity",
                    "import_marble_world",
                    {"source_path": source_path, "project_path": project_path or "", "include_colliders": include_colliders},
                )
                return format_success_response(
                    f"Environment {environment} loaded via Unity",
                    action="load_environment",
                    data={"environment": environment, "platform": platform, "unity_result": result},
                )
            else:
                return format_success_response(
                    f"Environment {environment} loaded (mock - Unity MCP not available)",
                    action="load_environment",
                    data={"environment": environment, "platform": platform, "note": "Unity MCP not available, using mock"},
                )
        except Exception as e:
            return handle_tool_error("_handle_load_environment", e, context={"environment": environment, "platform": platform})

    async def _handle_get_status(self, robot_id: Optional[str]) -> Dict[str, Any]:
        """Get virtual robot status."""
        if not robot_id:
            return format_error_response("robot_id required", error_type="validation_error")
        robot = self.state_manager.get_robot(robot_id)
        if not robot:
            return format_error_response(f"Robot {robot_id} not found", error_type="not_found", robot_id=robot_id)
        return format_success_response(f"Robot {robot_id} status retrieved", robot_id=robot_id, data={"robot": robot.to_dict()})

    async def _handle_get_lidar(self, robot_id: Optional[str]) -> Dict[str, Any]:
        """Get virtual LiDAR scan."""
        if not robot_id:
            return format_error_response("robot_id required", error_type="validation_error")
        robot = self.state_manager.get_robot(robot_id)
        if not robot or not robot.is_virtual:
            return format_error_response(f"Virtual robot {robot_id} not found", error_type="not_found", robot_id=robot_id)
        if robot.platform == "unity" and "unity" in self.mounted_servers:
            result = await call_mounted_server_tool(
                self.mounted_servers,
                "unity",
                "execute_unity_method",
                {
                    "class_name": "VirtualLiDAR",
                    "method_name": "PerformScan",
                    "parameters": {"robotId": robot_id},
                },
            )
            return format_success_response(
                f"LiDAR scan retrieved for {robot_id}",
                robot_id=robot_id,
                data={"scan_data": result.get("scan_data", {}), "method": "unity_raycast"},
            )
        else:
            return format_success_response(
                f"LiDAR scan retrieved for {robot_id} (mock)",
                robot_id=robot_id,
                data={"scan_data": {"points": []}, "method": "mock"},
            )

    async def _handle_set_scale(self, robot_id: Optional[str], scale: Optional[float]) -> Dict[str, Any]:
        """Set robot scale."""
        if not robot_id:
            return format_error_response("robot_id required", error_type="validation_error")
        if scale is None:
            return format_error_response("scale required", error_type="validation_error", robot_id=robot_id)
        robot = self.state_manager.get_robot(robot_id)
        if not robot:
            return format_error_response(f"Robot {robot_id} not found", error_type="not_found", robot_id=robot_id)
        if robot.platform == "unity" and "unity" in self.mounted_servers:
            await call_mounted_server_tool(
                self.mounted_servers,
                "unity",
                "execute_unity_method",
                {
                    "class_name": "RobotController",
                    "method_name": "SetScale",
                    "parameters": {"robotId": robot_id, "scale": scale},
                },
            )
        robot.metadata["scale"] = scale
        return format_success_response(f"Robot scale set to {scale}", robot_id=robot_id, data={"scale": scale})

    async def _handle_test_navigation(self, robot_id: Optional[str], environment: Optional[str]) -> Dict[str, Any]:
        """Test navigation in environment."""
        return format_success_response(
            "Navigation test completed (mock)",
            robot_id=robot_id,
            data={"environment": environment, "note": "Navigation testing not yet implemented"},
        )

    async def _handle_sync_with_physical(self, robot_id: Optional[str]) -> Dict[str, Any]:
        """Sync virtual robot with physical robot state."""
        return format_success_response(
            "Virtual robot synced with physical (mock)",
            robot_id=robot_id,
            data={"note": "Physical robot sync not yet implemented"},
        )

    # Platform interaction helpers
    async def _spawn_in_platform(
        self, robot_id: str, robot_type: str, platform: str, position: Dict[str, float], scale: float, model_path: Optional[str]
    ) -> Dict[str, Any]:
        """Spawn robot in Unity or VRChat."""
        try:
            if platform == "unity" and "unity" in self.mounted_servers:
                pos = position or {"x": 0.0, "y": 0.0, "z": 0.0}
                result = await call_mounted_server_tool(
                    self.mounted_servers,
                    "unity",
                    "execute_unity_method",
                    {
                        "class_name": "VbotSpawner",
                        "method_name": "SpawnRobot",
                        "parameters": {
                            "robotId": robot_id,
                            "robotType": robot_type,
                            "position": {"x": pos.get("x", 0.0), "y": pos.get("y", 0.0), "z": pos.get("z", 0.0)},
                            "scale": scale,
                        },
                    },
                )
                return result
            else:
                logger.info("Mock spawn (platform not available)", robot_id=robot_id, platform=platform)
                return format_success_response(f"Mock spawn: {robot_id} in {platform}")
        except Exception as e:
            logger.error("Failed to spawn in platform", robot_id=robot_id, platform=platform, error=str(e))
            return format_error_response(f"Failed to spawn in {platform}: {str(e)}", error_type="connection_error")

    async def _update_in_platform(
        self, robot_id: str, platform: str, position: Optional[Dict[str, float]], scale: Optional[float]
    ) -> Dict[str, Any]:
        """Update robot in Unity or VRChat."""
        try:
            if platform == "unity" and "unity" in self.mounted_servers:
                pos = None
                if position:
                    pos = {"x": position.get("x", 0.0), "y": position.get("y", 0.0), "z": position.get("z", 0.0)}
                result = await call_mounted_server_tool(
                    self.mounted_servers,
                    "unity",
                    "execute_unity_method",
                    {
                        "class_name": "VbotSpawner",
                        "method_name": "UpdateRobot",
                        "parameters": {"robotId": robot_id, "position": pos, "scale": scale},
                    },
                )
                return result
            else:
                return format_success_response(f"Mock update: {robot_id}")
        except Exception as e:
            logger.error("Failed to update in platform", robot_id=robot_id, platform=platform, error=str(e))
            return format_error_response(f"Failed to update in {platform}: {str(e)}", error_type="connection_error")

    async def _delete_from_platform(self, robot_id: str, platform: str) -> Dict[str, Any]:
        """Delete robot from Unity or VRChat."""
        try:
            if platform == "unity" and "unity" in self.mounted_servers:
                result = await call_mounted_server_tool(
                    self.mounted_servers,
                    "unity",
                    "execute_unity_method",
                    {"class_name": "VbotSpawner", "method_name": "DeleteRobot", "parameters": {"robotId": robot_id}},
                )
                return result
            else:
                return format_success_response(f"Mock delete: {robot_id}")
        except Exception as e:
            logger.error("Failed to delete from platform", robot_id=robot_id, platform=platform, error=str(e))
            return format_error_response(f"Failed to delete from {platform}: {str(e)}", error_type="connection_error")
