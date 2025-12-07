"""Virtual Robot CRUD tool - Create, Read, Update, Delete for virtual robots."""

from typing import Any, Dict, Literal, Optional

import structlog
from fastmcp import Client

from ..utils.error_handler import format_error_response, format_success_response, handle_tool_error

logger = structlog.get_logger(__name__)

# Supported robot types
SUPPORTED_ROBOT_TYPES = [
    "scout",  # Moorebot Scout
    "scout_e",  # Moorebot Scout E (tracked, waterproof)
    "go2",  # Unitree Go2
    "g1",  # Unitree G1
    "robbie",  # Robbie from Forbidden Planet
    "custom",  # Custom robot type
]


class VbotCrudTool:
    """CRUD operations for virtual robots (vbots)."""

    def __init__(self, mcp: Any, state_manager: Any, mounted_servers: Optional[Dict[str, Any]] = None):
        """Initialize vbot CRUD tool.

        Args:
            mcp: FastMCP server instance.
            state_manager: Robot state manager instance.
            mounted_servers: Dictionary of mounted MCP servers.
        """
        self.mcp = mcp
        self.state_manager = state_manager
        self.mounted_servers = mounted_servers or {}

    def register(self):
        """Register vbot CRUD tool with MCP server."""

        @self.mcp.tool()
        async def vbot_crud(
            operation: Literal["create", "read", "update", "delete", "list"],
            robot_type: Optional[str] = None,
            robot_id: Optional[str] = None,
            platform: Literal["unity", "vrchat"] = "unity",
            position: Optional[Dict[str, float]] = None,
            scale: Optional[float] = None,
            metadata: Optional[Dict[str, Any]] = None,
            model_path: Optional[str] = None,
        ) -> Dict[str, Any]:
            """CRUD operations for virtual robots (vbots).

            This tool provides complete lifecycle management for virtual robots:
            - Create: Spawn and register a new virtual robot
            - Read: Get details of an existing virtual robot
            - Update: Modify virtual robot properties (scale, position, metadata, etc.)
            - Delete: Remove and unregister a virtual robot
            - List: List all virtual robots with optional filtering

            Supported robot types:
            - "scout": Moorebot Scout (mecanum wheels, indoor)
            - "scout_e": Moorebot Scout E (tracked, waterproof, outdoor)
            - "go2": Unitree Go2 (quadruped)
            - "g1": Unitree G1 (humanoid with arms)
            - "robbie": Robbie from Forbidden Planet (classic sci-fi robot)
            - "custom": Custom robot type (requires model_path)

            Args:
                operation: CRUD operation to perform:
                    - "create": Create/spawn a new virtual robot
                    - "read": Read/get details of an existing virtual robot
                    - "update": Update properties of an existing virtual robot
                    - "delete": Delete/remove a virtual robot
                    - "list": List all virtual robots (optionally filtered)
                robot_type: Type of robot (required for "create", optional for "list").
                    Must be one of: "scout", "scout_e", "go2", "g1", "robbie", "custom".
                robot_id: Virtual robot identifier (required for "read", "update", "delete").
                    Auto-generated for "create" if not provided.
                platform: Target platform ("unity" or "vrchat"). Default: "unity".
                position: Spawn/update position (x, y, z) for "create" or "update".
                scale: Size multiplier for "create" or "update" (e.g., 1.0 = original size).
                metadata: Additional metadata dictionary for "create" or "update".
                model_path: Path to 3D model file (.glb, .fbx, .vrm) for "create" with "custom" robot_type.

            Returns:
                Dictionary containing operation result with robot details.

            Examples:
                Create a Scout vbot:
                    result = await vbot_crud(
                        operation="create",
                        robot_type="scout",
                        platform="unity",
                        position={"x": 0.0, "y": 0.0, "z": 0.0},
                        scale=1.0
                    )

                Create Robbie from Forbidden Planet:
                    result = await vbot_crud(
                        operation="create",
                        robot_type="robbie",
                        platform="unity",
                        position={"x": 1.0, "y": 0.0, "z": 1.0},
                        scale=1.0
                    )

                Read vbot details:
                    result = await vbot_crud(
                        operation="read",
                        robot_id="vbot_scout_01"
                    )

                Update vbot scale and position:
                    result = await vbot_crud(
                        operation="update",
                        robot_id="vbot_scout_01",
                        scale=1.5,
                        position={"x": 2.0, "y": 0.0, "z": 2.0}
                    )

                Delete a vbot:
                    result = await vbot_crud(
                        operation="delete",
                        robot_id="vbot_scout_01"
                    )

                List all vbots:
                    result = await vbot_crud(operation="list")

                List only Scout vbots:
                    result = await vbot_crud(
                        operation="list",
                        robot_type="scout"
                    )
            """
            try:
                if operation == "create":
                    return await self._create_vbot(robot_type, robot_id, platform, position, scale, metadata, model_path)
                elif operation == "read":
                    return await self._read_vbot(robot_id)
                elif operation == "update":
                    return await self._update_vbot(robot_id, position, scale, metadata)
                elif operation == "delete":
                    return await self._delete_vbot(robot_id)
                elif operation == "list":
                    return await self._list_vbots(robot_type, platform)
                else:
                    return format_error_response(f"Unknown operation: {operation}", error_type="validation_error")
            except Exception as e:
                return handle_tool_error("vbot_crud", e, operation=operation, robot_type=robot_type, robot_id=robot_id)

    async def _create_vbot(
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
            return format_error_response("robot_type is required for create operation", error_type="validation_error")

        if robot_type not in SUPPORTED_ROBOT_TYPES:
            return format_error_response(
                f"Unsupported robot_type: {robot_type}. Supported types: {', '.join(SUPPORTED_ROBOT_TYPES)}",
                error_type="validation_error",
            )

        if robot_type == "custom" and not model_path:
            return format_error_response(
                "model_path is required for custom robot_type", error_type="validation_error"
            )

        # Generate robot_id if not provided
        if not robot_id:
            robot_id = f"vbot_{robot_type}_{len(self.state_manager.list_robots(is_virtual=True)) + 1:02d}"

        # Check if robot_id already exists
        if self.state_manager.get_robot(robot_id):
            return format_error_response(f"Robot {robot_id} already exists", error_type="validation_error")

        # Default position
        if position is None:
            position = {"x": 0.0, "y": 0.0, "z": 0.0}

        # Default scale
        if scale is None:
            scale = 1.0

        # Prepare metadata
        vbot_metadata = {
            "spawned": True,
            "platform": platform,
            "position": position,
            "scale": scale,
            "model_path": model_path,
            **(metadata or {}),
        }

        # Register robot in state manager
        try:
            robot = self.state_manager.register_robot(robot_id, robot_type, platform=platform, metadata=vbot_metadata)
        except ValueError as e:
            return format_error_response(str(e), error_type="validation_error")

        # Spawn in Unity/VRChat via mounted servers
        spawn_result = await self._spawn_in_platform(robot_id, robot_type, platform, position, scale, model_path)

        if spawn_result.get("status") != "success":
            # Cleanup registration if spawn failed
            self.state_manager.unregister_robot(robot_id)
            return spawn_result

        return format_success_response(
            f"Virtual robot {robot_id} created successfully",
            data={
                "robot_id": robot_id,
                "robot_type": robot_type,
                "platform": platform,
                "position": position,
                "scale": scale,
                "metadata": vbot_metadata,
            },
            robot_id=robot_id,
        )

    async def _read_vbot(self, robot_id: Optional[str]) -> Dict[str, Any]:
        """Read/get details of an existing virtual robot."""
        if not robot_id:
            return format_error_response("robot_id is required for read operation", error_type="validation_error")

        robot = self.state_manager.get_robot(robot_id)
        if not robot:
            return format_error_response(f"Virtual robot {robot_id} not found", error_type="not_found", robot_id=robot_id)

        if not robot.is_virtual:
            return format_error_response(f"Robot {robot_id} is not a virtual robot", error_type="validation_error")

        return format_success_response(
            f"Virtual robot {robot_id} details retrieved",
            data=robot.to_dict(),
            robot_id=robot_id,
        )

    async def _update_vbot(
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

        # Update metadata
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

        # Update in Unity/VRChat if position or scale changed
        if position is not None or scale is not None:
            update_result = await self._update_in_platform(robot_id, robot.platform, position, scale)
            if update_result.get("status") != "success":
                return update_result

        return format_success_response(
            f"Virtual robot {robot_id} updated successfully",
            data={"robot_id": robot_id, "updates": updates, "robot": robot.to_dict()},
            robot_id=robot_id,
        )

    async def _delete_vbot(self, robot_id: Optional[str]) -> Dict[str, Any]:
        """Delete/remove a virtual robot."""
        if not robot_id:
            return format_error_response("robot_id is required for delete operation", error_type="validation_error")

        robot = self.state_manager.get_robot(robot_id)
        if not robot:
            return format_error_response(f"Virtual robot {robot_id} not found", error_type="not_found", robot_id=robot_id)

        if not robot.is_virtual:
            return format_error_response(f"Robot {robot_id} is not a virtual robot", error_type="validation_error")

        # Remove from Unity/VRChat
        delete_result = await self._delete_from_platform(robot_id, robot.platform)
        if delete_result.get("status") != "success":
            logger.warning("Failed to delete from platform, but continuing with unregister", robot_id=robot_id)

        # Unregister from state manager
        self.state_manager.unregister_robot(robot_id)

        return format_success_response(
            f"Virtual robot {robot_id} deleted successfully",
            data={"robot_id": robot_id},
            robot_id=robot_id,
        )

    async def _list_vbots(self, robot_type: Optional[str], platform: Optional[str]) -> Dict[str, Any]:
        """List all virtual robots with optional filtering."""
        robots = self.state_manager.list_robots(is_virtual=True)

        # Filter by robot_type if provided
        if robot_type:
            robots = [r for r in robots if r.robot_type == robot_type]

        # Filter by platform if provided
        if platform:
            robots = [r for r in robots if r.platform == platform]

        return format_success_response(
            f"Found {len(robots)} virtual robot(s)",
            data={
                "count": len(robots),
                "robots": [r.to_dict() for r in robots],
                "filters": {"robot_type": robot_type, "platform": platform},
            },
        )

    async def _spawn_in_platform(
        self, robot_id: str, robot_type: str, platform: str, position: Dict[str, float], scale: float, model_path: Optional[str]
    ) -> Dict[str, Any]:
        """Spawn robot in Unity or VRChat.
        
        For Unity, this uses execute_unity_method to call a VbotSpawner script
        that instantiates the robot prefab in the scene.
        """
        try:
            if platform == "unity" and "unity" in self.mounted_servers:
                async with Client(self.mcp) as client:
                    # Use unity3d-mcp execute_unity_method to spawn robot
                    # This calls VbotSpawner.SpawnRobot() static method
                    # VbotSpawner.SpawnRobot(string robotId, string robotType, Vector3 position, float scale)
                    pos = position or {"x": 0.0, "y": 0.0, "z": 0.0}
                    scale_val = scale or 1.0
                    result = await client.call_tool(
                        "execute_unity_method",
                        {
                            "class_name": "VbotSpawner",
                            "method_name": "SpawnRobot",
                            "parameters": {
                                "robotId": robot_id,
                                "robotType": robot_type,
                                "position": {"x": pos.get("x", 0.0), "y": pos.get("y", 0.0), "z": pos.get("z", 0.0)},
                                "scale": scale_val,
                            },
                        },
                    )
                    return result
            elif platform == "vrchat" and "osc" in self.mounted_servers:
                # VRChat spawning via OSC
                async with Client(self.mcp) as client:
                    result = await client.call_tool(
                        "osc_send_osc",
                        {
                            "host": "127.0.0.1",
                            "port": 9000,
                            "address": f"/avatar/parameters/SpawnRobot",
                            "values": [robot_id, robot_type],
                        },
                    )
                    return result
            else:
                # Mock spawn for testing
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
                async with Client(self.mcp) as client:
                    # VbotSpawner.UpdateRobot(string robotId, Vector3? position, float? scale)
                    pos = None
                    if position:
                        pos = {"x": position.get("x", 0.0), "y": position.get("y", 0.0), "z": position.get("z", 0.0)}
                    result = await client.call_tool(
                        "execute_unity_method",
                        {
                            "class_name": "VbotSpawner",
                            "method_name": "UpdateRobot",
                            "parameters": {
                                "robotId": robot_id,
                                "position": pos,
                                "scale": scale,
                            },
                        },
                    )
                    return result
            else:
                # Mock update
                logger.info("Mock update", robot_id=robot_id, platform=platform)
                return format_success_response(f"Mock update: {robot_id}")

        except Exception as e:
            logger.error("Failed to update in platform", robot_id=robot_id, platform=platform, error=str(e))
            return format_error_response(f"Failed to update in {platform}: {str(e)}", error_type="connection_error")

    async def _delete_from_platform(self, robot_id: str, platform: str) -> Dict[str, Any]:
        """Delete robot from Unity or VRChat."""
        try:
            if platform == "unity" and "unity" in self.mounted_servers:
                async with Client(self.mcp) as client:
                    # VbotSpawner.DeleteRobot(string robotId)
                    result = await client.call_tool(
                        "execute_unity_method",
                        {
                            "class_name": "VbotSpawner",
                            "method_name": "DeleteRobot",
                            "parameters": {
                                "robotId": robot_id,
                            },
                        },
                    )
                    return result
            else:
                # Mock delete
                logger.info("Mock delete", robot_id=robot_id, platform=platform)
                return format_success_response(f"Mock delete: {robot_id}")

        except Exception as e:
            logger.error("Failed to delete from platform", robot_id=robot_id, platform=platform, error=str(e))
            return format_error_response(f"Failed to delete from {platform}: {str(e)}", error_type="connection_error")

