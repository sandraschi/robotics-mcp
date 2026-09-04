"""Robot virtual portmanteau tool - Virtual robot lifecycle and operations.

Consolidates virtual robot CRUD operations and virtual robotics operations into a single unified tool.
"""

import asyncio
from pathlib import Path
from typing import Any, Literal

import structlog
from fastmcp import Context

from ..osc_bridge import osc_bridge
from ..utils.error_handler import (
    format_error_response,
    format_not_implemented_error,
    format_success_response,
    format_unavailable_error,
    handle_tool_error,
)
from ..utils.mcp_client_helper import call_mounted_server_tool
from ..utils.response_builders import (
    build_robotics_error_response,
)
from .vbot_crud import extract_result_data

logger = structlog.get_logger(__name__)

SUPPORTED_ROBOT_TYPES = [
    "scout",
    "scout_e",
    "go2",
    "g1",
    "robbie",
    "custom",
    "yahboom",
    "nori_a3",  # Nori A3 bimanual home robot (placeholder primitive - no real 3D model yet)
    "mechazilla",
    "godzilla",
    "dreame",
]


class RobotVirtualTool:
    """Portmanteau tool for virtual robot lifecycle and operations."""

    def __init__(self, mcp: Any, state_manager: Any, mounted_servers: dict[str, Any] | None = None):
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
            ctx: Context,
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
            robot_type: str | None = None,
            robot_id: str | None = None,
            platform: Literal["unity", "vrchat", "resonite"] = "unity",
            position: dict[str, float] | None = None,
            scale: float | None = None,
            metadata: dict[str, Any] | None = None,
            model_path: str | None = None,
            environment: str | None = None,
            environment_path: str | None = None,
            project_path: str | None = None,
            include_colliders: bool = True,
        ) -> dict[str, Any]:
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
                platform: Target platform ("unity", "vrchat", "resonite"). Default: "unity".
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
            return await self.handle_operations(
                operation,
                robot_type,
                robot_id,
                platform,
                position,
                scale,
                metadata,
                model_path,
                environment,
                environment_path,
                project_path,
                include_colliders,
            )

    async def handle_operations(
        self,
        operation: str,
        robot_type: str | None = None,
        robot_id: str | None = None,
        platform: str = "unity",
        position: dict[str, float] | None = None,
        scale: float | None = None,
        metadata: dict[str, Any] | None = None,
        model_path: str | None = None,
        environment: str | None = None,
        environment_path: str | None = None,
        project_path: str | None = None,
        include_colliders: bool = True,
    ) -> dict[str, Any]:
        """Handle virtual robot operations (exposed for testing)."""
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
                return await self._handle_load_environment(
                    environment, platform, environment_path, project_path, include_colliders
                )
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
            logger.error(
                f"Error in robot virtual {operation}",
                robot_id=robot_id,
                robot_type=robot_type,
                error=str(e),
                exc_info=True,
            )

            # Intelligent error analysis for virtual robot issues
            error_str = str(e).lower()
            recovery_options = []

            if operation in ["create", "spawn"] and ("unity" in error_str or "vrchat" in error_str):
                recovery_options = [
                    f"Verify {platform} MCP server is running and mounted",
                    f"Check {platform} project is open and accessible",
                    f"Ensure {platform} VbotSpawner script is loaded",
                    f"Verify {platform} network connectivity and firewall settings",
                ]
            elif operation == "load_environment" and ("unity" in error_str or "environment" in error_str):
                recovery_options = [
                    f"Check {platform} project contains the environment '{environment}'",
                    f"Verify environment file path is correct: {environment_path}",
                    f"Ensure {platform} project has proper permissions",
                    f"Check {platform} Unity version compatibility",
                ]
            elif operation == "get_lidar" and ("physics" in error_str or "raycast" in error_str):
                recovery_options = [
                    f"Verify {robot_id} is spawned in {platform} scene",
                    f"Check {platform} physics engine is enabled",
                    f"Ensure {robot_id} LiDAR sensor is properly configured",
                    f"Verify {platform} scene has collision objects",
                ]
            elif "connection" in error_str or "network" in error_str or "timeout" in error_str:
                recovery_options = [
                    f"Check {platform} MCP server connectivity",
                    f"Verify {platform} project is running and accessible",
                    f"Ensure firewall allows communication with {platform}",
                    f"Try restarting the {platform} MCP server",
                ]
            elif "file" in error_str or "path" in error_str:
                recovery_options = [
                    "Verify model_path points to a valid 3D model file (.fbx, .glb, .vrm)",
                    "Check file permissions and accessibility",
                    "Ensure model file is not corrupted",
                    "Verify supported model format for the platform",
                ]
            else:
                recovery_options = [
                    f"Check {platform} MCP server status and connectivity",
                    f"Verify {robot_id or 'robot'} configuration and parameters",
                    f"Try the {operation} operation again",
                    "Check Robotics MCP server logs for detailed error information",
                ]

            robot_info = f" for {robot_type}" if robot_type else ""
            return build_robotics_error_response(
                error=f"Virtual robot operation failed: {operation}{robot_info}. Error: {e!s}",
                robot_type=robot_type or "virtual_robot",
                robot_id=robot_id or "unknown",
                recovery_options=recovery_options,
                suggestions=[
                    f"Try the virtual robot {operation} operation again after applying recovery steps",
                    f"Check {platform} MCP server status with 'robotics_system status'",
                    f"Verify {platform} project configuration and connectivity",
                ],
            )

    # CRUD handlers (from vbot_crud.py)
    async def _handle_create(
        self,
        robot_type: str | None,
        robot_id: str | None,
        platform: str,
        position: dict[str, float] | None,
        scale: float | None,
        metadata: dict[str, Any] | None,
        model_path: str | None,
    ) -> dict[str, Any]:
        """Create/spawn a new virtual robot."""
        if not robot_type:
            return format_error_response(
                "robot_type is required for create/spawn operation", error_type="validation_error"
            )
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
            self.state_manager.register_robot(robot_id, robot_type, platform=platform, metadata=vbot_metadata)
        except ValueError as e:
            return format_error_response(str(e), error_type="validation_error")
        spawn_result = await self._spawn_in_platform(robot_id, robot_type, platform, position, scale, model_path)
        if spawn_result.get("status") != "success":
            self.state_manager.unregister_robot(robot_id)
            return spawn_result

        if spawn_result.get("resonite_slot_id"):
            robot = self.state_manager.get_robot(robot_id)
            if robot:
                robot.metadata["resonite_slot_id"] = spawn_result["resonite_slot_id"]

        return format_success_response(
            f"Virtual robot {robot_id} created successfully",
            data={
                "robot_id": robot_id,
                "robot_type": robot_type,
                "platform": platform,
                "position": position,
                "scale": scale,
            },
            robot_id=robot_id,
        )

    async def _handle_read(self, robot_id: str | None) -> dict[str, Any]:
        """Read/get details of an existing virtual robot."""
        if not robot_id:
            return format_error_response("robot_id is required for read operation", error_type="validation_error")
        robot = self.state_manager.get_robot(robot_id)
        if not robot:
            return format_error_response(
                f"Virtual robot {robot_id} not found", error_type="not_found", robot_id=robot_id
            )
        if not robot.is_virtual:
            return format_error_response(f"Robot {robot_id} is not a virtual robot", error_type="validation_error")
        return format_success_response(
            f"Virtual robot {robot_id} details retrieved", data=robot.to_dict(), robot_id=robot_id
        )

    async def _handle_update(
        self,
        robot_id: str | None,
        position: dict[str, float] | None,
        scale: float | None,
        metadata: dict[str, Any] | None,
    ) -> dict[str, Any]:
        """Update properties of an existing virtual robot."""
        if not robot_id:
            return format_error_response("robot_id is required for update operation", error_type="validation_error")
        robot = self.state_manager.get_robot(robot_id)
        if not robot:
            return format_error_response(
                f"Virtual robot {robot_id} not found", error_type="not_found", robot_id=robot_id
            )
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

    async def _handle_delete(self, robot_id: str | None) -> dict[str, Any]:
        """Delete/remove a virtual robot."""
        if not robot_id:
            return format_error_response("robot_id is required for delete operation", error_type="validation_error")
        robot = self.state_manager.get_robot(robot_id)
        if not robot:
            return format_error_response(
                f"Virtual robot {robot_id} not found", error_type="not_found", robot_id=robot_id
            )
        if not robot.is_virtual:
            return format_error_response(f"Robot {robot_id} is not a virtual robot", error_type="validation_error")
        delete_result = await self._delete_from_platform(robot_id, robot.platform)
        if delete_result.get("status") != "success":
            logger.warning("Failed to delete from platform, but continuing with unregister", robot_id=robot_id)
        self.state_manager.unregister_robot(robot_id)
        return format_success_response(
            f"Virtual robot {robot_id} deleted successfully",
            data={"robot_id": robot_id},
            robot_id=robot_id,
        )

    async def _handle_list(self, robot_type: str | None, platform: str | None) -> dict[str, Any]:
        """List all virtual robots with optional filtering."""
        robots = self.state_manager.list_robots(is_virtual=True)
        if robot_type:
            robots = [r for r in robots if r.robot_type == robot_type]
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

    # Virtual robot operation handlers (from virtual_robotics.py)
    async def _handle_load_environment(
        self,
        environment: str,
        platform: str,
        environment_path: str | None,
        project_path: str | None,
        include_colliders: bool,
    ) -> dict[str, Any]:
        """Load Marble/Chisel environment."""
        logger.info(
            "Loading environment",
            environment=environment,
            platform=platform,
            environment_path=environment_path,
        )
        try:
            if platform == "unity" and "unity" in self.mounted_servers:
                source_path = environment_path or environment
                if source_path and source_path.lower().endswith(".spz"):
                    return format_error_response(
                        ".spz files are not supported by Unity. Use robot_model(operation='spz_convert') or re-export from Marble as .ply/.fbx/.glb.",
                        error_type="unsupported_format",
                    )
                result = await call_mounted_server_tool(
                    self.mounted_servers,
                    "unity",
                    "import_marble_world",
                    {
                        "source_path": source_path,
                        "project_path": project_path or "",
                        "include_colliders": include_colliders,
                    },
                )
                return format_success_response(
                    f"Environment {environment} loaded via Unity",
                    action="load_environment",
                    data={"environment": environment, "platform": platform, "unity_result": result},
                )
            else:
                return format_unavailable_error(
                    "Unity MCP",
                    "load environment",
                    platform=platform,
                    details={"environment": environment},
                )
        except Exception as e:
            return handle_tool_error(
                "_handle_load_environment",
                e,
                context={"environment": environment, "platform": platform},
            )

    async def _handle_get_status(self, robot_id: str | None) -> dict[str, Any]:
        """Get virtual robot status."""
        if not robot_id:
            return format_error_response("robot_id required", error_type="validation_error")
        robot = self.state_manager.get_robot(robot_id)
        if not robot:
            return format_error_response(f"Robot {robot_id} not found", error_type="not_found", robot_id=robot_id)
        return format_success_response(
            f"Robot {robot_id} status retrieved", robot_id=robot_id, data={"robot": robot.to_dict()}
        )

    async def _handle_get_lidar(self, robot_id: str | None) -> dict[str, Any]:
        """Get virtual LiDAR scan."""
        if not robot_id:
            return format_error_response("robot_id required", error_type="validation_error")
        robot = self.state_manager.get_robot(robot_id)
        if not robot or not robot.is_virtual:
            return format_error_response(
                f"Virtual robot {robot_id} not found", error_type="not_found", robot_id=robot_id
            )
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
            return format_unavailable_error(
                "Unity MCP",
                "perform LiDAR scan",
                robot_id=robot_id,
                platform=robot.platform,
            )

    async def _handle_set_scale(self, robot_id: str | None, scale: float | None) -> dict[str, Any]:
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

    async def _handle_test_navigation(self, robot_id: str | None, environment: str | None) -> dict[str, Any]:
        """Test navigation in environment."""
        return format_not_implemented_error(
            "Virtual robot navigation testing",
            robot_id=robot_id,
            details={"environment": environment},
        )

    async def _handle_sync_with_physical(self, robot_id: str | None) -> dict[str, Any]:
        """Sync virtual robot with physical robot state."""
        return format_not_implemented_error(
            "Virtual/physical robot sync",
            robot_id=robot_id,
        )

    # Platform interaction helpers
    async def _spawn_in_platform(
        self,
        robot_id: str,
        robot_type: str,
        platform: str,
        position: dict[str, float],
        scale: float,
        model_path: str | None,
    ) -> dict[str, Any]:
        """Spawn robot in Unity or VRChat."""
        try:
            if platform == "unity" and "unity" in self.mounted_servers:
                # No real 3D model exists for most vbot robot_types - spawn a labeled
                # primitive placeholder via the real unity_bridge(operation="create_object")
                # action (MCPBridge.cs CreateObject). The formerly-called "execute_unity_method"
                # -> "VbotSpawner.SpawnRobot" path never existed as a real Unity class/method.
                pos = position or {"x": 0.0, "y": 0.0, "z": 0.0}
                result = await call_mounted_server_tool(
                    self.mounted_servers,
                    "unity",
                    "unity_bridge",
                    {
                        "operation": "create_object",
                        "name": robot_id,
                        "object_type": "Capsule",
                        "position": [pos.get("x", 0.0), pos.get("y", 0.0), pos.get("z", 0.0)],
                        "scale": scale,
                    },
                )
                # MCPBridge.cs CreateObject returns {"status": "created", ...} - normalize to
                # the CRUD layer's "success" convention (_handle_create checks for it literally).
                data = extract_result_data(result)
                if data.get("status") == "created":
                    data["status"] = "success"
                if robot_type == "nori_a3":
                    data["real_model"] = await self._stage_nori_a3_glb_in_unity()
                return data
            elif platform == "resonite":
                return await self._spawn_in_resonite(robot_id, position, scale, robot_type)
            else:
                return format_unavailable_error(
                    f"{platform} integration",
                    "spawn robot",
                    robot_id=robot_id,
                    platform=platform,
                )
        except Exception as e:
            logger.error("Failed to spawn in platform", robot_id=robot_id, platform=platform, error=str(e))
            return format_error_response(f"Failed to update in {platform}: {e!s}", error_type="connection_error")

    async def _stage_nori_a3_glb_in_unity(self) -> dict[str, Any]:
        """Best-effort: stage the real, correctly-posed A3 GLB into the mounted Unity project's
        Assets folder via unity3d-mcp's real import_3d_model (verified: it genuinely copies the
        file, not a stub - see import_export_manager.py:123-133).

        This does NOT instantiate a live GameObject from the imported asset - unity3d-mcp has
        no "instantiate from imported prefab" tool yet, only asset-import and primitive
        create_object. The capsule spawned above is what's actually visible in the running
        scene; this just makes the real model available in the project for a manual drag-in
        (or a future tool) instead of only ever having a capsule. Never fails the overall spawn.
        """
        glb_path = (
            Path(__file__).resolve().parent.parent.parent.parent.parent
            / "norirobotics-mcp"
            / "models"
            / "nori_description"
            / "nori_a3_posed.glb"
        )
        if not glb_path.is_file():
            return {
                "staged": False,
                "reason": "nori_a3_posed.glb not found - run norirobotics-mcp/scripts/export_posed_mesh.py",
            }
        try:
            result = extract_result_data(
                await call_mounted_server_tool(
                    self.mounted_servers, "unity", "import_3d_model", {"model_path": str(glb_path)}
                )
            )
            return {"staged": bool(result.get("success")), "detail": result}
        except Exception as e:
            return {"staged": False, "reason": str(e)}

    async def _ensure_resonite_connected(self) -> dict[str, Any] | None:
        """Discover + connect to a live ResoniteLink session. Returns an error dict on failure, else None.

        Real protocol (Yellow-Dog-Man/ResoniteLink), not the dead OSC vbot_osc_receiver path -
        proven working by resonite-mcp's scripts/spawn_nekomimi_in_home.py (a VRM spawned live
        into Sandra's Resonite Home via this exact discover -> connect -> spawn_mesh sequence).
        """
        if "resonite" not in self.mounted_servers:
            return format_unavailable_error("Resonite MCP", "connect", details={"hint": "resonite-mcp not mounted"})

        discover = extract_result_data(
            await call_mounted_server_tool(
                self.mounted_servers, "resonite", "resonite_link_discover", {"timeout_seconds": 8.0}
            )
        )
        sessions = discover.get("sessions")
        if not sessions:
            return format_error_response(
                "No ResoniteLink sessions found on the LAN. Is Resonite running with ResoniteLink "
                "enabled (Dashboard -> Session -> Settings -> Enable ResoniteLink)?",
                error_type="not_available",
            )
        session = sessions[0]
        connect = extract_result_data(
            await call_mounted_server_tool(
                self.mounted_servers,
                "resonite",
                "resonite_link_connect",
                {"input_data": {"host": session.get("host", "localhost"), "port": session["linkPort"]}},
            )
        )
        if connect.get("status") != "success":
            return format_error_response(
                f"Failed to connect to ResoniteLink session {session.get('sessionName')!r}",
                error_type="connection_error",
                details={"connect_result": connect},
            )
        return None

    async def _spawn_in_resonite(
        self, robot_id: str, position: dict[str, float] | None, scale: float | None, robot_type: str | None = None
    ) -> dict[str, Any]:
        """Spawn a robot mesh in a live Resonite world via ResoniteLink.

        For "nori_a3" this is the REAL, correctly-posed A3 geometry (26k tris, decimated from
        norirobotics-mcp's vendored nori_description meshes - see
        norirobotics-mcp/scripts/export_posed_mesh.py). Every other robot_type still gets a
        procedural placeholder box, since no real model exists for them. Both paths use the
        same discover -> connect -> spawn_mesh pipeline already proven for the nekomimi-chan
        VRM spawn.
        """
        from ..utils.mesh_primitives import box_mesh, load_nori_a3_mesh

        preflight = await self._ensure_resonite_connected()
        if preflight:
            return preflight

        pos = position or {"x": 0.0, "y": 0.0, "z": 0.0}
        real = load_nori_a3_mesh() if robot_type == "nori_a3" else None
        if real is not None:
            vertices, submeshes = real
            color = {"r": 0.88, "g": 0.88, "b": 0.86, "a": 1.0}  # off-white, matches the real A3's body color
        else:
            vertices, submeshes = box_mesh(size=(scale or 1.0) * 0.3)
            color = {"r": 0.2, "g": 0.6, "b": 0.9, "a": 1.0}
        result = extract_result_data(
            await call_mounted_server_tool(
                self.mounted_servers,
                "resonite",
                "resonite_link_spawn_mesh",
                {
                    "vertices": vertices,
                    "submeshes": submeshes,
                    "name": robot_id,
                    "pos_x": pos.get("x", 0.0),
                    "pos_y": pos.get("y", 0.0),
                    "pos_z": pos.get("z", 0.0),
                    "color_r": color["r"],
                    "color_g": color["g"],
                    "color_b": color["b"],
                    "color_a": color["a"],
                },
            )
        )
        if result.get("status") != "success":
            return format_error_response(
                "ResoniteLink spawn_mesh failed", error_type="spawn_error", details={"result": result}
            )
        return format_success_response(
            f"Spawned {robot_id} in Resonite via ResoniteLink",
            data={"robot_id": robot_id, "platform": "resonite", "resonite_slot_id": result.get("slot_id")},
        )

    async def _update_in_platform(
        self, robot_id: str, platform: str, position: dict[str, float] | None, scale: float | None
    ) -> dict[str, Any]:
        """Update robot in Unity, VRChat, or Resonite."""
        try:
            # Unity Platform Handler
            if platform == "unity" and "unity" in self.mounted_servers:
                params: dict[str, Any] = {"operation": "transform_object", "target": robot_id}
                if position:
                    params["position"] = [position.get("x", 0.0), position.get("y", 0.0), position.get("z", 0.0)]
                if scale is not None:
                    params["scale"] = scale
                result = await call_mounted_server_tool(self.mounted_servers, "unity", "unity_bridge", params)
                return extract_result_data(result)

            elif platform == "resonite":
                # ResoniteLink has no generic "move slot" tool wired here yet (spawn_mesh
                # doesn't return a Transform component id to write to) - respawn to move,
                # rather than silently no-op via the dead OSC path.
                return format_not_implemented_error(
                    "Resonite vbot move/update — respawn (delete + create) to reposition for now",
                    robot_id=robot_id,
                )

            # VRChat Handler (OSC) — see docs VBOT_SPAWN_MANUAL_STEPS.md: VRChat OSC has no
            # real spawn/move primitive until a custom world+avatar is built and uploaded.
            elif platform == "vrchat":
                robot = self.state_manager.get_robot(robot_id)
                robot_type = robot.robot_type if robot else "unknown"

                tasks = []
                if position:
                    # Flatten position for OSC (simple representation)
                    # In reality, might need separate x/y/z parameters
                    tasks.append(osc_bridge.send_command(platform, robot_type, "position_x", position.get("x", 0)))
                    tasks.append(osc_bridge.send_command(platform, robot_type, "position_y", position.get("y", 0)))
                    tasks.append(osc_bridge.send_command(platform, robot_type, "position_z", position.get("z", 0)))

                if scale:
                    tasks.append(osc_bridge.send_command(platform, robot_type, "scale", scale))

                await asyncio.gather(*tasks)
                return format_success_response(f"Updated {robot_id} in {platform} via OSC")

            else:
                return format_unavailable_error(
                    f"{platform} integration",
                    "update robot",
                    robot_id=robot_id,
                    platform=platform,
                )

        except Exception as e:
            logger.error("Failed to update in platform", robot_id=robot_id, platform=platform, error=str(e))
            return format_error_response(f"Failed to update in {platform}: {e!s}", error_type="connection_error")

    async def _delete_from_platform(self, robot_id: str, platform: str) -> dict[str, Any]:
        """Delete robot from Unity or VRChat."""
        try:
            if platform == "unity" and "unity" in self.mounted_servers:
                result = await call_mounted_server_tool(
                    self.mounted_servers,
                    "unity",
                    "unity_bridge",
                    {"operation": "delete_object", "target": robot_id},
                )
                data = extract_result_data(result)
                if data.get("status") == "deleted":
                    data["status"] = "success"
                return data
            elif platform == "resonite" and "resonite" in self.mounted_servers:
                robot = self.state_manager.get_robot(robot_id)
                slot_id = (robot.metadata or {}).get("resonite_slot_id") if robot else None
                if not slot_id:
                    return format_error_response(
                        f"No resonite_slot_id recorded for {robot_id} — cannot delete",
                        error_type="not_found",
                        robot_id=robot_id,
                    )
                preflight = await self._ensure_resonite_connected()
                if preflight:
                    return preflight
                result = extract_result_data(
                    await call_mounted_server_tool(
                        self.mounted_servers,
                        "resonite",
                        "resonite_link_destroy_slot",
                        {"slot_id": slot_id},
                    )
                )
                return result
            else:
                return format_unavailable_error(
                    f"{platform} integration",
                    "delete robot",
                    robot_id=robot_id,
                    platform=platform,
                )
        except Exception as e:
            logger.error("Failed to delete from platform", robot_id=robot_id, platform=platform, error=str(e))
            return format_error_response(f"Failed to delete from {platform}: {e!s}", error_type="connection_error")
