"""Virtual Robot CRUD tool - Create, Read, Update, Delete for virtual robots."""

import json
from typing import Any, Literal

import structlog
from fastmcp import Context

from ..utils.error_handler import (
    format_error_response,
    format_success_response,
    format_unavailable_error,
    handle_tool_error,
)
from ..utils.mcp_client_helper import call_mounted_server_tool

logger = structlog.get_logger(__name__)


def extract_result_data(result):
    """Extract data from CallToolResult object."""
    if isinstance(result, dict):
        return result

    # Try .data attribute first (most direct)
    if hasattr(result, "data") and result.data:
        return result.data

    # Try .content attribute
    if hasattr(result, "content") and result.content:
        if isinstance(result.content, list) and len(result.content) > 0:
            first_content = result.content[0]
            if hasattr(first_content, "text"):
                try:
                    return json.loads(first_content.text)
                except (json.JSONDecodeError, ValueError):
                    return {
                        "status": "error",
                        "error_type": "parse_error",
                        "message": f"Could not parse MCP tool response: {first_content.text}",
                    }
        elif hasattr(result.content, "text"):
            try:
                return json.loads(result.content.text)
            except (json.JSONDecodeError, ValueError):
                return {
                    "status": "error",
                    "error_type": "parse_error",
                    "message": f"Could not parse MCP tool response: {result.content.text}",
                }

    return {
        "status": "error",
        "error_type": "unknown_response",
        "message": "MCP tool returned an unrecognized response format",
        "details": {"raw": str(result)},
    }


# Supported robot types
SUPPORTED_ROBOT_TYPES = [
    "scout",  # Moorebot Scout
    "scout_e",  # Moorebot Scout E (tracked, waterproof)
    "go2",  # Unitree Go2
    "g1",  # Unitree G1
    "yahboom",  # Yahboom ROSMASTER series
    "nori_a3",  # Nori A3 bimanual home robot (placeholder primitive - no real 3D model yet)
    "mechazilla",  # Creative vBot (same holonomic OSC contract)
    "godzilla",  # Kaiju-scale creative vBot (spawn scale 50+)
    "drone",  # PX4/ArduPilot drones
    "px4_quad",  # PX4 quadcopter drone
    "ardupilot",  # ArduPilot-based drones
    "robbie",  # Robbie from Forbidden Planet
    "custom",  # Custom robot type
]


class VbotCrudTool:
    """CRUD operations for virtual robots (vbots)."""

    def __init__(
        self,
        mcp: Any,
        state_manager: Any,
        mounted_servers: dict[str, Any] | None = None,
        unity_available: bool = False,
    ):
        """Initialize vbot CRUD tool.

        Args:
            mcp: FastMCP server instance.
            state_manager: Robot state manager instance.
            mounted_servers: Dictionary of mounted MCP servers.
            unity_available: Flag indicating if Unity MCP server is available.
        """
        self.mcp = mcp
        self.state_manager = state_manager
        self.mounted_servers = mounted_servers or {}
        self.unity_available = unity_available

    def register(self):
        """Register vbot CRUD tool with MCP server."""

        @self.mcp.tool()
        async def vbot_crud(
            ctx: Context,
            operation: Literal["create", "read", "update", "delete", "list"],
            robot_type: str | None = None,
            robot_id: str | None = None,
            platform: Literal["unity", "vrchat", "resonite"] = "unity",
            position: dict[str, float] | None = None,
            scale: float | None = None,
            metadata: dict[str, Any] | None = None,
            model_path: str | None = None,
        ) -> dict[str, Any]:
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
            - "yahboom": Yahboom ROSMASTER series (AI-enabled wheeled robots)
            - "drone": Generic PX4/ArduPilot drones
            - "px4_quad": PX4 quadcopter drones
            - "ardupilot": ArduPilot-based drones
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
                    Must be one of: "scout", "scout_e", "go2", "g1", "yahboom", "drone", "px4_quad", "ardupilot", "robbie", "custom".
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

                Create Yahboom ROSMASTER robot:
                    result = await vbot_crud(
                        operation="create",
                        robot_type="yahboom",
                        platform="unity",
                        position={"x": 0.0, "y": 0.0, "z": 0.0},
                        scale=1.0
                    )

                Create PX4 quadcopter drone:
                    result = await vbot_crud(
                        operation="create",
                        robot_type="px4_quad",
                        platform="unity",
                        position={"x": 0.0, "y": 5.0, "z": 0.0},
                        scale=0.5
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

                List only Yahboom robots:
                    result = await vbot_crud(
                        operation="list",
                        robot_type="yahboom"
                    )

                List only drones:
                    result = await vbot_crud(
                        operation="list",
                        robot_type="drone"
                    )
            """
            try:
                if operation == "create":
                    return await self._create_vbot(
                        robot_type, robot_id, platform, position, scale, metadata, model_path
                    )
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
                return handle_tool_error(
                    "vbot_crud",
                    e,
                    action=operation,
                    context={"robot_type": robot_type, "robot_id": robot_id},
                )

    async def _create_vbot(
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
            return format_error_response("robot_type is required for create operation", error_type="validation_error")

        if robot_type not in SUPPORTED_ROBOT_TYPES:
            return format_error_response(
                f"Unsupported robot_type: {robot_type}. Supported types: {', '.join(SUPPORTED_ROBOT_TYPES)}",
                error_type="validation_error",
            )

        if robot_type == "custom" and not model_path:
            return format_error_response("model_path is required for custom robot_type", error_type="validation_error")

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
            self.state_manager.register_robot(robot_id, robot_type, platform=platform, metadata=vbot_metadata)
        except ValueError as e:
            return format_error_response(str(e), error_type="validation_error")

        # Spawn in Unity/VRChat via mounted servers
        spawn_result = await self._spawn_in_platform(robot_id, robot_type, platform, position, scale, model_path)

        spawn_data = extract_result_data(spawn_result)
        if spawn_data.get("status") != "success":
            # Cleanup registration if spawn failed
            self.state_manager.unregister_robot(robot_id)
            return spawn_result

        if spawn_data.get("resonite_slot_id"):
            robot = self.state_manager.get_robot(robot_id)
            if robot:
                robot.metadata["resonite_slot_id"] = spawn_data["resonite_slot_id"]

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

    async def _read_vbot(self, robot_id: str | None) -> dict[str, Any]:
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
            f"Virtual robot {robot_id} details retrieved",
            data=robot.to_dict(),
            robot_id=robot_id,
        )

    async def _update_vbot(
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

    async def _delete_vbot(self, robot_id: str | None) -> dict[str, Any]:
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

    async def _list_vbots(self, robot_type: str | None, platform: str | None) -> dict[str, Any]:
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
        self,
        robot_id: str,
        robot_type: str,
        platform: str,
        position: dict[str, float],
        scale: float,
        model_path: str | None,
    ) -> dict[str, Any]:
        """Spawn robot in Unity or VRChat.

        For Unity, this uses execute_unity_method to call a VbotSpawner script
        that instantiates the robot prefab in the scene.
        """
        try:
            if platform == "unity":
                if not self.unity_available:
                    logger.error(
                        "Unity MCP not available for spawn",
                        robot_id=robot_id,
                        platform=platform,
                    )
                    return format_unavailable_error(
                        "Unity MCP",
                        "spawn robot",
                        robot_id=robot_id,
                        platform=platform,
                    )

                # Unity is available - attempt real spawn with timeout protection
                try:
                    import asyncio

                    pos = position or {"x": 0.0, "y": 0.0, "z": 0.0}
                    scale_val = scale or 1.0

                    # No real 3D model exists for most vbot robot_types - spawn a labeled
                    # primitive placeholder via unity_bridge(operation="create_object")
                    # (MCPBridge.cs CreateObject). "execute_unity_method" -> "VbotSpawner"
                    # never existed as a real Unity class/method.
                    result = await asyncio.wait_for(
                        call_mounted_server_tool(
                            self.mounted_servers,
                            "unity",
                            "unity_bridge",
                            {
                                "operation": "create_object",
                                "name": robot_id,
                                "object_type": "Capsule",
                                "position": [pos.get("x", 0.0), pos.get("y", 0.0), pos.get("z", 0.0)],
                                "scale": scale_val,
                            },
                        ),
                        timeout=10.0,  # 10 second timeout for Unity operations
                    )
                    # MCPBridge.cs CreateObject returns {"status": "created", ...} - normalize
                    # to the CRUD layer's "success" convention (_create_vbot checks literally).
                    data = extract_result_data(result)
                    if data.get("status") == "created":
                        data["status"] = "success"
                    return data

                except TimeoutError:
                    logger.error("Unity spawn operation timed out", robot_id=robot_id, timeout=10.0)
                    return format_error_response(
                        "Unity spawn operation timed out",
                        error_type="timeout",
                        robot_id=robot_id,
                        timeout_seconds=10.0,
                    )
                except Exception as e:
                    logger.error("Unity spawn failed", robot_id=robot_id, error=str(e))
                    return format_error_response(
                        f"Unity spawn failed: {e!s}",
                        error_type="unity_error",
                        robot_id=robot_id,
                        details={"platform": platform},
                    )

            elif platform == "vrchat" and "osc" in self.mounted_servers:
                # VRChat spawning via OSC (mounted server pattern)
                result = await call_mounted_server_tool(
                    self.mounted_servers,
                    "osc",
                    "osc_send_osc",
                    {
                        "host": "127.0.0.1",
                        "port": 9000,
                        "address": "/avatar/parameters/SpawnRobot",
                        "values": [robot_id, robot_type],
                    },
                )
                return result
            elif platform == "resonite":
                return await self._spawn_in_resonite(robot_id, position, scale_val, robot_type)
            else:
                return format_unavailable_error(
                    f"{platform} integration",
                    "spawn robot",
                    robot_id=robot_id,
                    platform=platform,
                )

        except Exception as e:
            logger.error("Failed to spawn in platform", robot_id=robot_id, platform=platform, error=str(e))
            return format_error_response(f"Failed to spawn in {platform}: {e!s}", error_type="connection_error")

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

        For "nori_a3" this is the REAL, correctly-posed A3 geometry - see
        norirobotics-mcp/scripts/export_posed_mesh.py. Every other robot_type gets a
        procedural placeholder box.
        """
        from ..utils.mesh_primitives import box_mesh, load_nori_a3_mesh

        preflight = await self._ensure_resonite_connected()
        if preflight:
            return preflight

        pos = position or {"x": 0.0, "y": 0.0, "z": 0.0}
        real = load_nori_a3_mesh() if robot_type == "nori_a3" else None
        if real is not None:
            vertices, submeshes = real
            color = {"r": 0.88, "g": 0.88, "b": 0.86, "a": 1.0}
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
        """Update robot in Unity or VRChat."""
        try:
            if platform == "unity":
                if not self.unity_available:
                    logger.error("Unity MCP not available for update", robot_id=robot_id, platform=platform)
                    return format_unavailable_error(
                        "Unity MCP",
                        "update robot",
                        robot_id=robot_id,
                        platform=platform,
                    )

                # Unity available - attempt real update with timeout
                try:
                    import asyncio

                    pos = None
                    if position:
                        pos = {
                            "x": position.get("x", 0.0),
                            "y": position.get("y", 0.0),
                            "z": position.get("z", 0.0),
                        }

                    params: dict[str, Any] = {"operation": "transform_object", "target": robot_id}
                    if pos:
                        params["position"] = [pos["x"], pos["y"], pos["z"]]
                    if scale is not None:
                        params["scale"] = scale
                    result = await asyncio.wait_for(
                        call_mounted_server_tool(self.mounted_servers, "unity", "unity_bridge", params),
                        timeout=5.0,  # Shorter timeout for updates
                    )
                    return result

                except TimeoutError:
                    logger.error("Unity update timed out", robot_id=robot_id, timeout=5.0)
                    return format_error_response(
                        f"Unity update timed out after 5s for {robot_id}",
                        error_type="timeout",
                        robot_id=robot_id,
                    )
                except Exception as e:
                    logger.error("Unity update failed", robot_id=robot_id, error=str(e))
                    return format_error_response(
                        f"Unity update failed: {e!s}",
                        error_type="unity_error",
                        robot_id=robot_id,
                    )

            elif platform == "vrchat" and "osc" in self.mounted_servers:
                # VRChat update via OSC (mounted server pattern)
                try:
                    result = await asyncio.wait_for(
                        call_mounted_server_tool(
                            self.mounted_servers,
                            "osc",
                            "osc_send_osc",
                            {
                                "host": "127.0.0.1",
                                "port": 9000,
                                "address": "/avatar/parameters/UpdateRobot",
                                "values": [robot_id, position or {}, scale or 1.0],
                            },
                        ),
                        timeout=2.0,
                    )
                    return result
                except TimeoutError:
                    return format_error_response("VRChat update timeout", error_type="timeout")
                except Exception as e:
                    logger.error("VRChat update failed", robot_id=robot_id, error=str(e))
                    return format_error_response(f"VRChat update failed: {e!s}", error_type="osc_error")

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
            if platform == "unity":
                if not self.unity_available:
                    logger.error("Unity MCP not available for delete", robot_id=robot_id, platform=platform)
                    return format_unavailable_error(
                        "Unity MCP",
                        "delete robot",
                        robot_id=robot_id,
                        platform=platform,
                    )

                # Unity available - attempt real delete with timeout
                try:
                    import asyncio

                    result = await asyncio.wait_for(
                        call_mounted_server_tool(
                            self.mounted_servers,
                            "unity",
                            "unity_bridge",
                            {"operation": "delete_object", "target": robot_id},
                        ),
                        timeout=5.0,  # Shorter timeout for deletes
                    )
                    data = extract_result_data(result)
                    if data.get("status") == "deleted":
                        data["status"] = "success"
                    return data

                except TimeoutError:
                    logger.error("Unity delete timed out", robot_id=robot_id, timeout=5.0)
                    return format_error_response(
                        f"Unity delete timed out after 5s for {robot_id}",
                        error_type="timeout",
                        robot_id=robot_id,
                    )
                except Exception as e:
                    logger.error("Unity delete failed", robot_id=robot_id, error=str(e))
                    return format_error_response(
                        f"Unity delete failed: {e!s}",
                        error_type="unity_error",
                        robot_id=robot_id,
                    )

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
                return extract_result_data(
                    await call_mounted_server_tool(
                        self.mounted_servers, "resonite", "resonite_link_destroy_slot", {"slot_id": slot_id}
                    )
                )

            elif platform == "vrchat" and "osc" in self.mounted_servers:
                # VRChat delete via OSC (mounted server pattern)
                try:
                    result = await asyncio.wait_for(
                        call_mounted_server_tool(
                            self.mounted_servers,
                            "osc",
                            "osc_send_osc",
                            {
                                "host": "127.0.0.1",
                                "port": 9000,
                                "address": "/avatar/parameters/DeleteRobot",
                                "values": [robot_id],
                            },
                        ),
                        timeout=2.0,
                    )
                    return result
                except TimeoutError:
                    return format_error_response("VRChat delete timeout", error_type="timeout")
                except Exception as e:
                    logger.error("VRChat delete failed", robot_id=robot_id, error=str(e))
                    return format_error_response(f"VRChat delete failed: {e!s}", error_type="osc_error")

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
