"""Gazebo Fuel model management tool.

Portmanteau MCP tool for browsing, downloading, and spawning models
from the Gazebo Fuel library (3000+ free simulation assets).

Operations:
- search: Search/browse Gazebo Fuel models
- download: Download a model to local Gazebo model path
- spawn: Download + spawn a model into running Gazebo simulation
- list_local: List locally downloaded models
- delete_local: Remove a locally cached model
"""

import time
from typing import Any, Literal

import structlog
from fastmcp import Context

from ..services import gazebo_fuel_service

logger = structlog.get_logger(__name__)


class GazeboModelsTool:
    """Gazebo Fuel model browser and spawner."""

    def __init__(
        self,
        mcp: Any,
        state_manager: Any,
        mounted_servers: dict[str, Any] | None = None,
    ):
        self.mcp = mcp
        self.state_manager = state_manager
        self.mounted = mounted_servers or {}

    def register(self) -> None:
        """Register gazebo_models portmanteau tool."""

        @self.mcp.tool()
        async def gazebo_models(
            ctx: Context,
            operation: Literal["search", "download", "spawn", "list_local", "delete_local"],
            query: str = "",
            owner: str = "",
            model_name: str = "",
            category: str = "",
            page: int = 1,
            per_page: int = 20,
            spawn_name: str = "",
            x: float = 0.0,
            y: float = 0.0,
            z: float = 0.5,
        ) -> dict[str, Any]:
            """Manage Gazebo Fuel simulation models - browse, download, and spawn.

            Browse, download, and spawn 3000+ free models from Gazebo Fuel
            (fuel.gazebosim.org) into your Gazebo simulation environment.

            Operations:
                search: Search the Fuel model library.
                    - query: Search text (e.g., "turtlebot", "drone", "warehouse")
                    - owner: Filter by model owner (e.g., "OpenRobotics")
                    - category: Filter by category ("Robots", "Vehicles", "Buildings",
                      "Furniture", "Electronics", "Animals", "Warehouse")
                    - page, per_page: Pagination (max 100 per page)

                download: Download a model to local Gazebo model path.
                    - owner: Model owner (required)
                    - model_name: Model name (required)
                    Downloads to ~/.gz/fuel/fuel.gazebosim.org/<owner>/models/<name>/

                spawn: Download (if needed) and spawn into running Gazebo simulation.
                    - owner, model_name: Model to spawn (required)
                    - spawn_name: Instance name (auto-generated if empty)
                    - x, y, z: Spawn position in world frame (meters)
                    Requires Gazebo + rosbridge running.

                list_local: List all locally downloaded models.
                    No parameters needed.

                delete_local: Remove a locally cached model.
                    - owner, model_name: Model to delete (required)

            Args:
                ctx: MCP context
                operation: Operation to perform
                query: Search text
                owner: Model owner
                model_name: Model name
                category: Category filter
                page: Page number
                per_page: Results per page
                spawn_name: Name for spawned instance
                x: Spawn X position (meters)
                y: Spawn Y position (meters)
                z: Spawn Z position (meters)

            Returns:
                Operation-specific result dict.

            Examples:
                Search for robot models:
                    gazebo_models(operation="search", query="turtlebot")

                Browse OpenRobotics collection:
                    gazebo_models(operation="search", owner="OpenRobotics", category="Robots")

                Download a model:
                    gazebo_models(operation="download", owner="OpenRobotics", model_name="TurtleBot")

                Spawn into simulation:
                    gazebo_models(operation="spawn", owner="OpenRobotics",
                                  model_name="TurtleBot", x=1.0, y=0.0, z=0.1)
            """
            await ctx.info(f"gazebo_models: {operation}")

            try:
                if operation == "search":
                    return await gazebo_fuel_service.search_models(
                        query=query,
                        owner=owner,
                        category=category,
                        page=page,
                        per_page=per_page,
                    )

                elif operation == "download":
                    if not owner or not model_name:
                        return {"error": "owner and model_name are required for download"}
                    result = await gazebo_fuel_service.download_model(owner, model_name)
                    return result

                elif operation == "spawn":
                    if not owner or not model_name:
                        return {"error": "owner and model_name are required for spawn"}

                    # Step 1: Download
                    dl_result = await gazebo_fuel_service.download_model(owner, model_name)
                    if not dl_result.get("success") and not dl_result.get("already_exists"):
                        return {
                            "error": f"Download failed: {dl_result.get('error')}",
                            "download_result": dl_result,
                        }

                    # Step 2: Get SDF
                    sdf_xml = gazebo_fuel_service.get_spawn_sdf(owner, model_name)
                    if not sdf_xml:
                        return {
                            "success": False,
                            "error": "No SDF file found in model",
                            "message": "No SDF file found in model",
                            "local_path": dl_result.get("local_path"),
                            "hint": "Model may use URDF or custom format. Try manual spawn.",
                        }

                    # Step 3: Try spawning via Gazebo client if available
                    final_spawn_name = spawn_name or f"{model_name}_{int(time.time())}"

                    gazebo_robot = self.state_manager.get_robot("gazebo_01")
                    if gazebo_robot and hasattr(gazebo_robot, "client") and gazebo_robot.client:
                        spawn_result = await gazebo_robot.client.spawn_model(
                            final_spawn_name,
                            sdf_xml,
                            x,
                            y,
                            z,
                        )
                        return {
                            **spawn_result,
                            "model_name": model_name,
                            "spawn_name": final_spawn_name,
                        }

                    # No live Gazebo - return spawn-ready result
                    return {
                        "success": True,
                        "spawn_ready": True,
                        "model_name": model_name,
                        "spawn_name": final_spawn_name,
                        "position": {"x": x, "y": y, "z": z},
                        "local_path": dl_result.get("local_path"),
                        "sdf_available": True,
                        "note": (
                            "Model downloaded and SDF ready. "
                            "Connect Gazebo + rosbridge to spawn automatically, or use: "
                            f"gz service -s /world/default/create --reqtype gz.msgs.EntityFactory "
                            f"--reptype gz.msgs.Boolean --req 'sdf_filename: "
                            f'"model://{model_name}", name: "{final_spawn_name}"\''
                        ),
                    }

                elif operation == "list_local":
                    models = gazebo_fuel_service.list_local_models()
                    return {"models": models, "count": len(models)}

                elif operation == "delete_local":
                    if not owner or not model_name:
                        return {"error": "owner and model_name are required for delete"}
                    return gazebo_fuel_service.delete_local_model(owner, model_name)

                else:
                    return {"error": f"Unknown operation: {operation}"}

            except Exception as e:
                logger.error("gazebo_models operation failed", operation=operation, error=str(e))
                return {"error": str(e), "operation": operation}
