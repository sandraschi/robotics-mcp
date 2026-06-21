"""Fleet sim-art bridge to gimp-mcp via HTTP (no stdio mount)."""

from __future__ import annotations

from typing import Any, Literal

import structlog
from fastmcp import Context

from ..utils.error_handler import format_error_response, format_success_response
from ..utils.fleet_http_client import (
    DEFAULT_AVATAR_URL,
    DEFAULT_GIMP_URL,
    call_avatar_tool,
    call_gimp_tool,
    check_fleet_health,
)

logger = structlog.get_logger(__name__)


class SimArtFleetTool:
    """Robotics composition layer for gimp-mcp sim-art and avatar thumbnails."""

    def __init__(self, mcp: Any, state_manager: Any, mounted_servers: dict[str, Any] | None = None):
        self.mcp = mcp
        self.state_manager = state_manager
        self.mounted = mounted_servers or {}

    def register(self) -> None:
        @self.mcp.tool()
        async def robotics_sim_art(
            ctx: Context,
            operation: Literal[
                "gimp_status",
                "batch_gazebo_icons",
                "import_gazebo_models",
                "batch_vrchat_icons",
                "avatar_thumbnail",
            ],
            input_dir: str = "",
            models_root: str = "",
            model_dir: str = "",
            icon_path: str = "",
            avatar_id: str = "",
            template_id: str = "gazebo_icon_256",
            gimp_url: str = DEFAULT_GIMP_URL,
            avatar_url: str = DEFAULT_AVATAR_URL,
            auto_import: bool = True,
        ) -> dict[str, Any]:
            """Sim-art fleet bridge via gimp-mcp HTTP (Gazebo icons, VRChat, avatar thumbs).

            Uses HTTP tool calls instead of mounting gimp-mcp stdio (avoids protocol hangs).

            Operations:
                gimp_status: Probe gimp-mcp health and list sim-art templates.
                batch_gazebo_icons: Run gimp_sim_art_tool gazebo_model_icons on input_dir.
                import_gazebo_models: Batch import thumbnails into models_root (auto_import).
                batch_vrchat_icons: Run vrchat_icon_batch on input_dir.
                avatar_thumbnail: Push icon_path to avatar via avatar_manager set_thumbnail.
            """
            await ctx.info(f"robotics_sim_art: {operation}")

            try:
                if operation == "gimp_status":
                    online = await check_fleet_health(gimp_url)
                    templates = {}
                    if online:
                        templates = await call_gimp_tool(
                            "gimp_sim_art_tool",
                            {"operation": "list_templates"},
                            base_url=gimp_url,
                        )
                    return format_success_response(
                        "gimp-mcp fleet status",
                        data={
                            "gimp_url": gimp_url,
                            "gimp_reachable": online,
                            "templates": templates,
                        },
                    )

                if operation == "batch_gazebo_icons":
                    if not input_dir:
                        return format_error_response("input_dir required", error_type="validation_error")
                    result = await call_gimp_tool(
                        "gimp_sim_art_tool",
                        {
                            "operation": "gazebo_model_icons",
                            "input_dir": input_dir,
                            "template_id": template_id,
                            "validate": True,
                        },
                        base_url=gimp_url,
                    )
                    if not result.get("success"):
                        return format_error_response(
                            result.get("error") or "Gazebo icon batch failed",
                            error_type="gimp_error",
                            details=result,
                        )
                    return format_success_response(
                        f"Batch Gazebo icons: {result.get('data', {}).get('count', 0)} file(s)",
                        data=result,
                    )

                if operation == "import_gazebo_models":
                    icons = input_dir or ""
                    if not icons or not models_root:
                        return format_error_response(
                            "input_dir (icons) and models_root required",
                            error_type="validation_error",
                        )
                    result = await call_gimp_tool(
                        "gimp_sim_art_tool",
                        {
                            "operation": "batch_import_gazebo",
                            "input_dir": icons,
                            "models_root": models_root,
                        },
                        base_url=gimp_url,
                    )
                    if not result.get("success"):
                        return format_error_response(
                            result.get("error") or "Gazebo import failed",
                            error_type="gimp_error",
                            details=result,
                        )
                    return format_success_response(
                        f"Imported {result.get('data', {}).get('imported', 0)} Gazebo thumbnail(s)",
                        data=result,
                    )

                if operation == "batch_vrchat_icons":
                    if not input_dir:
                        return format_error_response("input_dir required", error_type="validation_error")
                    result = await call_gimp_tool(
                        "gimp_sim_art_tool",
                        {
                            "operation": "vrchat_icon_batch",
                            "input_dir": input_dir,
                            "template_id": "vrchat_profile_256",
                        },
                        base_url=gimp_url,
                    )
                    if not result.get("success"):
                        return format_error_response(
                            result.get("error") or "VRChat batch failed",
                            error_type="gimp_error",
                            details=result,
                        )
                    return format_success_response("VRChat icon batch complete", data=result)

                if operation == "avatar_thumbnail":
                    if not icon_path or not avatar_id:
                        return format_error_response(
                            "icon_path and avatar_id required",
                            error_type="validation_error",
                        )
                    if auto_import:
                        avatar_result = await call_avatar_tool(
                            "avatar_manager",
                            {
                                "operation": "set_thumbnail",
                                "avatar_id": avatar_id,
                                "icon_path": icon_path,
                            },
                            base_url=avatar_url,
                        )
                        if avatar_result.get("success") or avatar_result.get("status") == "success":
                            return format_success_response(
                                f"Avatar thumbnail set for {avatar_id}",
                                data=avatar_result,
                            )
                    gimp_result = await call_gimp_tool(
                        "gimp_sim_art_tool",
                        {
                            "operation": "import_avatar_model",
                            "icon_path": icon_path,
                            "model_id": avatar_id,
                            "auto_import": True,
                        },
                        base_url=gimp_url,
                    )
                    if not gimp_result.get("success"):
                        return format_error_response(
                            gimp_result.get("error") or "Avatar thumbnail import failed",
                            error_type="handoff_error",
                            details=gimp_result,
                        )
                    return format_success_response(
                        f"Avatar thumbnail imported for {avatar_id}",
                        data=gimp_result,
                    )

                return format_error_response(f"Unknown operation: {operation}", error_type="validation_error")
            except Exception as exc:
                logger.exception("robotics_sim_art failed", operation=operation)
                return format_error_response(str(exc), error_type="sim_art_error")
