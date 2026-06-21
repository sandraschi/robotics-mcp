"""Fleet fab-art bridge to inkscape-mcp via HTTP (DXF, schematics, staging)."""

from __future__ import annotations

from typing import Any, Literal

import structlog
from fastmcp import Context

from ..utils.error_handler import format_error_response, format_success_response
from ..utils.fleet_http_client import (
    DEFAULT_GIMP_URL,
    DEFAULT_INKSCAPE_URL,
    call_inkscape_tool,
    check_fleet_health,
)

logger = structlog.get_logger(__name__)


class FabArtFleetTool:
    """Robotics composition layer for inkscape-mcp fab art over HTTP."""

    def __init__(self, mcp: Any, state_manager: Any, mounted_servers: dict[str, Any] | None = None):
        self.mcp = mcp
        self.state_manager = state_manager
        self.mounted = mounted_servers or {}

    def register(self) -> None:
        @self.mcp.tool()
        async def robotics_fab_art(
            ctx: Context,
            operation: Literal[
                "inkscape_status",
                "batch_dxf_export",
                "batch_laser_dots",
                "gazebo_schematic",
                "stage_fab_paths",
                "push_gimp_schematic",
            ],
            input_dir: str = "",
            svg_path: str = "",
            output_dir: str = "",
            preset_id: str = "gazebo_model_doc_192",
            laser_preset_id: str = "fab_calibration_grid",
            inkscape_url: str = DEFAULT_INKSCAPE_URL,
            gimp_url: str = DEFAULT_GIMP_URL,
            push_gimp: bool = True,
        ) -> dict[str, Any]:
            """Fab art fleet bridge via inkscape-mcp HTTP (DXF, laser dots, Gazebo schematics).

            Operations:
                inkscape_status: Probe inkscape-mcp and list fab presets.
                batch_dxf_export: Run inkscape_fab_art batch_dxf_export on input_dir.
                batch_laser_dots: Generate laser dot SVG batch from preset.
                gazebo_schematic: SVG to PNG schematic for model documentation.
                stage_fab_paths: Stage DXF/SVG/PNG outputs for robotics workflows.
                push_gimp_schematic: Export schematic and push through gimp validation.
            """
            await ctx.info(f"robotics_fab_art: {operation}")

            try:
                if operation == "inkscape_status":
                    online = await check_fleet_health(inkscape_url)
                    presets: dict[str, Any] = {}
                    if online:
                        presets = await call_inkscape_tool(
                            "inkscape_fab_art",
                            {"operation": "list_presets"},
                            base_url=inkscape_url,
                        )
                    return format_success_response(
                        "inkscape-mcp fab art status",
                        data={
                            "inkscape_url": inkscape_url,
                            "inkscape_reachable": online,
                            "presets": presets,
                        },
                    )

                if operation == "batch_dxf_export":
                    if not input_dir:
                        return format_error_response("input_dir required", error_type="validation_error")
                    result = await call_inkscape_tool(
                        "inkscape_fab_art",
                        {
                            "operation": "batch_dxf_export",
                            "input_dir": input_dir,
                            "output_dir": output_dir,
                        },
                        base_url=inkscape_url,
                    )
                    if not result.get("success"):
                        return format_error_response(
                            result.get("error") or "DXF batch failed",
                            error_type="inkscape_error",
                            details=result,
                        )
                    return format_success_response("DXF batch complete", data=result)

                if operation == "batch_laser_dots":
                    result = await call_inkscape_tool(
                        "inkscape_fab_art",
                        {
                            "operation": "batch_laser_dots",
                            "output_dir": output_dir,
                            "laser_preset_id": laser_preset_id,
                        },
                        base_url=inkscape_url,
                    )
                    if not result.get("success"):
                        return format_error_response(
                            result.get("error") or "Laser dot batch failed",
                            error_type="inkscape_error",
                            details=result,
                        )
                    return format_success_response("Laser dot batch complete", data=result)

                if operation == "gazebo_schematic":
                    if not svg_path:
                        return format_error_response("svg_path required", error_type="validation_error")
                    result = await call_inkscape_tool(
                        "inkscape_fab_art",
                        {
                            "operation": "gazebo_schematic",
                            "svg_path": svg_path,
                            "preset_id": preset_id,
                            "push_gimp": push_gimp,
                            "gimp_url": gimp_url,
                        },
                        base_url=inkscape_url,
                    )
                    if not result.get("success"):
                        return format_error_response(
                            result.get("error") or "Schematic export failed",
                            error_type="inkscape_error",
                            details=result,
                        )
                    return format_success_response("Gazebo schematic exported", data=result)

                if operation == "stage_fab_paths":
                    result = await call_inkscape_tool(
                        "inkscape_fab_art",
                        {
                            "operation": "stage_for_robotics",
                            "input_dir": input_dir,
                            "svg_path": svg_path,
                        },
                        base_url=inkscape_url,
                    )
                    if not result.get("success"):
                        return format_error_response(
                            result.get("error") or "Fab staging failed",
                            error_type="inkscape_error",
                            details=result,
                        )
                    return format_success_response("Fab paths staged", data=result)

                if operation == "push_gimp_schematic":
                    if not svg_path:
                        return format_error_response("svg_path required", error_type="validation_error")
                    schematic = await call_inkscape_tool(
                        "inkscape_fab_art",
                        {
                            "operation": "gazebo_schematic",
                            "svg_path": svg_path,
                            "preset_id": preset_id,
                            "push_gimp": True,
                            "gimp_url": gimp_url,
                        },
                        base_url=inkscape_url,
                    )
                    if not schematic.get("success"):
                        return format_error_response(
                            schematic.get("error") or "Schematic handoff failed",
                            error_type="inkscape_error",
                            details=schematic,
                        )
                    gimp_data = (schematic.get("data") or {}).get("gimp_handoff")
                    if push_gimp and gimp_data and not gimp_data.get("success"):
                        return format_error_response(
                            gimp_data.get("error") or "GIMP validation failed",
                            error_type="gimp_error",
                            details=gimp_data,
                        )
                    return format_success_response("Schematic pushed through GIMP QA", data=schematic)

                return format_error_response(f"Unknown operation: {operation}", error_type="validation_error")
            except Exception as exc:
                logger.exception("robotics_fab_art failed", operation=operation)
                return format_error_response(str(exc), error_type="fab_art_error")
