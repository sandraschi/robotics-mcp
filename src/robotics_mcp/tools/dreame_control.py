#!/usr/bin/env python3
"""Dreame Control Portmanteau Tool - Dreame D20 Pro Plus (DreameHome cloud).

Portmanteau pattern: Consolidates Dreame vacuum operations via DreameHome cloud (no miio):
- dreame_control: Status, cleaning, dock, map, configuration
- dreame_onboard: Cloud setup instructions (DREAME_USER, DREAME_PASSWORD)

SOTA: FastMCP 3.4.4+ compliant with conversational responses.
"""

from pathlib import Path
from typing import Any, Literal

import structlog
from fastmcp import Context

from ..utils.error_handler import format_error_response, format_success_response
from .dreame_client import get_dreame_client
from .dreame_map_export import export_dreame_map

logger = structlog.get_logger(__name__)


class DreameControlTool:
    """Dreame Control Portmanteau - D20 Pro vacuum operations."""

    def __init__(
        self,
        mcp: Any,
        state_manager: Any,
        config_loader: Any,
        mounted_servers: dict[str, Any] | None = None,
    ):
        self.mcp = mcp
        self.state_manager = state_manager
        self.config_loader = config_loader
        self.mounted = mounted_servers or {}

    def _get_dreame_config(self, robot_id: str) -> dict[str, Any]:
        """Get Dreame config from state_manager or config_loader."""
        robot = self.state_manager.get_robot(robot_id)
        if robot and hasattr(robot, "metadata") and robot.metadata is not None:
            return robot.metadata
        data = self.config_loader.load()
        robotics = data.get("robotics", {})
        for key, cfg in robotics.items():
            if isinstance(cfg, dict) and cfg.get("robot_id") == robot_id and key.startswith("dreame"):
                return cfg
        return {}

    def register(self) -> None:
        """Register dreame_control portmanteau tool."""

        @self.mcp.tool()
        async def dreame_control(
            ctx: Context,
            operation: Literal[
                "get_status",
                "start_cleaning",
                "stop_cleaning",
                "return_to_dock",
                "move",
                "play_sound",
                "set_suction_level",
                "set_water_volume",
                "set_mop_humidity",
                "get_map",
                "export_map",
                "clean_zone",
                "clean_spot",
                "clean_room",
                "get_cleaning_history",
                "clear_error",
            ],
            robot_id: str = "dreame_01",
            # Optional config override (for onboard flow)
            ip_address: str | None = None,
            token: str | None = None,
            # Cleaning parameters
            suction_level: int | None = None,
            water_volume: int | None = None,
            mop_humidity: int | None = None,
            zones: list[list[int]] | None = None,
            spot_x: int | None = None,
            spot_y: int | None = None,
            room_id: int | None = None,
            # Movement parameters
            rotation: int | None = None,
            velocity: int | None = None,
            # Export parameters
            export_formats: str | None = None,
            output_dir: str | None = None,
        ) -> dict[str, Any]:
            """Dreame D20 Pro vacuum control with conversational responses.

            Provides unified interface for Dreame vacuum operations: status, battery,
            water level, suction, LIDAR map, cleaning programs, zone/spot/room cleaning,
            and configuration.

            Operations:
            - get_status: Battery, water, suction, charging state, device status
            - start_cleaning / stop_cleaning: Full-house or spot cleaning
            - return_to_dock: Return to charging dock
            - move: Manual drive (rotation, velocity)
            - play_sound: Locate robot
            - set_suction_level: Suction power 1-4
            - set_water_volume: Water flow 1-3
            - set_mop_humidity: Mop pad humidity 1-3
            - get_map: LIDAR map data (rooms, walls, robot/charger positions)
            - export_map: Export LIDAR map to OBJ/PLY/Unity/Blender formats
            - clean_zone / clean_spot / clean_room: Targeted cleaning
            - get_cleaning_history / clear_error: History and error recovery

            Args:
                operation: Operation to perform
                robot_id: Robot identifier (default dreame_01)
                ip_address: Override IP (for testing or onboard flow)
                token: Override token (for testing or onboard flow)
                suction_level: 1-4 for set_suction_level
                water_volume: 1-3 for set_water_volume
                mop_humidity: 1-3 for set_mop_humidity
                zones: [[x1,y1,x2,y2], ...] for clean_zone
                spot_x, spot_y: Coordinates for clean_spot
                room_id: Room ID for clean_room
                rotation: Degrees for move (-120 to 120)
                velocity: Distance for move (-300 to 300)
                export_formats: Comma-separated: obj,ply,unity,blender,json (for export_map)
                output_dir: Directory for exports (for export_map, default: ~/dreame_maps)
            """
            try:
                config = self._get_dreame_config(robot_id)
                if ip_address:
                    config = {**config, "ip_address": ip_address}
                if token:
                    config = {**config, "token": token}
                client = get_dreame_client(robot_id, config)

                if operation == "get_status":
                    status = await client.get_status()
                    if status:
                        return format_success_response(
                            f"Dreame {robot_id} status",
                            data={
                                "robot_id": robot_id,
                                "battery_level": status.get("battery_level"),
                                "charging_state": status.get("charging_state"),
                                "device_status": status.get("device_status"),
                                "cleaning_mode": status.get("cleaning_mode"),
                                "fan_speed": status.get("fan_speed"),
                                "water_flow": status.get("water_flow"),
                            },
                        )
                    return format_error_response(
                        f"Failed to get Dreame {robot_id} status. Check ip_address and token.",
                        robot_id=robot_id,
                    )

                if operation == "start_cleaning":
                    ok = await client.start_cleaning()
                    return (
                        format_success_response(f"Dreame {robot_id} started cleaning", data={"robot_id": robot_id})
                        if ok
                        else format_error_response("Failed to start cleaning", robot_id=robot_id)
                    )

                if operation == "stop_cleaning":
                    ok = await client.stop_cleaning()
                    return (
                        format_success_response(f"Dreame {robot_id} stopped cleaning", data={"robot_id": robot_id})
                        if ok
                        else format_error_response("Failed to stop cleaning", robot_id=robot_id)
                    )

                if operation == "return_to_dock":
                    ok = await client.return_to_dock()
                    return (
                        format_success_response(f"Dreame {robot_id} returning to dock", data={"robot_id": robot_id})
                        if ok
                        else format_error_response("Failed to return to dock", robot_id=robot_id)
                    )

                if operation == "move":
                    rot = rotation or 0
                    vel = velocity or 50
                    ok = await client.move(rotation=rot, velocity=vel)
                    return (
                        format_success_response(
                            f"Dreame {robot_id} move (rotation={rot}, velocity={vel})",
                            data={"robot_id": robot_id},
                        )
                        if ok
                        else format_error_response("Failed to move", robot_id=robot_id)
                    )

                if operation == "play_sound":
                    ok = await client.play_sound()
                    return (
                        format_success_response(f"Dreame {robot_id} playing locate sound", data={"robot_id": robot_id})
                        if ok
                        else format_error_response("Failed to play sound", robot_id=robot_id)
                    )

                if operation == "set_suction_level":
                    if suction_level is None:
                        return format_error_response("suction_level required (1-4)", robot_id=robot_id)
                    ok = await client.set_suction_level(suction_level)
                    return (
                        format_success_response(
                            f"Dreame {robot_id} suction set to {suction_level}",
                            data={"robot_id": robot_id, "suction_level": suction_level},
                        )
                        if ok
                        else format_error_response("Failed to set suction", robot_id=robot_id)
                    )

                if operation == "set_water_volume":
                    if water_volume is None:
                        return format_error_response("water_volume required (1-3)", robot_id=robot_id)
                    ok = await client.set_water_volume(water_volume)
                    return (
                        format_success_response(
                            f"Dreame {robot_id} water volume set to {water_volume}",
                            data={"robot_id": robot_id, "water_volume": water_volume},
                        )
                        if ok
                        else format_error_response("Failed to set water volume", robot_id=robot_id)
                    )

                if operation == "set_mop_humidity":
                    if mop_humidity is None:
                        return format_error_response("mop_humidity required (1-3)", robot_id=robot_id)
                    ok = await client.set_mop_humidity(mop_humidity)
                    return (
                        format_success_response(
                            f"Dreame {robot_id} mop humidity set to {mop_humidity}",
                            data={"robot_id": robot_id, "mop_humidity": mop_humidity},
                        )
                        if ok
                        else format_error_response("Failed to set mop humidity", robot_id=robot_id)
                    )

                if operation == "get_map":
                    map_data = await client.get_map()
                    if map_data:
                        return format_success_response(
                            f"Dreame {robot_id} map retrieved - "
                            f"{len(map_data.get('rooms', []))} rooms, "
                            f"{map_data.get('wall_count', 0)} wall pixels, "
                            f"{map_data.get('total_floor_area_m2', 0)} m2 floor",
                            data={"robot_id": robot_id, "map": map_data},
                        )
                    return format_error_response(
                        "Failed to retrieve map. Set DREAME_USER/DREAME_PASSWORD and DREAME_REF_PATH.",
                        robot_id=robot_id,
                        details={
                            "troubleshooting": [
                                "Set DREAME_USER and DREAME_PASSWORD (DreameHome cloud)",
                                "Set DREAME_REF_PATH to Tasshack ref clone if needed",
                                "Run dreame_onboard for setup instructions",
                            ],
                        },
                    )

                if operation == "export_map":
                    map_data = await client.get_map()
                    if not map_data:
                        return format_error_response(
                            "Cannot export: no map data available from robot. "
                            "Run get_map first to verify connectivity.",
                            robot_id=robot_id,
                        )
                    fmt_list = (
                        [f.strip() for f in export_formats.split(",")]
                        if export_formats
                        else ["obj", "ply", "unity", "blender", "json"]
                    )
                    out = output_dir or str(Path.home() / "dreame_maps")
                    result = await export_dreame_map(map_data, out, fmt_list)
                    return format_success_response(
                        f"Dreame {robot_id} map exported to {len(result.get('formats_exported', []))} formats: "
                        f"{', '.join(result.get('formats_exported', []))}",
                        data={
                            "robot_id": robot_id,
                            "export": result,
                            "next_commands": [
                                "Import OBJ into Blender: blender_import(operation='import_obj', filepath='<path>')",
                                "Import PLY point cloud: blender_import(operation='import_ply', filepath='<path>')",
                                "Import Unity NavMesh JSON into Unity editor",
                                "Run Blender script: Open dreame_map_blender.py in Blender scripting tab",
                            ],
                        },
                    )

                if operation == "clean_zone":
                    if not zones:
                        return format_error_response("zones required [[x1,y1,x2,y2], ...]", robot_id=robot_id)
                    ok = await client.clean_zone(zones)
                    return (
                        format_success_response(
                            f"Dreame {robot_id} zone cleaning started",
                            data={"robot_id": robot_id, "zones": zones},
                        )
                        if ok
                        else format_error_response(
                            "clean_zone not supported via DreameHome cloud",
                            robot_id=robot_id,
                        )
                    )

                if operation == "clean_spot":
                    sx = spot_x if spot_x is not None else 0
                    sy = spot_y if spot_y is not None else 0
                    ok = await client.clean_spot(sx, sy)
                    return (
                        format_success_response(
                            f"Dreame {robot_id} spot cleaning at ({sx},{sy})",
                            data={"robot_id": robot_id, "spot_x": sx, "spot_y": sy},
                        )
                        if ok
                        else format_error_response(
                            "clean_spot not supported via DreameHome cloud",
                            robot_id=robot_id,
                        )
                    )

                if operation == "clean_room":
                    if room_id is None:
                        return format_error_response("room_id required", robot_id=robot_id)
                    ok = await client.clean_room(room_id)
                    return (
                        format_success_response(
                            f"Dreame {robot_id} room {room_id} cleaning",
                            data={"robot_id": robot_id, "room_id": room_id},
                        )
                        if ok
                        else format_error_response(
                            "clean_room not supported via DreameHome cloud",
                            robot_id=robot_id,
                        )
                    )

                if operation == "get_cleaning_history":
                    history = await client.get_cleaning_history()
                    if history is not None:
                        return format_success_response(
                            f"Dreame {robot_id} cleaning history",
                            data={"robot_id": robot_id, "history": history},
                        )
                    return format_error_response(
                        "Cleaning history not supported",
                        robot_id=robot_id,
                    )

                if operation == "clear_error":
                    ok = await client.clear_error()
                    return (
                        format_success_response(f"Dreame {robot_id} error cleared", data={"robot_id": robot_id})
                        if ok
                        else format_error_response("clear_error not supported", robot_id=robot_id)
                    )

                return format_error_response(f"Unknown operation: {operation}", robot_id=robot_id)

            except Exception as e:
                logger.error("dreame_control failed", operation=operation, robot_id=robot_id, error=str(e))
                return format_error_response(str(e), robot_id=robot_id)

        @self.mcp.tool()
        async def dreame_onboard() -> dict[str, Any]:
            """Dreame D20 Pro Plus - DreameHome cloud setup (no local token).

            This server uses the DreameHome cloud API. Set environment variables
            and optionally the Tasshack ref clone path. No network discovery or
            miio token needed.

            Returns:
                Setup instructions and env var reference.
            """
            import os

            user = os.environ.get("DREAME_USER", "").strip()
            pwd = os.environ.get("DREAME_PASSWORD", "").strip()
            ref = os.environ.get("DREAME_REF_PATH", "D:/Dev/repos/tasshack_dreame_vacuum_ref")
            if user and pwd:
                return format_success_response(
                    "DreameHome cloud configured",
                    data={
                        "DREAME_USER": "set",
                        "DREAME_PASSWORD": "set",
                        "DREAME_REF_PATH": ref,
                        "next_commands": ["dreame_control(operation='get_status', robot_id='dreame_01')"],
                    },
                )
            return format_error_response(
                "Set DREAME_USER and DREAME_PASSWORD for DreameHome cloud (no miio token).",
                details={
                    "env_vars": {
                        "DREAME_USER": "DreameHome email or phone",
                        "DREAME_PASSWORD": "DreameHome password",
                        "DREAME_COUNTRY": "optional, default eu",
                        "DREAME_DID": "optional device ID (auto if single device)",
                        "DREAME_REF_PATH": f"optional, default {ref} (Tasshack ref clone)",
                    },
                    "next_commands": [
                        "Set env then run dreame_control(operation='get_status')",
                    ],
                },
            )
