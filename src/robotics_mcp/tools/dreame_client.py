"""Dreame robot vacuum client - DreameHome cloud only (no miio).

Uses DreameHome cloud API via clients.dreame_cloud_client. Set DREAME_USER and
DREAME_PASSWORD; optional DREAME_REF_PATH to Tasshack ref clone.
"""

from __future__ import annotations

from typing import Any

import structlog

from ..clients.dreame_cloud_client import (
    DreameCloudStatus,
    DreameHomeClient,
    client_from_env,
)
from ..utils.response_builders import (
    build_robotics_error_response,
    build_success_response,
)

logger = structlog.get_logger(__name__)

# Lazy singleton cloud client (connected on first use)
_cloud_client: DreameHomeClient | None = None
_cloud_connected = False


async def _ensure_cloud_client() -> DreameHomeClient | None:
    """Get or create cloud client; connect if not yet connected."""
    global _cloud_client, _cloud_connected
    if _cloud_client is None:
        _cloud_client = client_from_env()
    if _cloud_client is None:
        return None
    if not _cloud_connected:
        _cloud_connected = await _cloud_client.connect()
    if not _cloud_connected:
        return None
    return _cloud_client


def _status_to_dict(st: DreameCloudStatus) -> dict[str, Any]:
    """Convert DreameCloudStatus to the dict format expected by dreame_control."""
    return {
        "battery_level": st.battery,
        "charging_state": "charging" if st.is_charging else "idle",
        "device_status": st.state,
        "cleaning_mode": "cleaning" if st.is_cleaning else "idle",
        "fan_speed": st.fan_speed,
        "water_flow": "unknown",
    }


def _cloud_map_to_structured(cloud_result: dict[str, Any]) -> dict[str, Any] | None:
    """Convert cloud get_map() result to structured map for export_dreame_map."""
    if not cloud_result.get("success"):
        return None
    map_data = cloud_result.get("map_data") or {}
    robot = map_data.get("robot_position") or {}
    charger = map_data.get("charger_position") or {}
    rooms = map_data.get("rooms")
    if isinstance(rooms, int):
        rooms = [{"id": i, "name": f"Room {i}", "bounds": {}, "area_m2": 0} for i in range(rooms)]
    elif not isinstance(rooms, list):
        rooms = []
    return {
        "header": {},
        "rooms": rooms,
        "walls": [],
        "wall_count": 0,
        "floor_bounds": {"min_x": 0, "min_y": 0, "max_x": 10, "max_y": 10},
        "robot_position": robot,
        "charger_position": charger,
        "image_size": {"width": 0, "height": 0},
        "resolution_m": 0.05,
        "total_floor_area_m2": 0,
        "additional_data": None,
        "metadata": {"source": "DreameHome cloud", "parser": "cloud"},
    }


class DreameClient:
    """Dreame D20 Pro Plus client via DreameHome cloud (no local token)."""

    def __init__(self, robot_id: str, config: dict[str, Any] | None = None):
        self.robot_id = robot_id
        self.config = config or {}

    async def get_status(self) -> dict[str, Any] | None:
        client = await _ensure_cloud_client()
        if client is None:
            return None
        st = await client.get_status()
        if st.error:
            logger.warning("Dreame status error", robot_id=self.robot_id, error=st.error)
            return None
        return _status_to_dict(st)

    async def start_cleaning(self) -> bool:
        client = await _ensure_cloud_client()
        if client is None:
            return False
        out = await client.control("start_clean")
        return out.get("success", False)

    async def stop_cleaning(self) -> bool:
        client = await _ensure_cloud_client()
        if client is None:
            return False
        out = await client.control("stop")
        return out.get("success", False)

    async def return_to_dock(self) -> bool:
        client = await _ensure_cloud_client()
        if client is None:
            return False
        out = await client.control("go_home")
        return out.get("success", False)

    async def move(self, rotation: int = 0, velocity: int = 0) -> bool:
        """Manual move not exposed in cloud portmanteau; use start_clean or stop."""
        logger.warning("move not supported via DreameHome cloud", robot_id=self.robot_id)
        return False

    async def set_suction_level(self, level: int) -> bool:
        """Suction level not exposed in cloud control; no-op."""
        logger.warning("set_suction_level not supported via cloud", robot_id=self.robot_id)
        return False

    async def set_water_volume(self, volume: int) -> bool:
        """Water volume not exposed in cloud control; no-op."""
        logger.warning("set_water_volume not supported via cloud", robot_id=self.robot_id)
        return False

    async def set_mop_humidity(self, humidity: int) -> bool:
        return await self.set_water_volume(humidity)

    async def play_sound(self) -> bool:
        client = await _ensure_cloud_client()
        if client is None:
            return False
        out = await client.control("find_robot")
        return out.get("success", False)

    async def get_map(self) -> dict[str, Any] | None:
        client = await _ensure_cloud_client()
        if client is None:
            return None
        result = await client.get_map()
        return _cloud_map_to_structured(result)

    async def clean_room(self, room_id: int) -> bool:
        logger.warning("clean_room not supported via cloud", room_id=room_id)
        return False

    async def clean_zone(self, zones: list[list[int]]) -> bool:
        logger.warning("clean_zone not supported via cloud")
        return False

    async def clean_spot(self, spot_x: int, spot_y: int) -> bool:
        logger.warning("clean_spot not supported via cloud")
        return False

    async def start_fast_mapping(self) -> bool:
        return False

    async def start_mapping(self) -> bool:
        return False

    async def set_cleaning_sequence(self, sequence: list[int]) -> bool:
        return False

    async def set_restricted_zones(self, zones: dict[str, list[list[int]]]) -> bool:
        return False

    async def get_cleaning_history(self) -> list[dict[str, Any]] | None:
        return None

    async def clear_error(self) -> bool:
        return False

    async def go_to_position(self, x: float, y: float) -> bool:
        return False


def get_dreame_client(robot_id: str = "dreame_01", config: dict[str, Any] | None = None) -> DreameClient:
    """Get Dreame client (DreameHome cloud only). Config optional; env DREAME_USER/DREAME_PASSWORD used."""
    return DreameClient(robot_id, config)


async def dreame_get_status(robot_id: str = "dreame_01", config: dict[str, Any] | None = None) -> dict[str, Any]:
    client = get_dreame_client(robot_id, config)
    status = await client.get_status()
    if status:
        return build_success_response(
            operation="get_dreame_status",
            summary=f"Dreame {robot_id} status retrieved",
            result={"status": status, "robot_id": robot_id},
        )
    return build_robotics_error_response(
        error="Dreame status failed - set DREAME_USER and DREAME_PASSWORD (DreameHome cloud)",
        robot_type="dreame",
        robot_id=robot_id,
        recovery_options=[
            "Set DREAME_USER and DREAME_PASSWORD environment variables",
            "Optional: DREAME_REF_PATH to Tasshack ref clone (default D:/Dev/repos/tasshack_dreame_vacuum_ref)",
        ],
    )


async def dreame_start_cleaning(robot_id: str = "dreame_01", config: dict[str, Any] | None = None) -> dict[str, Any]:
    client = get_dreame_client(robot_id, config)
    success = await client.start_cleaning()
    if success:
        return build_success_response(
            operation="start_dreame_cleaning",
            summary=f"Dreame {robot_id} started cleaning",
            result={"robot_id": robot_id, "action": "start_cleaning"},
        )
    return build_robotics_error_response(
        error="Failed to start cleaning - check DREAME_USER/DREAME_PASSWORD",
        robot_type="dreame",
        robot_id=robot_id,
    )


async def dreame_stop_cleaning(robot_id: str = "dreame_01", config: dict[str, Any] | None = None) -> dict[str, Any]:
    client = get_dreame_client(robot_id, config)
    success = await client.stop_cleaning()
    if success:
        return build_success_response(
            operation="stop_dreame_cleaning",
            summary=f"Dreame {robot_id} stopped cleaning",
            result={"robot_id": robot_id, "action": "stop_cleaning"},
        )
    return build_robotics_error_response(
        error="Failed to stop cleaning",
        robot_type="dreame",
        robot_id=robot_id,
    )


async def dreame_move(
    robot_id: str = "dreame_01",
    rotation: int = 0,
    velocity: int = 0,
    config: dict[str, Any] | None = None,
) -> dict[str, Any]:
    client = get_dreame_client(robot_id, config)
    success = await client.move(rotation=rotation, velocity=velocity)
    if success:
        return build_success_response(
            operation="move_dreame",
            summary=f"Dreame {robot_id} moved",
            result={"robot_id": robot_id, "rotation": rotation, "velocity": velocity},
        )
    return build_robotics_error_response(
        error="move not supported via DreameHome cloud",
        robot_type="dreame",
        robot_id=robot_id,
    )


async def dreame_get_map(robot_id: str = "dreame_01", config: dict[str, Any] | None = None) -> dict[str, Any]:
    client = get_dreame_client(robot_id, config)
    map_data = await client.get_map()
    if map_data:
        return build_success_response(
            operation="get_dreame_map",
            summary=f"Dreame {robot_id} map retrieved",
            result={"robot_id": robot_id, "map": map_data},
        )
    return build_robotics_error_response(
        error="Map not available - set DREAME_USER/DREAME_PASSWORD and DREAME_REF_PATH",
        robot_type="dreame",
        robot_id=robot_id,
    )


async def dreame_clean_room(
    robot_id: str = "dreame_01", room_id: int = 1, config: dict[str, Any] | None = None
) -> dict[str, Any]:
    client = get_dreame_client(robot_id, config)
    success = await client.clean_room(room_id)
    if success:
        return build_success_response(
            operation="clean_dreame_room",
            summary=f"Dreame {robot_id} cleaning room {room_id}",
            result={"robot_id": robot_id, "room_id": room_id},
        )
    return build_robotics_error_response(
        error="clean_room not supported via cloud",
        robot_type="dreame",
        robot_id=robot_id,
    )


async def dreame_clean_zone(
    robot_id: str = "dreame_01",
    zones: list[list[int]] | None = None,
    config: dict[str, Any] | None = None,
) -> dict[str, Any]:
    if not zones:
        return build_robotics_error_response(
            error="No zones specified for cleaning",
            robot_type="dreame",
            robot_id=robot_id,
        )
    client = get_dreame_client(robot_id, config)
    success = await client.clean_zone(zones)
    if success:
        return build_success_response(
            operation="clean_dreame_zone",
            summary=f"Dreame {robot_id} cleaning zones",
            result={"robot_id": robot_id, "zones": zones},
        )
    return build_robotics_error_response(
        error="clean_zone not supported via cloud",
        robot_type="dreame",
        robot_id=robot_id,
    )


async def dreame_clean_spot(
    robot_id: str = "dreame_01",
    spot_x: int = 0,
    spot_y: int = 0,
    config: dict[str, Any] | None = None,
) -> dict[str, Any]:
    client = get_dreame_client(robot_id, config)
    success = await client.clean_spot(spot_x, spot_y)
    if success:
        return build_success_response(
            operation="clean_dreame_spot",
            summary=f"Dreame {robot_id} spot clean",
            result={"robot_id": robot_id, "spot_x": spot_x, "spot_y": spot_y},
        )
    return build_robotics_error_response(
        error="clean_spot not supported via cloud",
        robot_type="dreame",
        robot_id=robot_id,
    )
