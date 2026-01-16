"""Dreame robot vacuum client for Robotics MCP.

Provides full control over Dreame robot vacuums including navigation, cleaning,
and status monitoring via the dreame-vacuum library.
"""

import asyncio
from typing import Any, Dict, List, Optional, Tuple

import structlog

from ..utils.response_builders import (
    build_robotics_error_response,
    build_success_response,
    build_error_response,
)

logger = structlog.get_logger(__name__)


class DreameClient:
    """Client for controlling Dreame robot vacuums."""

    def __init__(self, robot_id: str, config: Optional[Dict[str, Any]] = None):
        """Initialize Dreame client.

        Args:
            robot_id: Unique identifier for the robot
            config: Configuration including IP, token, etc.
        """
        self.robot_id = robot_id
        self.config = config or {}
        self.device = None
        self._connected = False

        # Dreame-specific configuration
        self.ip_address = self.config.get("ip_address")
        self.token = self.config.get("token")
        self.username = self.config.get("username")
        self.password = self.config.get("password")

    async def connect(self) -> bool:
        """Connect to Dreame robot.

        Returns:
            True if connection successful, False otherwise
        """
        try:
            if not self.ip_address or not self.token:
                logger.error("Missing Dreame configuration", robot_id=self.robot_id)
                return False

            # Import dreame vacuum library
            from dreame import DreameVacuum

            # Create device instance
            self.device = DreameVacuum(self.ip_address, self.token)

            # Connect and get status
            await self.device.connect()
            status = await self.device.get_status()

            if status:
                self._connected = True
                logger.info("Connected to Dreame robot", robot_id=self.robot_id, ip=self.ip_address)
                return True
            else:
                logger.error("Failed to get status from Dreame robot", robot_id=self.robot_id)
                return False

        except Exception as e:
            logger.error("Failed to connect to Dreame robot", robot_id=self.robot_id, error=str(e))
            return False

    async def disconnect(self) -> None:
        """Disconnect from Dreame robot."""
        if self.device:
            try:
                await self.device.disconnect()
            except Exception as e:
                logger.warning("Error disconnecting from Dreame", robot_id=self.robot_id, error=str(e))
        self._connected = False

    async def get_status(self) -> Optional[Dict[str, Any]]:
        """Get current robot status.

        Returns:
            Status dictionary or None if failed
        """
        if not self._connected or not self.device:
            return None

        try:
            status = await self.device.get_status()
            if status:
                return {
                    "battery_level": getattr(status, "battery_level", 0),
                    "charging_status": getattr(status, "charging_status", False),
                    "cleaning_status": getattr(status, "cleaning_status", "unknown"),
                    "error_code": getattr(status, "error_code", 0),
                    "fan_speed": getattr(status, "fan_speed", "unknown"),
                }
        except Exception as e:
            logger.error("Failed to get Dreame status", robot_id=self.robot_id, error=str(e))
        return None

    async def get_position(self) -> Optional[Dict[str, float]]:
        """Get current robot position.

        Returns:
            Position dictionary with x, y coordinates or None
        """
        if not self._connected or not self.device:
            return None

        try:
            position = await self.device.get_position()
            if position:
                return {
                    "x": getattr(position, "x", 0.0),
                    "y": getattr(position, "y", 0.0),
                    "heading": getattr(position, "heading", 0.0),
                }
        except Exception as e:
            logger.error("Failed to get Dreame position", robot_id=self.robot_id, error=str(e))
        return None

    async def start_cleaning(self) -> bool:
        """Start cleaning operation.

        Returns:
            True if successful, False otherwise
        """
        if not self._connected or not self.device:
            return False

        try:
            await self.device.start()
            return True
        except Exception as e:
            logger.error("Failed to start Dreame cleaning", robot_id=self.robot_id, error=str(e))
            return False

    async def stop_cleaning(self) -> bool:
        """Stop cleaning operation.

        Returns:
            True if successful, False otherwise
        """
        if not self._connected or not self.device:
            return False

        try:
            await self.device.stop()
            return True
        except Exception as e:
            logger.error("Failed to stop Dreame cleaning", robot_id=self.robot_id, error=str(e))
            return False

    async def pause_cleaning(self) -> bool:
        """Pause cleaning operation.

        Returns:
            True if successful, False otherwise
        """
        if not self._connected or not self.device:
            return False

        try:
            await self.device.pause()
            return True
        except Exception as e:
            logger.error("Failed to pause Dreame cleaning", robot_id=self.robot_id, error=str(e))
            return False

    async def return_to_dock(self) -> bool:
        """Return to charging dock.

        Returns:
            True if successful, False otherwise
        """
        if not self._connected or not self.device:
            return False

        try:
            await self.device.return_to_dock()
            return True
        except Exception as e:
            logger.error("Failed to send Dreame to dock", robot_id=self.robot_id, error=str(e))
            return False

    async def move(self, rotation: int = 0, velocity: int = 0) -> bool:
        """Move robot with specified rotation and velocity.

        Args:
            rotation: Rotation angle in degrees (-180 to 180)
            velocity: Movement velocity (-100 to 100)

        Returns:
            True if successful, False otherwise
        """
        if not self._connected or not self.device:
            return False

        try:
            await self.device.remote_control_move_step(rotation=rotation, velocity=velocity)
            return True
        except Exception as e:
            logger.error("Failed to move Dreame robot", robot_id=self.robot_id, rotation=rotation, velocity=velocity, error=str(e))
            return False

    async def go_to_position(self, x: float, y: float) -> bool:
        """Navigate to specific position (if supported).

        Args:
            x: X coordinate
            y: Y coordinate

        Returns:
            True if successful, False otherwise
        """
        # Dreame may support goto via zones or specific coordinates
        # For now, implement as zone cleaning if supported
        if not self._connected or not self.device:
            return False

        try:
            # Try to use goto if available, otherwise fall back to zone cleaning
            if hasattr(self.device, 'goto'):
                await self.device.goto(x, y)
                return True
            else:
                # Fall back to spot cleaning at coordinates
                await self.device.clean_spot([int(x), int(y)])
                return True
        except Exception as e:
            logger.error("Failed to navigate Dreame to position", robot_id=self.robot_id, x=x, y=y, error=str(e))
            return False

    async def get_map(self) -> Optional[Dict[str, Any]]:
        """Get current map data.

        Returns:
            Map data dictionary or None if failed
        """
        if not self._connected or not self.device:
            return None

        try:
            map_data = await self.device.get_map()
            if map_data:
                return {
                    "map_id": getattr(map_data, "map_id", "unknown"),
                    "rooms": getattr(map_data, "rooms", []),
                    "obstacles": getattr(map_data, "obstacles", []),
                    "charging_station": getattr(map_data, "charging_station", None),
                }
        except Exception as e:
            logger.error("Failed to get Dreame map", robot_id=self.robot_id, error=str(e))
        return None

    async def clean_room(self, room_id: int) -> bool:
        """Clean specific room.

        Args:
            room_id: Room identifier

        Returns:
            True if successful, False otherwise
        """
        if not self._connected or not self.device:
            return False

        try:
            await self.device.clean_segment([room_id])
            return True
        except Exception as e:
            logger.error("Failed to clean Dreame room", robot_id=self.robot_id, room_id=room_id, error=str(e))
            return False

    async def clean_zone(self, zones: list) -> bool:
        """Clean specific zone(s).

        Args:
            zones: List of zone coordinates [[x1,y1,x2,y2], ...]

        Returns:
            True if successful, False otherwise
        """
        if not self._connected or not self.device:
            return False

        try:
            await self.device.clean_zone(zones)
            return True
        except Exception as e:
            logger.error("Failed to clean Dreame zone", robot_id=self.robot_id, zones=zones, error=str(e))
            return False

    async def clean_spot(self, spot_x: int, spot_y: int) -> bool:
        """Clean specific spot.

        Args:
            spot_x: X coordinate for spot cleaning
            spot_y: Y coordinate for spot cleaning

        Returns:
            True if successful, False otherwise
        """
        if not self._connected or not self.device:
            return False

        try:
            await self.device.clean_spot([spot_x, spot_y])
            return True
        except Exception as e:
            logger.error("Failed to clean Dreame spot", robot_id=self.robot_id, spot_x=spot_x, spot_y=spot_y, error=str(e))
            return False

    async def set_suction_level(self, level: int) -> bool:
        """Set suction power level.

        Args:
            level: Suction level (1-4)

        Returns:
            True if successful, False otherwise
        """
        if not self._connected or not self.device:
            return False

        try:
            await self.device.set_suction_level(level)
            return True
        except Exception as e:
            logger.error("Failed to set Dreame suction level", robot_id=self.robot_id, level=level, error=str(e))
            return False

    async def set_water_volume(self, volume: int) -> bool:
        """Set water volume level for mopping.

        Args:
            volume: Water volume level (1-3)

        Returns:
            True if successful, False otherwise
        """
        if not self._connected or not self.device:
            return False

        try:
            await self.device.set_water_volume(volume)
            return True
        except Exception as e:
            logger.error("Failed to set Dreame water volume", robot_id=self.robot_id, volume=volume, error=str(e))
            return False

    async def set_mop_humidity(self, humidity: int) -> bool:
        """Set mop pad humidity level.

        Args:
            humidity: Mop humidity level (1-3)

        Returns:
            True if successful, False otherwise
        """
        if not self._connected or not self.device:
            return False

        try:
            await self.device.set_mop_humidity(humidity)
            return True
        except Exception as e:
            logger.error("Failed to set Dreame mop humidity", robot_id=self.robot_id, humidity=humidity, error=str(e))
            return False

    async def start_fast_mapping(self) -> bool:
        """Start fast mapping mode.

        Returns:
            True if successful, False otherwise
        """
        if not self._connected or not self.device:
            return False

        try:
            await self.device.start_fast_mapping()
            return True
        except Exception as e:
            logger.error("Failed to start Dreame fast mapping", robot_id=self.robot_id, error=str(e))
            return False

    async def start_mapping(self) -> bool:
        """Start standard mapping mode.

        Returns:
            True if successful, False otherwise
        """
        if not self._connected or not self.device:
            return False

        try:
            await self.device.start_mapping()
            return True
        except Exception as e:
            logger.error("Failed to start Dreame mapping", robot_id=self.robot_id, error=str(e))
            return False

    async def set_cleaning_sequence(self, sequence: List[int]) -> bool:
        """Set room cleaning sequence.

        Args:
            sequence: List of room IDs in cleaning order

        Returns:
            True if successful, False otherwise
        """
        if not self._connected or not self.device:
            return False

        try:
            await self.device.set_cleaning_sequence(sequence)
            return True
        except Exception as e:
            logger.error("Failed to set Dreame cleaning sequence", robot_id=self.robot_id, sequence=sequence, error=str(e))
            return False

    async def set_restricted_zones(self, zones: Dict[str, List[List[int]]]) -> bool:
        """Set restricted zones (virtual walls and no-go zones).

        Args:
            zones: Dictionary with 'walls' and 'zones' keys containing coordinates

        Returns:
            True if successful, False otherwise
        """
        if not self._connected or not self.device:
            return False

        try:
            await self.device.set_restricted_zones(zones)
            return True
        except Exception as e:
            logger.error("Failed to set Dreame restricted zones", robot_id=self.robot_id, zones=zones, error=str(e))
            return False

    async def get_cleaning_history(self) -> Optional[List[Dict[str, Any]]]:
        """Get cleaning history.

        Returns:
            List of cleaning sessions or None if failed
        """
        if not self._connected or not self.device:
            return None

        try:
            history = await self.device.get_cleaning_history()
            return history
        except Exception as e:
            logger.error("Failed to get Dreame cleaning history", robot_id=self.robot_id, error=str(e))
            return None

    async def clear_error(self) -> bool:
        """Clear error conditions.

        Returns:
            True if successful, False otherwise
        """
        if not self._connected or not self.device:
            return False

        try:
            await self.device.clear_error()
            return True
        except Exception as e:
            logger.error("Failed to clear Dreame error", robot_id=self.robot_id, error=str(e))
            return False


# Global Dreame client instance
_dreame_client: Optional[DreameClient] = None


def get_dreame_client(robot_id: str = "dreame_01", config: Optional[Dict[str, Any]] = None) -> DreameClient:
    """Get or create Dreame client instance.

    Args:
        robot_id: Robot identifier
        config: Client configuration

    Returns:
        DreameClient instance
    """
    global _dreame_client
    if _dreame_client is None:
        _dreame_client = DreameClient(robot_id, config)
    return _dreame_client


async def initialize_dreame_client(robot_id: str = "dreame_01", config: Optional[Dict[str, Any]] = None) -> bool:
    """Initialize and connect Dreame client.

    Args:
        robot_id: Robot identifier
        config: Client configuration

    Returns:
        True if initialization successful, False otherwise
    """
    client = get_dreame_client(robot_id, config)
    return await client.connect()


async def dreame_get_status(robot_id: str = "dreame_01") -> Dict[str, Any]:
    """Get Dreame robot status."""
    client = get_dreame_client(robot_id)
    status = await client.get_status()
    if status:
        return build_success_response(
            operation="get_dreame_status",
            summary=f"Dreame {robot_id} status retrieved",
            result={"status": status, "robot_id": robot_id}
        )
    else:
        return build_robotics_error_response(
            error="Failed to get Dreame status",
            robot_type="dreame",
            robot_id=robot_id,
            recovery_options=[
                "Check Dreame robot is powered on and connected to WiFi",
                "Verify Dreame app configuration and credentials",
                "Try restarting the Dreame robot",
                "Check network connectivity between MCP server and Dreame"
            ]
        )


async def dreame_start_cleaning(robot_id: str = "dreame_01") -> Dict[str, Any]:
    """Start Dreame cleaning."""
    client = get_dreame_client(robot_id)
    success = await client.start_cleaning()
    if success:
        return build_success_response(
            operation="start_dreame_cleaning",
            summary=f"Dreame {robot_id} started cleaning",
            result={"robot_id": robot_id, "action": "start_cleaning"}
        )
    else:
        return build_robotics_error_response(
            error="Failed to start Dreame cleaning",
            robot_type="dreame",
            robot_id=robot_id
        )


async def dreame_stop_cleaning(robot_id: str = "dreame_01") -> Dict[str, Any]:
    """Stop Dreame cleaning."""
    client = get_dreame_client(robot_id)
    success = await client.stop_cleaning()
    if success:
        return build_success_response(
            operation="stop_dreame_cleaning",
            summary=f"Dreame {robot_id} stopped cleaning",
            result={"robot_id": robot_id, "action": "stop_cleaning"}
        )
    else:
        return build_robotics_error_response(
            error="Failed to stop Dreame cleaning",
            robot_type="dreame",
            robot_id=robot_id
        )


async def dreame_move(robot_id: str = "dreame_01", rotation: int = 0, velocity: int = 0) -> Dict[str, Any]:
    """Move Dreame robot with rotation and velocity."""
    client = get_dreame_client(robot_id)
    success = await client.move(rotation=rotation, velocity=velocity)
    if success:
        return build_success_response(
            operation="move_dreame",
            summary=f"Dreame {robot_id} moved (rotation: {rotation}°, velocity: {velocity})",
            result={"robot_id": robot_id, "rotation": rotation, "velocity": velocity}
        )
    else:
        return build_robotics_error_response(
            error="Failed to move Dreame robot",
            robot_type="dreame",
            robot_id=robot_id
        )


async def dreame_get_map(robot_id: str = "dreame_01") -> Dict[str, Any]:
    """Get Dreame map data."""
    client = get_dreame_client(robot_id)
    map_data = await client.get_map()
    if map_data:
        return build_success_response(
            operation="get_dreame_map",
            summary=f"Dreame {robot_id} map retrieved",
            result={"robot_id": robot_id, "map": map_data}
        )
    else:
        return build_robotics_error_response(
            error="Failed to get Dreame map",
            robot_type="dreame",
            robot_id=robot_id
        )


async def dreame_clean_room(robot_id: str = "dreame_01", room_id: int = 1) -> Dict[str, Any]:
    """Clean specific room with Dreame."""
    client = get_dreame_client(robot_id)
    success = await client.clean_room(room_id)
    if success:
        return build_success_response(
            operation="clean_dreame_room",
            summary=f"Dreame {robot_id} cleaning room {room_id}",
            result={"robot_id": robot_id, "room_id": room_id}
        )
    else:
        return build_robotics_error_response(
            error="Failed to clean room with Dreame",
            robot_type="dreame",
            robot_id=robot_id
        )


async def dreame_clean_zone(robot_id: str = "dreame_01", zones: List[List[int]] = None) -> Dict[str, Any]:
    """Clean specific zone(s) with Dreame."""
    if not zones:
        return build_robotics_error_response(
            error="No zones specified for cleaning",
            robot_type="dreame",
            robot_id=robot_id
        )

    client = get_dreame_client(robot_id)
    success = await client.clean_zone(zones)
    if success:
        return build_success_response(
            operation="clean_dreame_zone",
            summary=f"Dreame {robot_id} cleaning {len(zones)} zone(s)",
            result={"robot_id": robot_id, "zones": zones}
        )
    else:
        return build_robotics_error_response(
            error="Failed to clean zone with Dreame",
            robot_type="dreame",
            robot_id=robot_id
        )


async def dreame_clean_spot(robot_id: str = "dreame_01", spot_x: int = 0, spot_y: int = 0) -> Dict[str, Any]:
    """Clean specific spot with Dreame."""
    client = get_dreame_client(robot_id)
    success = await client.clean_spot(spot_x, spot_y)
    if success:
        return build_success_response(
            operation="clean_dreame_spot",
            summary=f"Dreame {robot_id} cleaning spot at ({spot_x}, {spot_y})",
            result={"robot_id": robot_id, "spot_x": spot_x, "spot_y": spot_y}
        )
    else:
        return build_robotics_error_response(
            error="Failed to clean spot with Dreame",
            robot_type="dreame",
            robot_id=robot_id
        )