"""Philips Hue Bridge Pro client with HomeAware movement detection integration.

HomeAware uses RF signal strength analysis of Zigbee network to detect movement
without cameras, microphones, or PIR sensors - providing privacy-focused motion detection.
"""

import asyncio
import json
from typing import Dict, List, Optional, Any
from dataclasses import dataclass
import structlog
from phue import Bridge

from ..utils.response_builders import build_success_response, build_error_response, build_robotics_error_response

logger = structlog.get_logger(__name__)


@dataclass
class MovementEvent:
    """HomeAware movement detection event."""
    timestamp: float
    location: str
    confidence: float
    duration: Optional[float] = None
    intensity: Optional[float] = None
    sensor_id: Optional[str] = None


@dataclass
class HomeAwareSensor:
    """HomeAware sensor information."""
    sensor_id: str
    name: str
    location: str
    sensitivity: float
    enabled: bool
    last_movement: Optional[float] = None
    movement_count: int = 0


class HueBridgeProClient:
    """Philips Hue Bridge Pro client with HomeAware integration."""

    def __init__(self, bridge_ip: str, api_key: Optional[str] = None):
        """Initialize Hue Bridge Pro client.

        Args:
            bridge_ip: IP address of the Hue Bridge Pro
            api_key: API key for bridge authentication (optional, will be generated if needed)
        """
        self.bridge_ip = bridge_ip
        self.api_key = api_key
        self.bridge: Optional[Bridge] = None
        self._connected = False
        self.homeaware_sensors: Dict[str, HomeAwareSensor] = {}
        self.movement_events: List[MovementEvent] = []
        self._last_update = 0

    async def connect(self) -> bool:
        """Connect to Hue Bridge Pro and initialize HomeAware sensors."""
        try:
            logger.info("Connecting to Hue Bridge Pro", bridge_ip=self.bridge_ip)

            # Create bridge connection (phue handles API key automatically)
            self.bridge = Bridge(self.bridge_ip)

            # First connection attempt - this will prompt for bridge button press if needed
            self.bridge.connect()
            self._connected = True

            # Get API key from bridge object
            self.api_key = getattr(self.bridge, 'username', None)

            # Discover HomeAware sensors
            await self._discover_homeaware_sensors()

            logger.info("Connected to Hue Bridge Pro", sensors=len(self.homeaware_sensors))
            return True

        except Exception as e:
            logger.error("Failed to connect to Hue Bridge Pro",
                        bridge_ip=self.bridge_ip, error=str(e))
            return False

    async def _discover_homeaware_sensors(self):
        """Discover and initialize HomeAware sensors."""
        if not self.bridge:
            return

        try:
            # Get all sensors from bridge
            sensors = self.bridge.get_sensor_objects()

            for sensor_id, sensor_obj in sensors.items():
                sensor_data = sensor_obj.__dict__

                # Check if this is a HomeAware sensor
                if self._is_homeaware_sensor(sensor_data):
                    homeaware_sensor = HomeAwareSensor(
                        sensor_id=str(sensor_id),
                        name=sensor_data.get('name', f'Sensor {sensor_id}'),
                        location=self._extract_location(sensor_data),
                        sensitivity=self._extract_sensitivity(sensor_data),
                        enabled=sensor_data.get('config', {}).get('on', True)
                    )

                    self.homeaware_sensors[str(sensor_id)] = homeaware_sensor
                    logger.info("Discovered HomeAware sensor",
                              sensor_id=sensor_id, name=homeaware_sensor.name)

        except Exception as e:
            logger.error("Failed to discover HomeAware sensors", error=str(e))

    def _is_homeaware_sensor(self, sensor_data: Dict[str, Any]) -> bool:
        """Check if sensor data represents a HomeAware sensor."""
        # HomeAware sensors have specific characteristics
        sensor_type = sensor_data.get('type', '')
        model_id = sensor_data.get('modelid', '')

        # HomeAware sensors typically have specific identifiers
        return ('HomeAware' in sensor_type or
                'homeaware' in model_id.lower() or
                'movement' in sensor_type.lower())

    def _extract_location(self, sensor_data: Dict[str, Any]) -> str:
        """Extract location information from sensor data."""
        # Try different location fields
        location = sensor_data.get('config', {}).get('location', '')
        if not location:
            location = sensor_data.get('name', 'Unknown')
        return location

    def _extract_sensitivity(self, sensor_data: Dict[str, Any]) -> float:
        """Extract sensitivity setting from sensor data."""
        config = sensor_data.get('config', {})
        sensitivity = config.get('sensitivity', 1.0)
        return float(sensitivity)

    async def get_movement_events(self, since_timestamp: Optional[float] = None) -> List[MovementEvent]:
        """Get recent movement detection events.

        Args:
            since_timestamp: Only return events after this timestamp

        Returns:
            List of movement events
        """
        if not self.bridge:
            return []

        try:
            # Get updated sensor data
            sensors = self.bridge.get_sensor_objects()
            events = []

            for sensor_id, sensor_obj in sensors.items():
                sensor_data = sensor_obj.__dict__

                if self._is_homeaware_sensor(sensor_data):
                    # Check for movement detection
                    state = sensor_data.get('state', {})
                    if state.get('presence', False):
                        # Movement detected
                        event = MovementEvent(
                            timestamp=asyncio.get_event_loop().time(),
                            location=self._extract_location(sensor_data),
                            confidence=state.get('presence_confidence', 1.0),
                            sensor_id=str(sensor_id)
                        )
                        events.append(event)

                        # Update sensor tracking
                        if str(sensor_id) in self.homeaware_sensors:
                            sensor = self.homeaware_sensors[str(sensor_id)]
                            sensor.last_movement = event.timestamp
                            sensor.movement_count += 1

            # Filter by timestamp if requested
            if since_timestamp:
                events = [e for e in events if e.timestamp > since_timestamp]

            self.movement_events.extend(events)
            return events

        except Exception as e:
            logger.error("Failed to get movement events", error=str(e))
            return []

    async def get_sensor_status(self) -> Dict[str, Any]:
        """Get comprehensive status of all HomeAware sensors."""
        if not self.bridge:
            return {"error": "Not connected to Hue Bridge"}

        try:
            sensors_status = {}
            sensors = self.bridge.get_sensor_objects()

            for sensor_id, sensor_obj in sensors.items():
                sensor_data = sensor_obj.__dict__

                if self._is_homeaware_sensor(sensor_data):
                    status = {
                        "name": sensor_data.get('name', f'Sensor {sensor_id}'),
                        "location": self._extract_location(sensor_data),
                        "enabled": sensor_data.get('config', {}).get('on', True),
                        "sensitivity": self._extract_sensitivity(sensor_data),
                        "battery": sensor_data.get('config', {}).get('battery', None),
                        "reachable": sensor_data.get('config', {}).get('reachable', True),
                        "presence": sensor_data.get('state', {}).get('presence', False),
                        "last_updated": sensor_data.get('state', {}).get('lastupdated', None)
                    }
                    sensors_status[str(sensor_id)] = status

            return {
                "bridge_connected": True,
                "sensors": sensors_status,
                "total_sensors": len(sensors_status),
                "timestamp": asyncio.get_event_loop().time()
            }

        except Exception as e:
            logger.error("Failed to get sensor status", error=str(e))
            return {"error": str(e), "bridge_connected": False}

    async def configure_sensor(self, sensor_id: str, sensitivity: Optional[float] = None,
                             enabled: Optional[bool] = None) -> bool:
        """Configure HomeAware sensor settings.

        Args:
            sensor_id: Sensor identifier
            sensitivity: Movement detection sensitivity (0.0-1.0)
            enabled: Whether sensor should be enabled

        Returns:
            Success status
        """
        if not self.bridge:
            return False

        try:
            # Update local sensor tracking
            if sensor_id in self.homeaware_sensors:
                sensor = self.homeaware_sensors[sensor_id]
                if sensitivity is not None:
                    sensor.sensitivity = sensitivity
                if enabled is not None:
                    sensor.enabled = enabled

            # Note: Actual bridge configuration would require additional API calls
            # This is a placeholder for future implementation
            logger.info("Sensor configuration updated",
                       sensor_id=sensor_id, sensitivity=sensitivity, enabled=enabled)
            return True

        except Exception as e:
            logger.error("Failed to configure sensor", sensor_id=sensor_id, error=str(e))
            return False

    async def get_movement_zones(self) -> Dict[str, Any]:
        """Get movement detection zones and coverage areas."""
        # HomeAware provides room-level detection rather than specific zones
        # This returns logical zones based on sensor locations
        zones = {}

        for sensor_id, sensor in self.homeaware_sensors.items():
            if sensor.enabled:
                zones[sensor.location] = {
                    "sensor_id": sensor_id,
                    "coverage_area": "room_level",  # HomeAware covers entire rooms
                    "detection_method": "rf_signal_analysis",
                    "last_activity": sensor.last_movement,
                    "activity_count": sensor.movement_count
                }

        return {
            "zones": zones,
            "detection_type": "rf_based_movement",
            "coverage": "room_level",
            "privacy_level": "high"  # No cameras, microphones, or PIR
        }


# Global client instance
_hue_client: Optional[HueBridgeProClient] = None


async def get_hue_client(bridge_ip: str = "192.168.1.100", api_key: Optional[str] = None) -> HueBridgeProClient:
    """Get or create Hue Bridge Pro client instance."""
    global _hue_client

    if _hue_client is None:
        _hue_client = HueBridgeProClient(bridge_ip, api_key)
        await _hue_client.connect()

    return _hue_client


async def hue_get_movement_events(robot_id: str = "hue_bridge") -> Dict[str, Any]:
    """Get HomeAware movement detection events."""
    try:
        # Get client (will be created if needed)
        client = await get_hue_client()

        events = await client.get_movement_events()

        return build_success_response(
            operation="hue_get_movement_events",
            summary=f"Hue HomeAware detected {len(events)} movement events",
            result={
                "robot_id": robot_id,
                "movement_events": [
                    {
                        "timestamp": event.timestamp,
                        "location": event.location,
                        "confidence": event.confidence,
                        "sensor_id": event.sensor_id
                    } for event in events
                ],
                "total_events": len(events)
            }
        )
    except Exception as e:
        return build_robotics_error_response(
            error=f"Failed to get Hue movement events: {str(e)}",
            sensor_type="hue_homeaware",
            robot_id=robot_id
        )


async def hue_get_sensor_status(robot_id: str = "hue_bridge") -> Dict[str, Any]:
    """Get HomeAware sensor status."""
    try:
        client = await get_hue_client()
        status = await client.get_sensor_status()

        return build_success_response(
            operation="hue_get_sensor_status",
            summary=f"Hue HomeAware status retrieved ({status.get('total_sensors', 0)} sensors)",
            result={"robot_id": robot_id, "status": status}
        )
    except Exception as e:
        return build_robotics_error_response(
            error=f"Failed to get Hue sensor status: {str(e)}",
            sensor_type="hue_homeaware",
            robot_id=robot_id
        )


async def hue_get_movement_zones(robot_id: str = "hue_bridge") -> Dict[str, Any]:
    """Get movement detection zones."""
    try:
        client = await get_hue_client()
        zones = await client.get_movement_zones()

        return build_success_response(
            operation="hue_get_movement_zones",
            summary=f"Hue HomeAware zones retrieved ({len(zones.get('zones', {}))} zones)",
            result={"robot_id": robot_id, "zones": zones}
        )
    except Exception as e:
        return build_robotics_error_response(
            error=f"Failed to get Hue movement zones: {str(e)}",
            sensor_type="hue_homeaware",
            robot_id=robot_id
        )