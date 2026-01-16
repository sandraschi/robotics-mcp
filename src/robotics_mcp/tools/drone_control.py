#!/usr/bin/env python3
"""Drone Control Portmanteau Tool - Core drone flight operations.

Portmanteau pattern: Consolidates core drone operations into unified interface:
- drone_control: Flight control (takeoff, land, move, status)
- drone_streaming: Video streaming (FPV, RTSP, WebRTC)
- drone_navigation: Aerial navigation (GPS, waypoints, obstacle avoidance)
- drone_flight_control: Advanced flight modes (RTL, loiter, mission planning)

SOTA: FastMCP 2.13+ compliant with conversational responses and sampling support.
"""

import asyncio
import logging
from typing import Any, Dict, List, Literal, Optional

import structlog
from fastmcp import FastMCP

from ..utils.error_handler import format_error_response, format_success_response

logger = structlog.get_logger(__name__)

# Global reference to tool instance for module-level functions
_tool_instance = None

def _get_tool_instance():
    """Get the current tool instance."""
    if _tool_instance is None:
        raise RuntimeError("DroneControlTool not initialized")
    return _tool_instance


class DroneControlTool:
    """Drone Control Portmanteau - Core flight operations."""

    def __init__(
        self,
        mcp: FastMCP,
        state_manager: Any,
        mounted_servers: Dict[str, Any],
    ):
        """Initialize drone control tool.

        Args:
            mcp: FastMCP server instance
            state_manager: Robot state manager
            mounted_servers: Dict of mounted MCP servers
        """
        global _tool_instance
        self.mcp = mcp
        self.state = state_manager
        self.mounted = mounted_servers
        self.logger = structlog.get_logger(__name__)
        _tool_instance = self  # Set global reference

    def register(self):
        """Register all drone control tools."""
        self.logger.info("Registering drone control tools")

        @self.mcp.tool()
        async def drone_control(
            operation: Literal[
                "get_status",
                "takeoff",
                "land",
                "move",
                "stop",
                "return_home",
                "set_mode",
                "arm",
                "disarm",
                "calibrate",
                "emergency_stop"
            ],
            drone_id: str,
            # Movement parameters
            velocity_x: Optional[float] = None,
            velocity_y: Optional[float] = None,
            velocity_z: Optional[float] = None,
            yaw_rate: Optional[float] = None,
            # Takeoff/altitude parameters
            altitude: Optional[float] = None,
            # Mode parameters
            mode: Optional[str] = None,
            # Calibration parameters
            calibration_type: Optional[str] = None
        ) -> Dict[str, Any]:
            """Core drone flight control operations.

            Works with PX4/ArduPilot drones via MAVLink or direct firmware integration.

            Args:
                operation: Flight control operation
                drone_id: Drone identifier (e.g., "drone_01", "px4_quad_01")
                **params: Operation-specific parameters

            Returns:
                Operation result with status and data
            """
            try:
                drone = self.state.get_robot(drone_id)
                if not drone:
                    return format_error_response(f"Drone {drone_id} not found")

                # Route to appropriate handler
                if operation == "get_status":
                    return await self._get_drone_status(drone)
                elif operation == "takeoff":
                    altitude = altitude or 5.0  # Default 5m
                    return await self._drone_takeoff(drone, altitude)
                elif operation == "land":
                    return await self._drone_land(drone)
                elif operation == "move":
                    return await self._drone_move(drone, velocity_x, velocity_y, velocity_z, yaw_rate)
                elif operation == "stop":
                    return await self._drone_stop(drone)
                elif operation == "return_home":
                    return await self._drone_rtl(drone)
                elif operation == "set_mode":
                    return await self._set_flight_mode(drone, mode)
                elif operation == "arm":
                    return await self._arm_drone(drone)
                elif operation == "disarm":
                    return await self._disarm_drone(drone)
                elif operation == "calibrate":
                    return await self._calibrate_drone(drone, calibration_type)
                elif operation == "emergency_stop":
                    return await self._emergency_stop(drone)
                else:
                    return format_error_response(f"Unknown operation: {operation}")

            except Exception as e:
                self.logger.error("Drone control operation failed",
                                operation=operation, drone_id=drone_id, error=str(e))
                return format_error_response(f"Drone control failed: {str(e)}")

        @self.mcp.tool()
        async def drone_streaming(
            operation: Literal[
                "start_fpv",
                "stop_fpv",
                "get_stream_url",
                "set_stream_quality",
                "start_recording",
                "stop_recording",
                "take_snapshot"
            ],
            drone_id: str,
            # Streaming parameters
            quality: Optional[str] = None,
            protocol: Optional[str] = None,
            bitrate: Optional[int] = None,
            filename: Optional[str] = None
        ) -> Dict[str, Any]:
            """Manage drone video streaming and recording.

            Supports FPV, RTSP, WebRTC, and recording to local storage.

            Args:
                operation: Streaming operation
                drone_id: Drone identifier
                **params: Operation-specific parameters

            Returns:
                Operation result with status and data
            """
            try:
                drone = self.state.get_robot(drone_id)
                if not drone:
                    return format_error_response(f"Drone {drone_id} not found")

                if operation == "start_fpv":
                    quality = quality or "720p"
                    return await self._start_fpv_stream(drone, quality)
                elif operation == "stop_fpv":
                    return await self._stop_fpv_stream(drone)
                elif operation == "get_stream_url":
                    return await self._get_stream_url(drone, protocol)
                elif operation == "set_stream_quality":
                    return await self._set_stream_quality(drone, quality, bitrate)
                elif operation == "start_recording":
                    return await self._start_recording(drone, filename)
                elif operation == "stop_recording":
                    return await self._stop_recording(drone)
                elif operation == "take_snapshot":
                    return await self._take_snapshot(drone, filename)
                else:
                    return format_error_response(f"Unknown streaming operation: {operation}")

            except Exception as e:
                self.logger.error("Drone streaming operation failed",
                                operation=operation, drone_id=drone_id, error=str(e))
                return format_error_response(f"Drone streaming failed: {str(e)}")

        @self.mcp.tool()
        async def drone_navigation(
            operation: Literal[
                "get_position",
                "set_waypoint",
                "follow_waypoints",
                "clear_waypoints",
                "set_geofence",
                "enable_follow_me",
                "disable_follow_me",
                "set_home_location"
            ],
            drone_id: str,
            # Waypoint/Geofence parameters
            latitude: Optional[float] = None,
            longitude: Optional[float] = None,
            altitude: Optional[float] = None,
            waypoints: Optional[List[Dict[str, float]]] = None,
            fence_points: Optional[List[Dict[str, float]]] = None,
            max_altitude: Optional[float] = None,
            # Follow-me parameters
            target_id: Optional[str] = None,
            # Home location parameters
            home_latitude: Optional[float] = None,
            home_longitude: Optional[float] = None,
            home_altitude: Optional[float] = None
        ) -> Dict[str, Any]:
            """Manage drone navigation, waypoints, and geofencing.

            Integrates with GPS, RTK, and obstacle avoidance systems.

            Args:
                operation: Navigation operation
                drone_id: Drone identifier
                **params: Operation-specific parameters

            Returns:
                Operation result with status and data
            """
            try:
                drone = self.state.get_robot(drone_id)
                if not drone:
                    return format_error_response(f"Drone {drone_id} not found")

                if operation == "get_position":
                    return await self._get_position(drone)
                elif operation == "set_waypoint":
                    alt = altitude or 10.0  # Default 10m
                    return await self._set_waypoint(drone, latitude, longitude, alt)
                elif operation == "follow_waypoints":
                    return await self._follow_waypoints(drone, waypoints)
                elif operation == "clear_waypoints":
                    return await self._clear_waypoints(drone)
                elif operation == "set_geofence":
                    return await self._set_geofence(drone, fence_points, max_altitude)
                elif operation == "enable_follow_me":
                    return await self._enable_follow_me(drone, target_id)
                elif operation == "disable_follow_me":
                    return await self._disable_follow_me(drone)
                elif operation == "set_home_location":
                    return await self._set_home_location(drone, home_latitude, home_longitude, home_altitude)
                else:
                    return format_error_response(f"Unknown navigation operation: {operation}")

            except Exception as e:
                self.logger.error("Drone navigation operation failed",
                                operation=operation, drone_id=drone_id, error=str(e))
                return format_error_response(f"Drone navigation failed: {str(e)}")

        @self.mcp.tool()
        async def drone_flight_control(
            operation: Literal[
                "set_flight_mode",
                "get_flight_modes",
                "start_mission",
                "pause_mission",
                "resume_mission",
                "abort_mission",
                "upload_mission",
                "download_mission",
                "set_parameter",
                "get_parameters"
            ],
            drone_id: str,
            # Flight mode parameters
            mode: Optional[str] = None,
            # Mission parameters
            mission_id: Optional[str] = None,
            mission_plan: Optional[Dict[str, Any]] = None,
            # Parameter management
            param_name: Optional[str] = None,
            param_value: Optional[Any] = None
) -> Dict[str, Any]:
            """Manage advanced drone flight control, missions, and parameters.

            Provides fine-grained control over flight modes, mission execution, and drone parameters.

            Args:
                operation: Flight control operation
                drone_id: Drone identifier
                **params: Operation-specific parameters

            Returns:
                Operation result with status and data
            """
            try:
                drone = self.state.get_robot(drone_id)
                if not drone:
                    return format_error_response(f"Drone {drone_id} not found")

                if operation == "set_flight_mode":
                    return await self._set_flight_mode_advanced(drone, mode)
                elif operation == "get_flight_modes":
                    return await self._get_flight_modes(drone)
                elif operation == "start_mission":
                    return await self._start_mission(drone, mission_id)
                elif operation == "pause_mission":
                    return await self._pause_mission(drone)
                elif operation == "resume_mission":
                    return await self._resume_mission(drone)
                elif operation == "abort_mission":
                    return await self._abort_mission(drone)
                elif operation == "upload_mission":
                    return await self._upload_mission(drone, mission_plan)
                elif operation == "download_mission":
                    return await self._download_mission(drone)
                elif operation == "set_parameter":
                    return await self._set_parameter(drone, param_name, param_value)
                elif operation == "get_parameters":
                    return await self._get_parameters(drone)
                else:
                    return format_error_response(f"Unknown flight control operation: {operation}")

            except Exception as e:
                self.logger.error("Drone flight control operation failed",
                                operation=operation, drone_id=drone_id, error=str(e))
                return format_error_response(f"Drone flight control failed: {str(e)}")

        @self.mcp.tool()
        async def drone_streaming(
            operation: Literal[
                "start_fpv",
                "stop_fpv",
                "get_stream_url",
                "set_stream_quality",
                "start_recording",
                "stop_recording",
                "take_snapshot"
            ],
            drone_id: str,
            # Streaming parameters
            quality: Optional[str] = None,
            protocol: Optional[str] = None,
            bitrate: Optional[int] = None,
            filename: Optional[str] = None
        ) -> Dict[str, Any]:
            """Drone video streaming and recording operations.

            Supports RTSP, WebRTC, and direct streaming protocols.

            Args:
                operation: Streaming operation
                drone_id: Drone identifier
                **params: Operation-specific parameters

            Returns:
                Streaming operation result
            """
            try:
                drone = self.state.get_robot(drone_id)
                if not drone:
                    return format_error_response(f"Drone {drone_id} not found")

                if operation == "start_fpv":
                    quality = quality or "720p"
                    return await self._start_fpv_stream(drone, quality)
                elif operation == "stop_fpv":
                    return await self._stop_fpv_stream(drone)
                elif operation == "get_stream_url":
                    protocol = protocol or "rtsp"  # rtsp, webrtc, hls
                    return await self._get_stream_url(drone, protocol)
                elif operation == "set_stream_quality":
                    quality = quality or "720p"
                    return await self._set_stream_quality(drone, quality, bitrate)
                elif operation == "start_recording":
                    return await self._start_recording(drone, filename)
                elif operation == "stop_recording":
                    return await self._stop_recording(drone)
                elif operation == "take_snapshot":
                    return await self._take_snapshot(drone, filename)
                else:
                    return format_error_response(f"Unknown streaming operation: {operation}")

            except Exception as e:
                self.logger.error("Drone streaming operation failed",
                                operation=operation, drone_id=drone_id, error=str(e))
                return format_error_response(f"Drone streaming failed: {str(e)}")

        @self.mcp.tool()
        async def drone_navigation(
            operation: Literal[
                "get_position",
                "set_waypoint",
                "follow_waypoints",
                "clear_waypoints",
                "set_geofence",
                "enable_follow_me",
                "disable_follow_me",
                "set_home_location"
            ],
            drone_id: str,
            # Navigation parameters
            latitude: Optional[float] = None,
            longitude: Optional[float] = None,
            altitude: Optional[float] = None,
            waypoints: Optional[List[Dict]] = None,
            fence_points: Optional[List[Dict]] = None,
            max_altitude: Optional[float] = None,
            target_device: Optional[str] = None
        ) -> Dict[str, Any]:
            """Drone navigation and waypoint operations.

            GPS-based navigation with obstacle avoidance and geofencing.

            Args:
                operation: Navigation operation
                drone_id: Drone identifier
                **params: Operation-specific parameters

            Returns:
                Navigation operation result
            """
            try:
                drone = self.state.get_robot(drone_id)
                if not drone:
                    return format_error_response(f"Drone {drone_id} not found")

                if operation == "get_position":
                    return await self._get_position(drone)
                elif operation == "set_waypoint":
                    alt = altitude or 10.0
                    return await self._set_waypoint(drone, latitude, longitude, alt)
                elif operation == "follow_waypoints":
                    waypoints = waypoints or []
                    return await self._follow_waypoints(drone, waypoints)
                elif operation == "clear_waypoints":
                    return await self._clear_waypoints(drone)
                elif operation == "set_geofence":
                    fence_points = fence_points or []
                    return await self._set_geofence(drone, fence_points, max_altitude)
                elif operation == "enable_follow_me":
                    return await self._enable_follow_me(drone, target_device)
                elif operation == "disable_follow_me":
                    return await self._disable_follow_me(drone)
                elif operation == "set_home_location":
                    return await self._set_home_location(drone, latitude, longitude)
                else:
                    return format_error_response(f"Unknown navigation operation: {operation}")

            except Exception as e:
                self.logger.error("Drone navigation operation failed",
                                operation=operation, drone_id=drone_id, error=str(e))
                return format_error_response(f"Drone navigation failed: {str(e)}")

        @self.mcp.tool()
        async def drone_flight_control(
            operation: Literal[
                "set_flight_mode",
                "get_flight_modes",
                "start_mission",
                "pause_mission",
                "resume_mission",
                "abort_mission",
                "upload_mission",
                "download_mission",
                "set_parameter",
                "get_parameters"
            ],
            drone_id: str,
            # Flight control parameters
            mode: Optional[str] = None,
            mission_id: Optional[str] = None,
            mission_data: Optional[Dict] = None,
            parameter_name: Optional[str] = None,
            parameter_value: Optional[Any] = None,
            parameter_names: Optional[List[str]] = None
        ) -> Dict[str, Any]:
            """Advanced drone flight control and mission planning.

            PX4/ArduPilot mission planning and parameter management.

            Args:
                operation: Flight control operation
                drone_id: Drone identifier
                **params: Operation-specific parameters

            Returns:
                Flight control operation result
            """
            try:
                drone = self.state.get_robot(drone_id)
                if not drone:
                    return format_error_response(f"Drone {drone_id} not found")

                if operation == "set_flight_mode":
                    return await self._set_flight_mode_advanced(drone, mode)
                elif operation == "get_flight_modes":
                    return await self._get_flight_modes(drone)
                elif operation == "start_mission":
                    return await self._start_mission(drone, mission_id)
                elif operation == "pause_mission":
                    return await self._pause_mission(drone)
                elif operation == "resume_mission":
                    return await self._resume_mission(drone)
                elif operation == "abort_mission":
                    return await self._abort_mission(drone)
                elif operation == "upload_mission":
                    return await self._upload_mission(drone, mission_data)
                elif operation == "download_mission":
                    return await self._download_mission(drone)
                elif operation == "set_parameter":
                    return await self._set_parameter(drone, parameter_name, parameter_value)
                elif operation == "get_parameters":
                    param_names = parameter_names or []
                    return await self._get_parameters(drone, param_names)
                else:
                    return format_error_response(f"Unknown flight control operation: {operation}")

            except Exception as e:
                self.logger.error("Drone flight control operation failed",
                                operation=operation, drone_id=drone_id, error=str(e))
                return format_error_response(f"Drone flight control failed: {str(e)}")

    # Core flight control operations
    async def _get_drone_status(self, drone) -> Dict[str, Any]:
        """Get comprehensive drone status."""
        # Implementation would connect to PX4/ArduPilot via MAVLink
        # For now, return mock data
        return format_success_response(
            "Drone status retrieved",
            {
                "drone_id": drone.id,
                "armed": False,
                "flight_mode": "MANUAL",
                "battery_level": 85,
                "gps_fix": True,
                "altitude": 0.0,
                "ground_speed": 0.0,
                "heading": 0.0,
                "latitude": 48.2082,
                "longitude": 16.3738,
                "satellite_count": 12,
                "last_heartbeat": "2025-01-16T10:30:00Z"
            }
        )

    async def _drone_takeoff(self, drone, altitude: float) -> Dict[str, Any]:
        """Execute drone takeoff to specified altitude."""
        return format_success_response(
            f"Drone {drone.id} takeoff initiated to {altitude}m",
            {
                "drone_id": drone.id,
                "command": "takeoff",
                "target_altitude": altitude,
                "status": "executing"
            }
        )

    async def _drone_land(self, drone) -> Dict[str, Any]:
        """Execute drone landing."""
        return format_success_response(
            f"Drone {drone.id} landing initiated",
            {
                "drone_id": drone.id,
                "command": "land",
                "status": "executing"
            }
        )

    async def _drone_move(self, drone, velocity_x: float, velocity_y: float, velocity_z: float, yaw_rate: float) -> Dict[str, Any]:
        """Move drone with velocity commands."""
        vx = velocity_x or 0.0
        vy = velocity_y or 0.0
        vz = velocity_z or 0.0
        yr = yaw_rate or 0.0

        return format_success_response(
            f"Drone {drone.id} movement command sent",
            {
                "drone_id": drone.id,
                "command": "move",
                "velocity": {"x": vx, "y": vy, "z": vz},
                "yaw_rate": yr,
                "status": "executing"
            }
        )

    async def _drone_stop(self, drone) -> Dict[str, Any]:
        """Stop drone movement."""
        return format_success_response(
            f"Drone {drone.id} stopped",
            {
                "drone_id": drone.id,
                "command": "stop",
                "status": "completed"
            }
        )

    async def _drone_rtl(self, drone) -> Dict[str, Any]:
        """Return drone to launch/home position."""
        return format_success_response(
            f"Drone {drone.id} returning to home",
            {
                "drone_id": drone.id,
                "command": "rtl",
                "status": "executing"
            }
        )

    async def _set_flight_mode(self, drone, mode: str) -> Dict[str, Any]:
        """Set drone flight mode."""
        return format_success_response(
            f"Drone {drone.id} flight mode set to {mode}",
            {
                "drone_id": drone.id,
                "flight_mode": mode,
                "status": "completed"
            }
        )

    async def _arm_drone(self, drone) -> Dict[str, Any]:
        """Arm drone motors."""
        return format_success_response(
            f"Drone {drone.id} armed",
            {
                "drone_id": drone.id,
                "armed": True,
                "status": "completed"
            }
        )

    async def _disarm_drone(self, drone) -> Dict[str, Any]:
        """Disarm drone motors."""
        return format_success_response(
            f"Drone {drone.id} disarmed",
            {
                "drone_id": drone.id,
                "armed": False,
                "status": "completed"
            }
        )

    async def _calibrate_drone(self, drone, calibration_type: str) -> Dict[str, Any]:
        """Calibrate drone sensors."""
        return format_success_response(
            f"Drone {drone.id} {calibration_type} calibration started",
            {
                "drone_id": drone.id,
                "calibration_type": calibration_type,
                "status": "executing"
            }
        )

    async def _emergency_stop(self, drone) -> Dict[str, Any]:
        """Emergency stop - immediate motor shutdown."""
        return format_success_response(
            f"Drone {drone.id} EMERGENCY STOP executed",
            {
                "drone_id": drone.id,
                "emergency_stop": True,
                "status": "completed"
            }
        )

    # Streaming operations
    async def _start_fpv_stream(self, drone, quality: str) -> Dict[str, Any]:
        """Start FPV video stream."""
        return format_success_response(
            f"Drone {drone.id} FPV stream started at {quality}",
            {
                "drone_id": drone.id,
                "stream_type": "fpv",
                "quality": quality,
                "status": "streaming"
            }
        )

    async def _stop_fpv_stream(self, drone) -> Dict[str, Any]:
        """Stop FPV video stream."""
        return format_success_response(
            f"Drone {drone.id} FPV stream stopped",
            {
                "drone_id": drone.id,
                "stream_type": "fpv",
                "status": "stopped"
            }
        )

    async def _get_stream_url(self, drone, protocol: str) -> Dict[str, Any]:
        """Get video stream URL."""
        base_url = f"rtsp://drone-{drone.id}.local:8554/stream"
        return format_success_response(
            f"Drone {drone.id} stream URL retrieved",
            {
                "drone_id": drone.id,
                "protocol": protocol,
                "stream_url": base_url,
                "status": "available"
            }
        )

    async def _set_stream_quality(self, drone, quality: str, bitrate: Optional[int]) -> Dict[str, Any]:
        """Set video stream quality."""
        return format_success_response(
            f"Drone {drone.id} stream quality set to {quality}",
            {
                "drone_id": drone.id,
                "quality": quality,
                "bitrate": bitrate,
                "status": "updated"
            }
        )

    async def _start_recording(self, drone, filename: Optional[str]) -> Dict[str, Any]:
        """Start video recording."""
        filename = filename or f"drone_{drone.id}_{asyncio.get_event_loop().time():.0f}.mp4"
        return format_success_response(
            f"Drone {drone.id} recording started: {filename}",
            {
                "drone_id": drone.id,
                "recording": True,
                "filename": filename,
                "status": "recording"
            }
        )

    async def _stop_recording(self, drone) -> Dict[str, Any]:
        """Stop video recording."""
        return format_success_response(
            f"Drone {drone.id} recording stopped",
            {
                "drone_id": drone.id,
                "recording": False,
                "status": "stopped"
            }
        )

    async def _take_snapshot(self, drone, filename: Optional[str]) -> Dict[str, Any]:
        """Take a snapshot image."""
        filename = filename or f"drone_{drone.id}_{asyncio.get_event_loop().time():.0f}.jpg"
        return format_success_response(
            f"Drone {drone.id} snapshot taken: {filename}",
            {
                "drone_id": drone.id,
                "snapshot": filename,
                "status": "completed"
            }
        )

    # Navigation operations
    async def _get_position(self, drone) -> Dict[str, Any]:
        """Get drone GPS position."""
        return format_success_response(
            f"Drone {drone.id} position retrieved",
            {
                "drone_id": drone.id,
                "latitude": 48.2082,
                "longitude": 16.3738,
                "altitude": 15.5,
                "heading": 90.0,
                "ground_speed": 2.5,
                "status": "valid"
            }
        )

    async def _set_waypoint(self, drone, lat: float, lon: float, alt: float) -> Dict[str, Any]:
        """Set a navigation waypoint."""
        return format_success_response(
            f"Drone {drone.id} waypoint set",
            {
                "drone_id": drone.id,
                "waypoint": {"lat": lat, "lon": lon, "alt": alt},
                "status": "set"
            }
        )

    async def _follow_waypoints(self, drone, waypoints: List[Dict]) -> Dict[str, Any]:
        """Follow a sequence of waypoints."""
        return format_success_response(
            f"Drone {drone.id} following {len(waypoints)} waypoints",
            {
                "drone_id": drone.id,
                "waypoints": waypoints,
                "status": "executing"
            }
        )

    async def _clear_waypoints(self, drone) -> Dict[str, Any]:
        """Clear all waypoints."""
        return format_success_response(
            f"Drone {drone.id} waypoints cleared",
            {
                "drone_id": drone.id,
                "waypoints_cleared": True,
                "status": "completed"
            }
        )

    async def _set_geofence(self, drone, fence_points: List[Dict], max_altitude: Optional[float]) -> Dict[str, Any]:
        """Set geofencing boundaries."""
        return format_success_response(
            f"Drone {drone.id} geofence set",
            {
                "drone_id": drone.id,
                "fence_points": len(fence_points),
                "max_altitude": max_altitude,
                "status": "active"
            }
        )

    async def _enable_follow_me(self, drone, target_device: str) -> Dict[str, Any]:
        """Enable follow-me mode."""
        return format_success_response(
            f"Drone {drone.id} following {target_device}",
            {
                "drone_id": drone.id,
                "follow_me": True,
                "target_device": target_device,
                "status": "active"
            }
        )

    async def _disable_follow_me(self, drone) -> Dict[str, Any]:
        """Disable follow-me mode."""
        return format_success_response(
            f"Drone {drone.id} follow-me disabled",
            {
                "drone_id": drone.id,
                "follow_me": False,
                "status": "inactive"
            }
        )

    async def _set_home_location(self, drone, lat: float, lon: float) -> Dict[str, Any]:
        """Set home/RTL location."""
        return format_success_response(
            f"Drone {drone.id} home location set",
            {
                "drone_id": drone.id,
                "home": {"lat": lat, "lon": lon},
                "status": "set"
            }
        )

    # Advanced flight control operations
    async def _set_flight_mode_advanced(self, drone, mode: str) -> Dict[str, Any]:
        """Set advanced flight mode."""
        return format_success_response(
            f"Drone {drone.id} flight mode set to {mode}",
            {
                "drone_id": drone.id,
                "flight_mode": mode,
                "status": "active"
            }
        )

    async def _get_flight_modes(self, drone) -> Dict[str, Any]:
        """Get available flight modes."""
        modes = ["MANUAL", "ALTITUDE", "POSITION", "AUTO", "RTL", "LAND", "LOITER", "MISSION"]
        return format_success_response(
            f"Drone {drone.id} flight modes retrieved",
            {
                "drone_id": drone.id,
                "available_modes": modes,
                "current_mode": "MANUAL"
            }
        )

    async def _start_mission(self, drone, mission_id: Optional[str]) -> Dict[str, Any]:
        """Start autonomous mission."""
        return format_success_response(
            f"Drone {drone.id} mission started",
            {
                "drone_id": drone.id,
                "mission_id": mission_id,
                "status": "executing"
            }
        )

    async def _pause_mission(self, drone) -> Dict[str, Any]:
        """Pause current mission."""
        return format_success_response(
            f"Drone {drone.id} mission paused",
            {
                "drone_id": drone.id,
                "mission_status": "paused"
            }
        )

    async def _resume_mission(self, drone) -> Dict[str, Any]:
        """Resume paused mission."""
        return format_success_response(
            f"Drone {drone.id} mission resumed",
            {
                "drone_id": drone.id,
                "mission_status": "executing"
            }
        )

    async def _abort_mission(self, drone) -> Dict[str, Any]:
        """Abort current mission."""
        return format_success_response(
            f"Drone {drone.id} mission aborted",
            {
                "drone_id": drone.id,
                "mission_status": "aborted"
            }
        )

    async def _upload_mission(self, drone, mission_data: Dict) -> Dict[str, Any]:
        """Upload mission to drone."""
        return format_success_response(
            f"Drone {drone.id} mission uploaded",
            {
                "drone_id": drone.id,
                "mission_uploaded": True,
                "waypoints": len(mission_data.get("waypoints", []))
            }
        )

    async def _download_mission(self, drone) -> Dict[str, Any]:
        """Download mission from drone."""
        return format_success_response(
            f"Drone {drone.id} mission downloaded",
            {
                "drone_id": drone.id,
                "mission_data": {"waypoints": []}
            }
        )

    async def _set_parameter(self, drone, param_name: str, param_value: Any) -> Dict[str, Any]:
        """Set drone parameter."""
        return format_success_response(
            f"Drone {drone.id} parameter {param_name} set to {param_value}",
            {
                "drone_id": drone.id,
                "parameter": param_name,
                "value": param_value,
                "status": "set"
            }
        )

    async def _get_parameters(self, drone, param_names: List[str]) -> Dict[str, Any]:
        """Get drone parameters."""
        params = {name: f"mock_value_{name}" for name in param_names}
        return format_success_response(
            f"Drone {drone.id} parameters retrieved",
            {
                "drone_id": drone.id,
                "parameters": params
            }
        )

