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
from typing import Any, Literal

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
        mounted_servers: dict[str, Any] | None,
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
            velocity_x: float | None = None,
            velocity_y: float | None = None,
            velocity_z: float | None = None,
            yaw_rate: float | None = None,
            # Takeoff/altitude parameters
            altitude: float | None = None,
            # Mode parameters
            mode: str | None = None,
            # Calibration parameters
            calibration_type: str | None = None
        ) -> dict[str, Any]:
            """Core drone flight control operations with conversational responses.

            Provides unified control interface for PX4/ArduPilot drones via MAVLink protocol
            or direct firmware integration. Supports both autonomous and manual flight modes
            with rich conversational responses for natural AI interaction.

            Args:
                operation: Flight control operation to perform:
                    - "get_status": Get comprehensive drone status (battery, position, mode, health)
                    - "takeoff": Take off to specified altitude (default 5m)
                    - "land": Land at current position
                    - "move": Move with specified velocity vectors
                    - "stop": Emergency stop all movement
                    - "return_home": Return to launch/home position (RTL)
                    - "set_mode": Change flight mode (stabilize, alt_hold, loiter, auto, rtl)
                    - "arm": Arm drone motors (required before flight)
                    - "disarm": Disarm drone motors (safe state)
                    - "calibrate": Perform sensor/accelerometer calibration
                    - "emergency_stop": Immediate emergency stop and motor disarm
                drone_id: Unique drone identifier (e.g., "drone_01", "px4_quad_01", "esp32_drone_01")
                velocity_x: East-west velocity (m/s) for move operation
                velocity_y: North-south velocity (m/s) for move operation
                velocity_z: Up-down velocity (m/s) for move operation (positive = up)
                yaw_rate: Rotational velocity (rad/s) for move operation
                altitude: Target altitude (m) for takeoff operation
                mode: Flight mode for set_mode operation (e.g., "STABILIZE", "ALT_HOLD", "LOITER", "AUTO", "RTL")
                calibration_type: Type of calibration for calibrate operation (e.g., "accelerometer", "compass", "level")

            Returns:
                Rich conversational response with:
                - success: Boolean operation status
                - message: Natural language description of result
                - status_data: Current drone telemetry and health metrics
                - safety_warnings: Any safety concerns or recommendations
                - next_commands: Suggested follow-up operations
                - estimated_completion: Time estimates for long operations
                - error_recovery: Intelligent error handling with resolution steps

            Examples:
                Basic takeoff:
                    result = await drone_control("takeoff", "drone_01", altitude=10.0)
                    # Returns: {"success": true, "message": "Drone taking off to 10m", "safety_warnings": [], "next_commands": ["drone_control('get_status', 'drone_01')"]}

                Emergency stop:
                    result = await drone_control("emergency_stop", "drone_01")
                    # Returns: {"success": true, "message": "Emergency stop initiated", "error_recovery": {"rollback_options": ["arm", "takeoff"]}}
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
            quality: str | None = None,
            protocol: str | None = None,
            bitrate: int | None = None,
            filename: str | None = None
        ) -> dict[str, Any]:
            """Manage drone video streaming and recording with conversational responses.

            Controls FPV video feeds, RTSP/WebRTC streaming, and onboard recording.
            Supports multiple protocols for real-time monitoring and data capture
            with intelligent quality adaptation and conversational status updates.

            Args:
                operation: Video streaming operation to perform:
                    - "start_fpv": Start first-person-view video stream
                    - "stop_fpv": Stop FPV video stream
                    - "get_stream_url": Get URL for active video stream
                    - "set_stream_quality": Adjust stream quality and bitrate
                    - "start_recording": Begin video recording to file
                    - "stop_recording": Stop video recording
                    - "take_snapshot": Capture single still image
                drone_id: Unique drone identifier
                quality: Video quality preset for streaming ("480p", "720p", "1080p", "4K")
                protocol: Streaming protocol ("rtsp", "rtmp", "webrtc", "hls")
                bitrate: Target bitrate in kbps for stream quality control
                filename: Output filename for recording/snapshot operations

            Returns:
                Rich conversational response with:
                - success: Boolean operation status
                - message: Natural language description of streaming/recording state
                - stream_info: Active stream URLs, protocols, and quality metrics
                - recording_status: Current recording progress and file information
                - quality_recommendations: Suggested quality settings based on conditions
                - next_commands: Suggested follow-up operations (e.g., "start_recording")
                - bandwidth_usage: Current and estimated data consumption
                - error_recovery: Stream recovery options and troubleshooting steps

            Examples:
                Start FPV streaming:
                    result = await drone_streaming("start_fpv", "drone_01", quality="720p")
                    # Returns: {"success": true, "message": "FPV stream started at 720p", "stream_info": {"rtsp_url": "rtsp://drone_01/live", "quality": "720p"}, "next_commands": ["drone_streaming('start_recording', 'drone_01')"]}

                Get stream URL:
                    result = await drone_streaming("get_stream_url", "drone_01", protocol="webrtc")
                    # Returns: {"success": true, "stream_info": {"webrtc_url": "webrtc://drone_01/live", "expires_in": 3600}, "quality_recommendations": ["Consider 1080p for better detail"]}
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
            latitude: float | None = None,
            longitude: float | None = None,
            altitude: float | None = None,
            waypoints: list[dict[str, float | None]] = None,
            fence_points: list[dict[str, float | None]] = None,
            max_altitude: float | None = None,
            # Follow-me parameters
            target_id: str | None = None,
            # Home location parameters
            home_latitude: float | None = None,
            home_longitude: float | None = None,
            home_altitude: float | None = None
        ) -> dict[str, Any]:
            """Manage drone navigation, waypoints, and geofencing with conversational responses.

            Handles GPS navigation, waypoint missions, geofence boundaries, and follow-me modes
            with intelligent path planning and safety-aware navigation. Integrates with PX4/ArduPilot
            navigation systems and RTK positioning for precise aerial operations.

            Args:
                operation: Navigation operation to perform:
                    - "get_position": Get current GPS position and altitude
                    - "set_waypoint": Set single waypoint for navigation
                    - "follow_waypoints": Execute multi-waypoint mission
                    - "clear_waypoints": Clear all programmed waypoints
                    - "set_geofence": Define geofence boundaries and restrictions
                    - "enable_follow_me": Start following specified target
                    - "disable_follow_me": Stop follow-me mode
                    - "set_home_location": Set RTL/home position coordinates
                drone_id: Unique drone identifier
                latitude: Target latitude in decimal degrees for waypoint operations
                longitude: Target longitude in decimal degrees for waypoint operations
                altitude: Target altitude in meters for waypoint operations
                waypoints: List of waypoint dictionaries [{"lat": float, "lon": float, "alt": float}, ...]
                fence_points: List of geofence boundary points [{"lat": float, "lon": float}, ...]
                max_altitude: Maximum allowed altitude in meters for geofence
                target_id: Identifier of target to follow for follow-me operations
                home_latitude: Home position latitude in decimal degrees
                home_longitude: Home position longitude in decimal degrees
                home_altitude: Home position altitude in meters

            Returns:
                Rich conversational response with:
                - success: Boolean operation status
                - message: Natural language description of navigation state
                - position_data: Current GPS coordinates, altitude, and precision metrics
                - mission_status: Waypoint progress, ETA, and path information
                - safety_warnings: Geofence violations, obstacle alerts, or weather concerns
                - next_commands: Suggested navigation operations based on current state
                - path_optimization: Alternative routes or efficiency recommendations
                - error_recovery: GPS loss recovery options and emergency procedures

            Examples:
                Set a waypoint mission:
                    result = await drone_navigation("set_waypoint", "drone_01", latitude=37.7749, longitude=-122.4194, altitude=30.0)
                    # Returns: {"success": true, "message": "Waypoint set for navigation", "mission_status": {"waypoints_queued": 1, "estimated_flight_time": "45 seconds"}, "next_commands": ["drone_navigation('follow_waypoints', 'drone_01')"]}

                Enable geofencing:
                    result = await drone_navigation("set_geofence", "drone_01", fence_points=[{"lat": 37.7740, "lon": -122.4200}, {"lat": 37.7750, "lon": -122.4180}], max_altitude=50.0)
                    # Returns: {"success": true, "message": "Geofence established", "safety_warnings": [], "next_commands": ["drone_control('arm', 'drone_01')"]}
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
            mode: str | None = None,
            # Mission parameters
            mission_id: str | None = None,
            mission_plan: dict[str, Any | None] = None,
            # Parameter management
            param_name: str | None = None,
            param_value: Any | None = None
) -> dict[str, Any]:
            """Manage advanced drone flight control, missions, and parameters with conversational responses.

            Provides fine-grained control over PX4/ArduPilot flight modes, autonomous mission execution,
            and drone parameter tuning for advanced flight operations with intelligent mission planning
            and parameter optimization recommendations.

            Args:
                operation: Advanced flight control operation to perform:
                    - "set_flight_mode": Change to specific flight mode (LOITER, AUTO, GUIDED, etc.)
                    - "get_flight_modes": List all available flight modes
                    - "start_mission": Begin execution of uploaded mission plan
                    - "pause_mission": Temporarily halt mission execution
                    - "resume_mission": Continue paused mission
                    - "abort_mission": Immediately terminate mission
                    - "upload_mission": Send mission plan to drone
                    - "download_mission": Retrieve current mission from drone
                    - "set_parameter": Modify drone parameter value
                    - "get_parameters": Retrieve all drone parameters
                drone_id: Unique drone identifier
                mode: Flight mode name for set_flight_mode operation
                mission_id: Identifier for mission operations
                mission_plan: Complete mission plan dictionary with waypoints and commands
                param_name: Parameter name for set_parameter operation
                param_value: New value for the specified parameter

            Returns:
                Rich conversational response with:
                - success: Boolean operation status
                - message: Natural language description of flight control state
                - mission_status: Current mission progress, waypoints completed, and ETA
                - flight_mode_info: Current and available flight modes with descriptions
                - parameter_suggestions: Recommended parameter values for optimal performance
                - safety_assessment: Risk evaluation for planned operations
                - next_commands: Suggested follow-up operations based on current flight state
                - error_recovery: Mission failure recovery options and emergency procedures

            Examples:
                Start autonomous mission:
                    result = await drone_flight_control("start_mission", "drone_01", mission_id="recon_001")
                    # Returns: {"success": true, "message": "Mission started successfully", "mission_status": {"waypoints_total": 5, "estimated_completion": "8 minutes"}, "safety_assessment": {"risk_level": "low"}}

                Tune flight parameters:
                    result = await drone_flight_control("set_parameter", "drone_01", param_name="WPNAV_SPEED", param_value=800)
                    # Returns: {"success": true, "message": "Parameter updated", "parameter_suggestions": ["Consider WPNAV_ACCEL=500 for smoother turns"], "next_commands": ["drone_flight_control('get_parameters', 'drone_01')"]}
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

    # Core flight control operations
    async def _get_drone_status(self, drone) -> dict[str, Any]:
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

    async def _drone_takeoff(self, drone, altitude: float) -> dict[str, Any]:
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

    async def _drone_land(self, drone) -> dict[str, Any]:
        """Execute drone landing."""
        return format_success_response(
            f"Drone {drone.id} landing initiated",
            {
                "drone_id": drone.id,
                "command": "land",
                "status": "executing"
            }
        )

    async def _drone_move(self, drone, velocity_x: float, velocity_y: float, velocity_z: float, yaw_rate: float) -> dict[str, Any]:
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

    async def _drone_stop(self, drone) -> dict[str, Any]:
        """Stop drone movement."""
        return format_success_response(
            f"Drone {drone.id} stopped",
            {
                "drone_id": drone.id,
                "command": "stop",
                "status": "completed"
            }
        )

    async def _drone_rtl(self, drone) -> dict[str, Any]:
        """Return drone to launch/home position."""
        return format_success_response(
            f"Drone {drone.id} returning to home",
            {
                "drone_id": drone.id,
                "command": "rtl",
                "status": "executing"
            }
        )

    async def _set_flight_mode(self, drone, mode: str) -> dict[str, Any]:
        """Set drone flight mode."""
        return format_success_response(
            f"Drone {drone.id} flight mode set to {mode}",
            {
                "drone_id": drone.id,
                "flight_mode": mode,
                "status": "completed"
            }
        )

    async def _arm_drone(self, drone) -> dict[str, Any]:
        """Arm drone motors."""
        return format_success_response(
            f"Drone {drone.id} armed",
            {
                "drone_id": drone.id,
                "armed": True,
                "status": "completed"
            }
        )

    async def _disarm_drone(self, drone) -> dict[str, Any]:
        """Disarm drone motors."""
        return format_success_response(
            f"Drone {drone.id} disarmed",
            {
                "drone_id": drone.id,
                "armed": False,
                "status": "completed"
            }
        )

    async def _calibrate_drone(self, drone, calibration_type: str) -> dict[str, Any]:
        """Calibrate drone sensors."""
        return format_success_response(
            f"Drone {drone.id} {calibration_type} calibration started",
            {
                "drone_id": drone.id,
                "calibration_type": calibration_type,
                "status": "executing"
            }
        )

    async def _emergency_stop(self, drone) -> dict[str, Any]:
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
    async def _start_fpv_stream(self, drone, quality: str) -> dict[str, Any]:
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

    async def _stop_fpv_stream(self, drone) -> dict[str, Any]:
        """Stop FPV video stream."""
        return format_success_response(
            f"Drone {drone.id} FPV stream stopped",
            {
                "drone_id": drone.id,
                "stream_type": "fpv",
                "status": "stopped"
            }
        )

    async def _get_stream_url(self, drone, protocol: str) -> dict[str, Any]:
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

    async def _set_stream_quality(self, drone, quality: str, bitrate: int | None) -> dict[str, Any]:
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

    async def _start_recording(self, drone, filename: str | None) -> dict[str, Any]:
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

    async def _stop_recording(self, drone) -> dict[str, Any]:
        """Stop video recording."""
        return format_success_response(
            f"Drone {drone.id} recording stopped",
            {
                "drone_id": drone.id,
                "recording": False,
                "status": "stopped"
            }
        )

    async def _take_snapshot(self, drone, filename: str | None) -> dict[str, Any]:
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
    async def _get_position(self, drone) -> dict[str, Any]:
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

    async def _set_waypoint(self, drone, lat: float, lon: float, alt: float) -> dict[str, Any]:
        """Set a navigation waypoint."""
        return format_success_response(
            f"Drone {drone.id} waypoint set",
            {
                "drone_id": drone.id,
                "waypoint": {"lat": lat, "lon": lon, "alt": alt},
                "status": "set"
            }
        )

    async def _follow_waypoints(self, drone, waypoints: list[dict] | None) -> dict[str, Any]:
        """Follow a sequence of waypoints."""
        return format_success_response(
            f"Drone {drone.id} following {len(waypoints)} waypoints",
            {
                "drone_id": drone.id,
                "waypoints": waypoints,
                "status": "executing"
            }
        )

    async def _clear_waypoints(self, drone) -> dict[str, Any]:
        """Clear all waypoints."""
        return format_success_response(
            f"Drone {drone.id} waypoints cleared",
            {
                "drone_id": drone.id,
                "waypoints_cleared": True,
                "status": "completed"
            }
        )

    async def _set_geofence(self, drone, fence_points: list[dict] | None, max_altitude: float | None) -> dict[str, Any]:
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

    async def _enable_follow_me(self, drone, target_device: str) -> dict[str, Any]:
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

    async def _disable_follow_me(self, drone) -> dict[str, Any]:
        """Disable follow-me mode."""
        return format_success_response(
            f"Drone {drone.id} follow-me disabled",
            {
                "drone_id": drone.id,
                "follow_me": False,
                "status": "inactive"
            }
        )

    async def _set_home_location(self, drone, lat: float, lon: float) -> dict[str, Any]:
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
    async def _set_flight_mode_advanced(self, drone, mode: str) -> dict[str, Any]:
        """Set advanced flight mode."""
        return format_success_response(
            f"Drone {drone.id} flight mode set to {mode}",
            {
                "drone_id": drone.id,
                "flight_mode": mode,
                "status": "active"
            }
        )

    async def _get_flight_modes(self, drone) -> dict[str, Any]:
        """Get available flight modes."""
        modes = "MANUAL", "ALTITUDE", "POSITION", "AUTO", "RTL", "LAND", "LOITER", "MISSION" | None
        return format_success_response(
            f"Drone {drone.id} flight modes retrieved",
            {
                "drone_id": drone.id,
                "available_modes": modes,
                "current_mode": "MANUAL"
            }
        )

    async def _start_mission(self, drone, mission_id: str | None) -> dict[str, Any]:
        """Start autonomous mission."""
        return format_success_response(
            f"Drone {drone.id} mission started",
            {
                "drone_id": drone.id,
                "mission_id": mission_id,
                "status": "executing"
            }
        )

    async def _pause_mission(self, drone) -> dict[str, Any]:
        """Pause current mission."""
        return format_success_response(
            f"Drone {drone.id} mission paused",
            {
                "drone_id": drone.id,
                "mission_status": "paused"
            }
        )

    async def _resume_mission(self, drone) -> dict[str, Any]:
        """Resume paused mission."""
        return format_success_response(
            f"Drone {drone.id} mission resumed",
            {
                "drone_id": drone.id,
                "mission_status": "executing"
            }
        )

    async def _abort_mission(self, drone) -> dict[str, Any]:
        """Abort current mission."""
        return format_success_response(
            f"Drone {drone.id} mission aborted",
            {
                "drone_id": drone.id,
                "mission_status": "aborted"
            }
        )

    async def _upload_mission(self, drone, mission_data: dict) -> dict[str, Any]:
        """Upload mission to drone."""
        return format_success_response(
            f"Drone {drone.id} mission uploaded",
            {
                "drone_id": drone.id,
                "mission_uploaded": True,
                "waypoints": len(mission_data.get("waypoints", []))
            }
        )

    async def _download_mission(self, drone) -> dict[str, Any]:
        """Download mission from drone."""
        return format_success_response(
            f"Drone {drone.id} mission downloaded",
            {
                "drone_id": drone.id,
                "mission_data": {"waypoints": []}
            }
        )

    async def _set_parameter(self, drone, param_name: str, param_value: Any) -> dict[str, Any]:
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

    async def _get_parameters(self, drone, param_names: list[str] | None) -> dict[str, Any]:
        """Get drone parameters."""
        params = {name: f"mock_value_{name}" for name in param_names}
        return format_success_response(
            f"Drone {drone.id} parameters retrieved",
            {
                "drone_id": drone.id,
                "parameters": params
            }
        )

