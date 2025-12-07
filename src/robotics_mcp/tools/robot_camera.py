"""Robot camera portmanteau tool - Camera and visual feed control.

Provides unified camera control for both physical and virtual robots.
"""

from typing import Any, Dict, Literal, Optional

import structlog
from fastmcp import Client

from ..utils.error_handler import format_error_response, format_success_response, handle_tool_error

logger = structlog.get_logger(__name__)


class RobotCameraTool:
    """Portmanteau tool for robot camera and visual feed control."""

    def __init__(self, mcp: Any, state_manager: Any, mounted_servers: Optional[Dict[str, Any]] = None):
        """Initialize robot camera tool.

        Args:
            mcp: FastMCP server instance.
            state_manager: Robot state manager instance.
            mounted_servers: Dictionary of mounted MCP servers.
        """
        self.mcp = mcp
        self.state_manager = state_manager
        self.mounted_servers = mounted_servers or {}

    def register(self):
        """Register robot camera tool with MCP server."""

        @self.mcp.tool()
        async def robot_camera(
            robot_id: str,
            action: Literal[
                "get_camera_feed",
                "get_virtual_camera",
                "set_camera_angle",
                "capture_image",
                "start_streaming",
                "stop_streaming",
                "get_camera_status",
            ],
            angle_x: Optional[float] = None,
            angle_y: Optional[float] = None,
            output_path: Optional[str] = None,
            stream_url: Optional[str] = None,
        ) -> Dict[str, Any]:
            """Robot camera and visual feed control portmanteau.

            PORTMANTEAU PATTERN: Consolidates camera operations into a single tool.

            SUPPORTED OPERATIONS:
            - get_camera_feed: Get live camera feed (physical Scout camera)
            - get_virtual_camera: Get Unity camera view from robot perspective
            - set_camera_angle: Adjust camera angle
            - capture_image: Capture still image
            - start_streaming: Start video stream
            - stop_streaming: Stop video stream
            - get_camera_status: Get camera status and settings

            Args:
                robot_id: Robot identifier (e.g., "scout_01", "vbot_scout_01").
                action: Camera operation to perform.
                angle_x: Camera angle X (pitch) in degrees for set_camera_angle.
                angle_y: Camera angle Y (yaw) in degrees for set_camera_angle.
                output_path: Output file path for capture_image.
                stream_url: Stream URL for start_streaming.

            Returns:
                Dictionary containing operation result with image data or stream info.

            Examples:
                Get camera feed:
                    result = await robot_camera(robot_id="scout_01", action="get_camera_feed")

                Capture image:
                    result = await robot_camera(
                        robot_id="scout_01",
                        action="capture_image",
                        output_path="C:/Images/scout_capture.jpg"
                    )

                Set camera angle:
                    result = await robot_camera(
                        robot_id="scout_01",
                        action="set_camera_angle",
                        angle_x=10.0,
                        angle_y=0.0
                    )
            """
            try:
                robot = self.state_manager.get_robot(robot_id)
                if not robot:
                    return format_error_response(
                        f"Robot {robot_id} not found",
                        error_type="not_found",
                        robot_id=robot_id,
                        action=action,
                    )

                # Route to appropriate handler
                if robot.is_virtual:
                    return await self._handle_virtual_camera(robot, action, angle_x, angle_y, output_path, stream_url)
                else:
                    return await self._handle_physical_camera(robot, action, angle_x, angle_y, output_path, stream_url)

            except Exception as e:
                return handle_tool_error("robot_camera", e, robot_id=robot_id, action=action)

    async def _handle_virtual_camera(
        self,
        robot: Any,
        action: str,
        angle_x: Optional[float],
        angle_y: Optional[float],
        output_path: Optional[str],
        stream_url: Optional[str],
    ) -> Dict[str, Any]:
        """Handle virtual robot camera."""
        try:
            if robot.platform == "unity" and "unity" in self.mounted_servers:
                async with Client(self.mcp) as client:
                    if action == "get_virtual_camera":
                        # Call Unity RobotCamera.GetCameraFeed()
                        result = await client.call_tool(
                            "execute_unity_method",
                            {
                                "class_name": "RobotCamera",
                                "method_name": "GetCameraFeed",
                                "parameters": {
                                    "robotId": robot.robot_id,
                                },
                            },
                        )
                        return format_success_response(
                            f"Virtual camera feed retrieved for {robot.robot_id}",
                            robot_id=robot.robot_id,
                            action=action,
                            data=result,
                        )

                    elif action == "set_camera_angle":
                        # Call Unity RobotCamera.SetCameraAngle()
                        result = await client.call_tool(
                            "execute_unity_method",
                            {
                                "class_name": "RobotCamera",
                                "method_name": "SetCameraAngle",
                                "parameters": {
                                    "robotId": robot.robot_id,
                                    "angleX": angle_x or 0.0,
                                    "angleY": angle_y or 0.0,
                                },
                            },
                        )
                        return format_success_response(
                            f"Camera angle set for {robot.robot_id}",
                            robot_id=robot.robot_id,
                            action=action,
                            data=result,
                        )

                    elif action == "capture_image":
                        # Call Unity RobotCamera.CaptureImage()
                        result = await client.call_tool(
                            "execute_unity_method",
                            {
                                "class_name": "RobotCamera",
                                "method_name": "CaptureImage",
                                "parameters": {
                                    "robotId": robot.robot_id,
                                    "outputPath": output_path or "",
                                },
                            },
                        )
                        return format_success_response(
                            f"Image captured for {robot.robot_id}",
                            robot_id=robot.robot_id,
                            action=action,
                            data=result,
                        )

                    elif action == "start_streaming":
                        # Call Unity RobotCamera.StartStreaming()
                        result = await client.call_tool(
                            "execute_unity_method",
                            {
                                "class_name": "RobotCamera",
                                "method_name": "StartStreaming",
                                "parameters": {
                                    "robotId": robot.robot_id,
                                    "streamUrl": stream_url or "",
                                },
                            },
                        )
                        return format_success_response(
                            f"Streaming started for {robot.robot_id}",
                            robot_id=robot.robot_id,
                            action=action,
                            data=result,
                        )

                    elif action == "stop_streaming":
                        # Call Unity RobotCamera.StopStreaming()
                        result = await client.call_tool(
                            "execute_unity_method",
                            {
                                "class_name": "RobotCamera",
                                "method_name": "StopStreaming",
                                "parameters": {
                                    "robotId": robot.robot_id,
                                },
                            },
                        )
                        return format_success_response(
                            f"Streaming stopped for {robot.robot_id}",
                            robot_id=robot.robot_id,
                            action=action,
                            data=result,
                        )

                    elif action == "get_camera_status":
                        # Call Unity RobotCamera.GetCameraStatus()
                        result = await client.call_tool(
                            "execute_unity_method",
                            {
                                "class_name": "RobotCamera",
                                "method_name": "GetCameraStatus",
                                "parameters": {
                                    "robotId": robot.robot_id,
                                },
                            },
                        )
                        return format_success_response(
                            f"Camera status retrieved for {robot.robot_id}",
                            robot_id=robot.robot_id,
                            action=action,
                            data=result,
                        )

                    else:
                        # get_camera_feed for virtual = get_virtual_camera
                        return await self._handle_virtual_camera(robot, "get_virtual_camera", None, None, None, None)

            else:
                # Mock camera for testing
                logger.info("Mock camera", robot_id=robot.robot_id, action=action)
                return format_success_response(
                    f"Mock camera: {action} for {robot.robot_id}",
                    robot_id=robot.robot_id,
                    action=action,
                    data={"note": "Mock mode - Unity not available", "mock_image": "base64_encoded_mock_image"},
                )

        except Exception as e:
            return handle_tool_error("_handle_virtual_camera", e, robot_id=robot.robot_id, action=action)

    async def _handle_physical_camera(
        self,
        robot: Any,
        action: str,
        angle_x: Optional[float],
        angle_y: Optional[float],
        output_path: Optional[str],
        stream_url: Optional[str],
    ) -> Dict[str, Any]:
        """Handle physical robot camera."""
        # TODO: Implement ROS-based camera control (sensor_msgs/Image topics)
        logger.info("Physical robot camera", robot_id=robot.robot_id, action=action)
        return format_success_response(
            f"Physical robot camera: {action} for {robot.robot_id}",
            robot_id=robot.robot_id,
            action=action,
            data={"note": "Physical robot camera not yet implemented (requires ROS integration)"},
        )

