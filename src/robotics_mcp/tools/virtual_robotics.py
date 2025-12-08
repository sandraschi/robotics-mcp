"""Virtual robotics portmanteau tool - Unity/VRChat virtual robot control."""

from typing import Any, Dict, Literal, Optional

import structlog
from fastmcp import Client

from ..utils.error_handler import format_error_response, format_success_response, handle_tool_error

logger = structlog.get_logger(__name__)


class VirtualRoboticsTool:
    """Portmanteau tool for virtual robot operations."""

    def __init__(self, mcp: Any, state_manager: Any, mounted_servers: Optional[Dict[str, Any]] = None):
        """Initialize virtual robotics tool.

        Args:
            mcp: FastMCP server instance.
            state_manager: Robot state manager instance.
            mounted_servers: Dictionary of mounted MCP servers.
        """
        self.mcp = mcp
        self.state_manager = state_manager
        self.mounted_servers = mounted_servers or {}

    def register(self):
        """Register virtual robotics tool with MCP server."""

        @self.mcp.tool()
        async def virtual_robotics(
            action: Literal[
                "spawn_robot",
                "move",
                "get_status",
                "get_lidar",
                "set_scale",
                "load_environment",
                "test_navigation",
                "sync_with_physical",
            ],
            robot_type: Optional[str] = None,
            robot_id: Optional[str] = None,
            position: Optional[Dict[str, float]] = None,
            scale: Optional[float] = None,
            environment: Optional[str] = None,
            environment_path: Optional[str] = None,
            platform: Literal["unity", "vrchat"] = "unity",
            project_path: Optional[str] = None,
            include_colliders: bool = True,
        ) -> Dict[str, Any]:
            """Virtual robot control (Unity/VRChat) using existing MCP servers.

            This portmanteau tool provides comprehensive virtual robot operations,
            leveraging unity3d-mcp, vrchat-mcp, and avatar-mcp for scene control,
            movement, and environment management.

            Args:
                robot_type: Robot model (e.g., "scout", "go2", "g1").
                action: Operation to perform:
                    - "spawn_robot": Spawn robot in Unity/VRChat scene
                    - "move": Control virtual robot movement
                    - "get_status": Get virtual robot state
                    - "get_lidar": Get virtual LiDAR scan (Unity physics raycast)
                    - "set_scale": Scale robot size (for size testing)
                    - "load_environment": Load Marble/Chisel environment
                    - "test_navigation": Test pathfinding
                    - "sync_with_physical": Sync vbot state with physical bot
                robot_id: Robot identifier (auto-generated if not provided).
                position: Spawn position (x, y, z).
                scale: Size multiplier (for size testing).
                environment: Environment name (Marble-generated).
                platform: Target platform ("unity" or "vrchat").
                **kwargs: Additional action-specific parameters.

            Returns:
                Dictionary containing operation result.

            Examples:
                Spawn Scout in Unity:
                    result = await virtual_robotics(
                        robot_type="scout",
                        action="spawn_robot",
                        platform="unity",
                        position={"x": 0, "y": 0, "z": 0}
                    )

                Load Marble environment:
                    result = await virtual_robotics(
                        action="load_environment",
                        environment="stroheckgasse_apartment",
                        platform="unity"
                    )

                Move virtual robot:
                    result = await virtual_robotics(
                        robot_id="vbot_scout_01",
                        action="move",
                        linear=0.2,
                        angular=0.0
                    )
            """
            if action == "spawn_robot":
                if not robot_type:
                    return format_error_response("robot_type is required for spawn_robot action", error_type="validation_error")
                return await self._spawn_robot(robot_type, robot_id, position, scale, platform)
            elif action == "load_environment":
                return await self._load_environment(environment or "", platform, environment_path=environment_path, project_path=project_path, include_colliders=include_colliders)
            elif action == "get_status":
                return await self._get_status(robot_id)
            elif action == "get_lidar":
                return await self._get_lidar(robot_id)
            elif action == "set_scale":
                return await self._set_scale(robot_id, scale)
            elif action == "test_navigation":
                return await self._test_navigation(robot_id, environment)
            elif action == "sync_with_physical":
                return await self._sync_with_physical(robot_id)
            else:
                return format_error_response(
                    f"Unknown action: {action}",
                    error_type="validation_error",
                    details={"valid_actions": ["spawn_robot", "move", "get_status", "get_lidar", "set_scale", "load_environment", "test_navigation", "sync_with_physical"]},
                )

    async def _spawn_robot(
        self,
        robot_type: str,
        robot_id: Optional[str],
        position: Optional[Dict[str, float]],
        scale: Optional[float],
        platform: str,
        **kwargs: Any,
    ) -> Dict[str, Any]:
        """Spawn virtual robot in scene.

        Args:
            robot_type: Type of robot.
            robot_id: Robot identifier.
            position: Spawn position.
            scale: Robot scale.
            platform: Target platform.
            **kwargs: Additional parameters.

        Returns:
            Spawn result.
        """
        if not robot_id:
            robot_id = f"vbot_{robot_type}_01"

        position = position or {"x": 0.0, "y": 0.0, "z": 0.0}
        scale = scale or 1.0

        # Register robot in state manager
        robot = self.state_manager.register_robot(robot_id, robot_type, platform=platform, metadata={
            "position": position,
            "scale": scale,
            "spawned": True,
        })

        logger.info("Spawning virtual robot", robot_id=robot_id, robot_type=robot_type, platform=platform)

        try:
            if platform == "vrchat":
                # Use VRChat OSC to spawn robot in world
                # VRChat worlds can have spawnable objects controlled via OSC
                result = await self._spawn_in_vrchat(robot_id, robot_type, position, scale, **kwargs)
            elif platform == "unity":
                # Use Unity tools to spawn robot
                result = await self._spawn_in_unity(robot_id, robot_type, position, scale, **kwargs)
            else:
                return format_error_response(
                    f"Unknown platform: {platform}",
                    error_type="validation_error",
                    details={"valid_platforms": ["unity", "vrchat"]},
                    robot_id=robot_id,
                )

            robot.connected = True
            self.state_manager.update_robot_status(robot_id, connected=True)

            return format_success_response(
                f"Virtual robot {robot_id} spawned in {platform}",
                robot_id=robot_id,
                action="spawn_robot",
                data={
                    "platform": platform,
                    "position": position,
                    "scale": scale,
                    **result,
                },
            )
        except Exception as e:
            return handle_tool_error("_spawn_robot", e, robot_id=robot_id, context={"platform": platform, "robot_type": robot_type})

    async def _spawn_in_vrchat(
        self,
        robot_id: str,
        robot_type: str,
        position: Dict[str, float],
        scale: float,
        **kwargs: Any,
    ) -> Dict[str, Any]:
        """Spawn robot in VRChat world via OSC.

        Args:
            robot_id: Robot identifier.
            robot_type: Type of robot.
            position: Spawn position.
            scale: Robot scale.
            **kwargs: Additional parameters.

        Returns:
            Spawn result.
        """
        # Use OSC to trigger robot spawn in VRChat world
        # VRChat worlds can have spawnable objects controlled via OSC addresses
        try:
            # Try to use mounted vrchat-mcp or osc-mcp
            if "vrchat" in self.mounted_servers:
                # Use vrchat-mcp to send OSC message
                async with Client(self.mcp) as client:
                    # Send OSC to spawn robot object in VRChat world
                    # Format: /world/spawn/{robot_type} with position/scale
                    await client.call_tool(
                        "vrchat_send_osc_message",
                        address=f"/world/spawn/{robot_type}",
                        args=[position["x"], position["y"], position["z"], scale],
                    )
            elif "osc" in self.mounted_servers:
                # Use osc-mcp directly
                async with Client(self.mcp) as client:
                    await client.call_tool(
                        "osc_send_osc",
                        host="127.0.0.1",
                        port=9000,  # VRChat OSC port
                        address=f"/world/spawn/{robot_type}",
                        values=[position["x"], position["y"], position["z"], scale],
                    )

            logger.info("Robot spawn command sent to VRChat", robot_id=robot_id, robot_type=robot_type)
            return {"method": "osc", "vrchat_ready": True}

        except Exception as e:
            logger.warning("VRChat spawn via MCP failed, using fallback", error=str(e))
            # Fallback: Return success but note it's a mock
            return {"method": "mock", "note": "VRChat MCP not available, using mock spawn"}

    async def _spawn_in_unity(
        self,
        robot_id: str,
        robot_type: str,
        position: Dict[str, float],
        scale: float,
        model_path: Optional[str] = None,
        **kwargs: Any,
    ) -> Dict[str, Any]:
        """Spawn robot in Unity scene.

        Args:
            robot_id: Robot identifier.
            robot_type: Type of robot.
            position: Spawn position.
            scale: Robot scale.
            model_path: Path to 3D model file.
            **kwargs: Additional parameters.

        Returns:
            Spawn result.
        """
        try:
            # Use unity3d-mcp to spawn robot
            if "unity" in self.mounted_servers:
                async with Client(self.mcp) as client:
                    # First, import model if path provided
                    if model_path:
                        # Import model using unity3d-mcp
                        import_result = await client.call_tool(
                            "unity_import_model",
                            model_path=model_path,
                            project_path=kwargs.get("project_path", ""),
                        )
                        logger.info("Model imported", model_path=model_path, result=import_result)

                    # Spawn object in Unity scene
                    # Note: Unity tools may need to be called via execute_method
                    spawn_result = await client.call_tool(
                        "unity_execute_method",
                        class_name="RobotSpawner",
                        method_name="SpawnRobot",
                        parameters={
                            "robotId": robot_id,
                            "robotType": robot_type,
                            "positionX": position["x"],
                            "positionY": position["y"],
                            "positionZ": position["z"],
                            "scale": scale,
                        },
                    )
                    logger.info("Robot spawn command sent to Unity", robot_id=robot_id, result=spawn_result)
                    return {"method": "unity", "unity_ready": True}

            # Fallback: Mock spawn
            logger.warning("Unity MCP not available, using mock spawn")
            return {"method": "mock", "note": "Unity MCP not available, using mock spawn"}

        except Exception as e:
            logger.error("Unity spawn failed", error=str(e))
            return {"method": "mock", "error": str(e)}

    async def _load_environment(self, environment: str, platform: str, environment_path: Optional[str] = None, project_path: Optional[str] = None, include_colliders: bool = True, **kwargs: Any) -> Dict[str, Any]:
        """Load Marble/Chisel environment.

        Args:
            environment: Environment name or path.
            platform: Target platform.
            environment_path: Path to environment file (supports .fbx, .glb, .obj, .ply, .splat).
                            NOTE: .spz files are NOT supported - re-export from Marble as .ply or mesh format.
            project_path: Unity project path (optional, auto-detected if not provided).
            include_colliders: Whether to import collider meshes (default: True).
            **kwargs: Additional parameters.

        Returns:
            Load result.
        """
        logger.info("Loading environment", environment=environment, platform=platform, environment_path=environment_path)

        try:
            if platform == "unity" and "unity" in self.mounted_servers:
                async with Client(self.mcp) as client:
                    # Use environment_path if provided, otherwise use environment name
                    source_path = environment_path or environment
                    
                    # Check for .spz format (not supported)
                    if source_path and source_path.lower().endswith('.spz'):
                        # Suggest using spz_converter tool
                        return format_error_response(
                            ".spz files are not supported by Unity. Use spz_converter tool to check conversion options, or re-export from Marble as .ply (for splats) or .fbx/.glb (for meshes).",
                            error_type="unsupported_format",
                            details={
                                "file": source_path,
                                "supported_formats": [".fbx", ".glb", ".obj", ".ply", ".splat"],
                                "recommendation": "Re-export from Marble as .fbx or .glb for best robotics/navigation support",
                                "alternative": "Use spz_converter tool: await spz_converter(operation='check_spz_support') or spz_converter(operation='convert_spz', spz_path='...')"
                            }
                        )
                    
                    # Use unity3d-mcp import_marble_world tool
                    result = await client.call_tool(
                        "unity_import_marble_world",
                        {
                            "source_path": source_path,  # Supports .fbx, .glb, .obj, .ply, .splat
                            "project_path": project_path or "",
                            "include_colliders": include_colliders,
                        }
                    )
                    logger.info("Environment loaded via Unity MCP", environment=environment, result=result)
                    # Extract result content (CallToolResult has .data attribute with the actual dict)
                    if hasattr(result, 'data'):
                        unity_result = result.data
                    elif hasattr(result, 'content') and result.content:
                        # Try to parse from content
                        first_content = result.content[0] if isinstance(result.content, list) else result.content
                        if hasattr(first_content, 'text'):
                            import json
                            try:
                                unity_result = json.loads(first_content.text)
                            except:
                                unity_result = {"status": "success", "message": first_content.text}
                        else:
                            unity_result = {"status": "success", "data": first_content}
                    else:
                        unity_result = {"status": "success", "message": str(result)}
                    
                    return format_success_response(
                        f"Environment {environment} loaded via Unity",
                        action="load_environment",
                        data={
                            "environment": environment,
                            "platform": platform,
                            "unity_result": unity_result,
                        },
                    )
            else:
                # Fallback
                return format_success_response(
                    f"Environment {environment} loaded (mock - Unity MCP not available)",
                    action="load_environment",
                    data={
                        "environment": environment,
                        "platform": platform,
                        "note": "Unity MCP not available, using mock",
                    },
                )
        except Exception as e:
            return handle_tool_error("_load_environment", e, context={"environment": environment, "platform": platform})

    async def _get_status(self, robot_id: Optional[str]) -> Dict[str, Any]:
        """Get virtual robot status.

        Args:
            robot_id: Robot identifier.

        Returns:
            Robot status.
        """
        if not robot_id:
            return {"status": "error", "message": "robot_id required"}

        try:
            robot = self.state_manager.get_robot(robot_id)
            if not robot:
                return format_error_response(
                    f"Robot {robot_id} not found",
                    error_type="not_found",
                    robot_id=robot_id,
                    action="get_status",
                )

            return format_success_response(
                f"Robot {robot_id} status retrieved",
                robot_id=robot_id,
                action="get_status",
                data={"robot": robot.to_dict()},
            )
        except Exception as e:
            return handle_tool_error("_get_status", e, robot_id=robot_id)

    async def _get_lidar(self, robot_id: Optional[str]) -> Dict[str, Any]:
        """Get virtual LiDAR scan.

        Args:
            robot_id: Robot identifier.

        Returns:
            LiDAR scan data.
        """
        if not robot_id:
            return format_error_response("robot_id required", error_type="validation_error", action="get_lidar")

        try:
            robot = self.state_manager.get_robot(robot_id)
            if not robot or not robot.is_virtual:
                return format_error_response(
                    f"Virtual robot {robot_id} not found",
                    error_type="not_found",
                    robot_id=robot_id,
                    action="get_lidar",
                )
            if robot.platform == "unity" and "unity" in self.mounted_servers:
                # Use Unity physics raycast for virtual LiDAR
                async with Client(self.mcp) as client:
                    # Execute Unity method to perform LiDAR scan
                    result = await client.call_tool(
                        "unity_execute_method",
                        class_name="VirtualLiDAR",
                        method_name="PerformScan",
                        parameters={"robotId": robot_id},
                    )
                    return format_success_response(
                        f"LiDAR scan retrieved for {robot_id}",
                        robot_id=robot_id,
                        action="get_lidar",
                        data={
                            "scan_data": result.get("scan_data", {}),
                            "method": "unity_raycast",
                        },
                    )
            else:
                # Fallback: Return mock scan data
                from ..utils.mock_data import mock_lidar_scan

                scan_data = mock_lidar_scan()
                return format_success_response(
                    f"LiDAR scan retrieved for {robot_id} (mock)",
                    robot_id=robot_id,
                    action="get_lidar",
                    data={
                        "scan_data": scan_data,
                        "method": "mock",
                    },
                )
        except Exception as e:
            return handle_tool_error("_get_lidar", e, robot_id=robot_id)

    async def _set_scale(self, robot_id: Optional[str], scale: Optional[float]) -> Dict[str, Any]:
        """Set robot scale.

        Args:
            robot_id: Robot identifier.
            scale: Scale multiplier.

        Returns:
            Scale result.
        """
        if not robot_id:
            return format_error_response("robot_id required", error_type="validation_error", action="set_scale")

        if scale is None:
            return format_error_response("scale required", error_type="validation_error", robot_id=robot_id, action="set_scale")

        try:
            robot = self.state_manager.get_robot(robot_id)
            if not robot:
                return format_error_response(
                    f"Robot {robot_id} not found",
                    error_type="not_found",
                    robot_id=robot_id,
                    action="set_scale",
                )

            if robot.platform == "unity" and "unity" in self.mounted_servers:
                from fastmcp import Client

                async with Client(self.mcp) as client:
                    result = await client.call_tool(
                        "unity_execute_method",
                        class_name="RobotController",
                        method_name="SetScale",
                        parameters={"robotId": robot_id, "scale": scale},
                    )
                    # Update metadata
                    robot.metadata["scale"] = scale
                    return format_success_response(
                        f"Robot scale set to {scale}",
                        robot_id=robot_id,
                        action="set_scale",
                        data={
                            "scale": scale,
                            "unity_result": result,
                        },
                    )
            else:
                # Update metadata even if Unity not available
                robot.metadata["scale"] = scale
                return format_success_response(
                    f"Robot scale set to {scale} (metadata only)",
                    robot_id=robot_id,
                    action="set_scale",
                    data={"scale": scale, "note": "Unity MCP not available, only metadata updated"},
                )
        except Exception as e:
            return handle_tool_error("_set_scale", e, robot_id=robot_id)

    async def _test_navigation(self, robot_id: Optional[str], environment: Optional[str]) -> Dict[str, Any]:
        """Test navigation in environment.

        Args:
            robot_id: Robot identifier.
            environment: Environment name.

        Returns:
            Navigation test result.
        """
        # TODO: Implement navigation testing
        return {
            "status": "success",
            "message": "Navigation test completed (mock)",
            "robot_id": robot_id,
            "environment": environment,
        }

    async def _sync_with_physical(self, robot_id: Optional[str], **kwargs: Any) -> Dict[str, Any]:
        """Sync virtual robot with physical robot state.

        Args:
            robot_id: Robot identifier.
            **kwargs: Additional parameters.

        Returns:
            Sync result.
        """
        # TODO: Implement sync with physical robot
        return {
            "status": "success",
            "message": "Virtual robot synced with physical (mock)",
            "robot_id": robot_id,
        }

