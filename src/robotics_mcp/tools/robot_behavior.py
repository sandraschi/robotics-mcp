"""Robot behavior portmanteau tool - Animation, camera, navigation, and manipulation control.

Consolidates animation, camera, navigation, and manipulation (arms/grippers) operations into a single unified tool.
"""

from typing import Any, Dict, List, Literal, Optional

import structlog

from ..utils.error_handler import format_error_response, format_success_response, handle_tool_error
from ..utils.mcp_client_helper import call_mounted_server_tool

logger = structlog.get_logger(__name__)


class RobotBehaviorTool:
    """Portmanteau tool for robot behavior: animation, camera, and navigation."""

    def __init__(self, mcp: Any, state_manager: Any, mounted_servers: Optional[Dict[str, Any]] = None):
        """Initialize robot behavior tool.

        Args:
            mcp: FastMCP server instance.
            state_manager: Robot state manager instance.
            mounted_servers: Dictionary of mounted MCP servers.
        """
        self.mcp = mcp
        self.state_manager = state_manager
        self.mounted_servers = mounted_servers or {}

    def register(self):
        """Register robot behavior tool with MCP server."""

        @self.mcp.tool()
        async def robot_behavior(
            robot_id: str,
            category: Literal["animation", "camera", "navigation", "manipulation"],
            action: str,
            # Animation parameters
            wheel_speeds: Optional[Dict[str, float]] = None,
            animation_name: Optional[str] = None,
            pose: Optional[str] = None,
            animation_speed: Optional[float] = None,
            loop: bool = False,
            # Camera parameters
            angle_x: Optional[float] = None,
            angle_y: Optional[float] = None,
            output_path: Optional[str] = None,
            stream_url: Optional[str] = None,
            # Navigation parameters
            start_position: Optional[Dict[str, float]] = None,
            goal_position: Optional[Dict[str, float]] = None,
            waypoint: Optional[Dict[str, float]] = None,
            obstacle_position: Optional[Dict[str, float]] = None,
            path_id: Optional[str] = None,
            # Manipulation parameters
            joint_positions: Optional[Dict[str, float]] = None,
            end_effector_pose: Optional[Dict[str, Any]] = None,
            gripper_position: Optional[float] = None,
            arm_id: Optional[str] = None,
            force_limit: Optional[float] = None,
            manipulation_speed: Optional[float] = None,
        ) -> Dict[str, Any]:
            """Robot behavior control portmanteau - Animation, camera, navigation, and manipulation.

            PORTMANTEAU PATTERN: Consolidates animation, camera, navigation, and manipulation operations
            into a single unified tool. This reduces tool explosion while maintaining
            full functionality across all behavior categories.

            CATEGORIES:
            - animation: Animation and pose control
            - camera: Camera feed and visual control
            - navigation: Path planning and navigation
            - manipulation: Arm and gripper control

            ANIMATION ACTIONS:
            - animate_wheels: Rotate wheels during movement (Scout mecanum wheels)
            - animate_movement: Play movement animations (walk, turn, etc.)
            - set_pose: Set robot pose (sitting, standing, etc. for Unitree)
            - play_animation: Play custom animations
            - stop_animation: Stop current animation
            - get_animation_state: Get current animation state

            CAMERA ACTIONS:
            - get_camera_feed: Get live camera feed (physical Scout camera)
            - get_virtual_camera: Get Unity camera view from robot perspective
            - set_camera_angle: Adjust camera angle
            - capture_image: Capture still image
            - start_streaming: Start video stream
            - stop_streaming: Stop video stream
            - get_camera_status: Get camera status and settings

            NAVIGATION ACTIONS:
            - plan_path: Plan path from A to B (A* or RRT)
            - follow_path: Execute planned path
            - set_waypoint: Set navigation waypoint
            - clear_waypoints: Clear waypoint list
            - get_path_status: Check path execution status
            - avoid_obstacle: Dynamic obstacle avoidance
            - get_current_path: Get current path being followed

            MANIPULATION ACTIONS:
            - move_arm: Move arm to target joint positions or end-effector pose
            - set_joint_positions: Set individual joint positions (dict of joint_name: angle)
            - set_end_effector_pose: Move end-effector to target pose (position + orientation)
            - get_arm_state: Get current arm joint positions and end-effector pose
            - open_gripper: Open gripper fully
            - close_gripper: Close gripper fully
            - set_gripper_position: Set gripper position (0.0 = open, 1.0 = closed)
            - get_gripper_state: Get current gripper position and force feedback
            - move_to_pose: Move arm to target pose with IK (inverse kinematics)
            - home_arm: Return arm to home/rest position

            Args:
                robot_id: Robot identifier (e.g., "scout_01", "vbot_scout_01").
                category: Behavior category: "animation", "camera", or "navigation".
                action: Action to perform (see category-specific actions above).
                # Animation parameters (used when category="animation")
                wheel_speeds: Wheel speeds for animate_wheels.
                animation_name: Animation name for play_animation.
                pose: Pose name for set_pose.
                animation_speed: Animation speed multiplier.
                loop: Whether to loop animation.
                # Camera parameters (used when category="camera")
                angle_x: Camera angle X (pitch) in degrees.
                angle_y: Camera angle Y (yaw) in degrees.
                output_path: Output file path for capture_image.
                stream_url: Stream URL for start_streaming.
                # Navigation parameters (used when category="navigation")
                start_position: Start position (x, y, z) for plan_path.
                goal_position: Goal position (x, y, z) for plan_path.
                waypoint: Waypoint position (x, y, z) for set_waypoint.
                obstacle_position: Obstacle position (x, y, z) for avoid_obstacle.
                path_id: Path identifier for follow_path or get_path_status.
                # Manipulation parameters (used when category="manipulation")
                joint_positions: Joint positions dict (e.g., {"shoulder": 45.0, "elbow": 90.0}).
                end_effector_pose: End-effector pose dict with position and orientation.
                gripper_position: Gripper position (0.0 = open, 1.0 = closed).
                arm_id: Arm identifier for multi-arm robots (e.g., "left", "right").
                force_limit: Maximum force/torque limit for movement.
                manipulation_speed: Movement speed (0.0-1.0) for arm/gripper motion.

            Returns:
                Dictionary containing operation result.

            Examples:
                # Animation
                result = await robot_behavior(
                    robot_id="scout_01",
                    category="animation",
                    action="play_animation",
                    animation_name="walk",
                    animation_speed=1.0,
                    loop=True
                )

                # Camera
                result = await robot_behavior(
                    robot_id="scout_01",
                    category="camera",
                    action="capture_image",
                    output_path="C:/Images/capture.jpg"
                )

                # Navigation
                result = await robot_behavior(
                    robot_id="scout_01",
                    category="navigation",
                    action="plan_path",
                    start_position={"x": 0, "y": 0, "z": 0},
                    goal_position={"x": 5, "y": 0, "z": 0}
                )

                # Manipulation - Move arm
                result = await robot_behavior(
                    robot_id="g1_01",
                    category="manipulation",
                    action="move_arm",
                    joint_positions={"shoulder": 45.0, "elbow": 90.0, "wrist": 0.0}
                )

                # Manipulation - Open gripper
                result = await robot_behavior(
                    robot_id="g1_01",
                    category="manipulation",
                    action="open_gripper"
                )
            """
            try:
                robot = self.state_manager.get_robot(robot_id)
                if not robot:
                    return format_error_response(
                        f"Robot {robot_id} not found",
                        error_type="not_found",
                        robot_id=robot_id,
                        category=category,
                        action=action,
                    )

                # Route to category handler
                if category == "animation":
                    return await self._handle_animation(
                        robot, action, wheel_speeds, animation_name, pose, speed, loop
                    )
                elif category == "camera":
                    return await self._handle_camera(robot, action, angle_x, angle_y, output_path, stream_url)
                elif category == "navigation":
                    return await self._handle_navigation(
                        robot, action, start_position, goal_position, waypoint, obstacle_position, path_id
                    )
                elif category == "manipulation":
                    return await self._handle_manipulation(
                        robot, action, joint_positions, end_effector_pose, gripper_position, arm_id, force_limit, manipulation_speed
                    )
                else:
                    return format_error_response(
                        f"Unknown category: {category}",
                        error_type="validation_error",
                        valid_categories=["animation", "camera", "navigation", "manipulation"],
                    )

            except Exception as e:
                return handle_tool_error("robot_behavior", e, robot_id=robot_id, category=category, action=action)

    async def _handle_animation(
        self,
        robot: Any,
        action: str,
        wheel_speeds: Optional[Dict[str, float]],
        animation_name: Optional[str],
        pose: Optional[str],
        animation_speed: Optional[float],
        loop: bool,
    ) -> Dict[str, Any]:
        """Handle animation operations."""
        if robot.is_virtual:
            return await self._handle_virtual_animation(robot, action, wheel_speeds, animation_name, pose, animation_speed, loop)
        else:
            return await self._handle_physical_animation(robot, action, wheel_speeds, animation_name, pose, animation_speed, loop)

    async def _handle_camera(
        self,
        robot: Any,
        action: str,
        angle_x: Optional[float],
        angle_y: Optional[float],
        output_path: Optional[str],
        stream_url: Optional[str],
    ) -> Dict[str, Any]:
        """Handle camera operations."""
        if robot.is_virtual:
            return await self._handle_virtual_camera(robot, action, angle_x, angle_y, output_path, stream_url)
        else:
            return await self._handle_physical_camera(robot, action, angle_x, angle_y, output_path, stream_url)

    async def _handle_navigation(
        self,
        robot: Any,
        action: str,
        start_position: Optional[Dict[str, float]],
        goal_position: Optional[Dict[str, float]],
        waypoint: Optional[Dict[str, float]],
        obstacle_position: Optional[Dict[str, float]],
        path_id: Optional[str],
    ) -> Dict[str, Any]:
        """Handle navigation operations."""
        if robot.is_virtual:
            return await self._handle_virtual_navigation(
                robot, action, start_position, goal_position, waypoint, obstacle_position, path_id
            )
        else:
            return await self._handle_physical_navigation(
                robot, action, start_position, goal_position, waypoint, obstacle_position, path_id
            )

    async def _handle_manipulation(
        self,
        robot: Any,
        action: str,
        joint_positions: Optional[Dict[str, float]],
        end_effector_pose: Optional[Dict[str, Any]],
        gripper_position: Optional[float],
        arm_id: Optional[str],
        force_limit: Optional[float],
        manipulation_speed: Optional[float],
    ) -> Dict[str, Any]:
        """Handle manipulation operations (arms and grippers)."""
        if robot.is_virtual:
            return await self._handle_virtual_manipulation(
                robot, action, joint_positions, end_effector_pose, gripper_position, arm_id, force_limit, manipulation_speed
            )
        else:
            return await self._handle_physical_manipulation(
                robot, action, joint_positions, end_effector_pose, gripper_position, arm_id, force_limit, manipulation_speed
            )

    # Virtual animation handlers (from robot_animation.py)
    async def _handle_virtual_animation(
        self,
        robot: Any,
        action: str,
        wheel_speeds: Optional[Dict[str, float]],
        animation_name: Optional[str],
        pose: Optional[str],
        animation_speed: Optional[float],
        loop: bool,
    ) -> Dict[str, Any]:
        """Handle virtual robot animation."""
        try:
            if robot.platform == "unity" and "unity" in self.mounted_servers:
                method_map = {
                    "animate_wheels": ("RobotAnimator", "AnimateWheels", {"robotId": robot.robot_id, "wheelSpeeds": wheel_speeds or {}}),
                    "animate_movement": ("RobotAnimator", "AnimateMovement", {"robotId": robot.robot_id, "animationName": animation_name or "walk", "speed": animation_speed or 1.0, "loop": loop}),
                    "set_pose": ("RobotAnimator", "SetPose", {"robotId": robot.robot_id, "pose": pose or "idle"}),
                    "play_animation": ("RobotAnimator", "PlayAnimation", {"robotId": robot.robot_id, "animationName": animation_name or "idle", "speed": animation_speed or 1.0, "loop": loop}),
                    "stop_animation": ("RobotAnimator", "StopAnimation", {"robotId": robot.robot_id}),
                    "get_animation_state": ("RobotAnimator", "GetAnimationState", {"robotId": robot.robot_id}),
                }

                if action not in method_map:
                    return format_error_response(f"Unknown animation action: {action}", error_type="validation_error")

                class_name, method_name, params = method_map[action]
                result = await call_mounted_server_tool(
                    self.mounted_servers,
                    "unity",
                    "execute_unity_method",
                    {
                        "class_name": class_name,
                        "method_name": method_name,
                        "parameters": params,
                    },
                )

                return format_success_response(
                    f"Animation {action} executed for {robot.robot_id}",
                    robot_id=robot.robot_id,
                    category="animation",
                    action=action,
                    data=result,
                )
            else:
                logger.info("Mock animation", robot_id=robot.robot_id, action=action)
                return format_success_response(
                    f"Mock animation: {action} for {robot.robot_id}",
                    robot_id=robot.robot_id,
                    category="animation",
                    action=action,
                    data={"note": "Mock mode - Unity/VRChat not available"},
                )

        except Exception as e:
            return handle_tool_error("_handle_virtual_animation", e, robot_id=robot.robot_id, action=action)

    async def _handle_physical_animation(
        self,
        robot: Any,
        action: str,
        wheel_speeds: Optional[Dict[str, float]],
        animation_name: Optional[str],
        pose: Optional[str],
        animation_speed: Optional[float],
        loop: bool,
    ) -> Dict[str, Any]:
        """Handle physical robot animation."""
        logger.info("Physical robot animation", robot_id=robot.robot_id, action=action)
        return format_success_response(
            f"Physical robot animation: {action} for {robot.robot_id}",
            robot_id=robot.robot_id,
            category="animation",
            action=action,
            data={"note": "Physical robot animation not yet implemented (requires ROS integration)"},
        )

    # Virtual camera handlers (from robot_camera.py)
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
                method_map = {
                    "get_camera_feed": ("RobotCamera", "GetCameraFeed", {"robotId": robot.robot_id}),
                    "get_virtual_camera": ("RobotCamera", "GetCameraFeed", {"robotId": robot.robot_id}),
                    "set_camera_angle": ("RobotCamera", "SetCameraAngle", {"robotId": robot.robot_id, "angleX": angle_x or 0.0, "angleY": angle_y or 0.0}),
                    "capture_image": ("RobotCamera", "CaptureImage", {"robotId": robot.robot_id, "outputPath": output_path or ""}),
                    "start_streaming": ("RobotCamera", "StartStreaming", {"robotId": robot.robot_id, "streamUrl": stream_url or ""}),
                    "stop_streaming": ("RobotCamera", "StopStreaming", {"robotId": robot.robot_id}),
                    "get_camera_status": ("RobotCamera", "GetCameraStatus", {"robotId": robot.robot_id}),
                }

                if action not in method_map:
                    return format_error_response(f"Unknown camera action: {action}", error_type="validation_error")

                class_name, method_name, params = method_map[action]
                result = await call_mounted_server_tool(
                    self.mounted_servers,
                    "unity",
                    "execute_unity_method",
                    {
                        "class_name": class_name,
                        "method_name": method_name,
                        "parameters": params,
                    },
                )

                return format_success_response(
                    f"Camera {action} executed for {robot.robot_id}",
                    robot_id=robot.robot_id,
                    category="camera",
                    action=action,
                    data=result,
                )
            else:
                logger.info("Mock camera", robot_id=robot.robot_id, action=action)
                return format_success_response(
                    f"Mock camera: {action} for {robot.robot_id}",
                    robot_id=robot.robot_id,
                    category="camera",
                    action=action,
                    data={"note": "Mock mode - Unity not available"},
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
        logger.info("Physical robot camera", robot_id=robot.robot_id, action=action)
        return format_success_response(
            f"Physical robot camera: {action} for {robot.robot_id}",
            robot_id=robot.robot_id,
            category="camera",
            action=action,
            data={"note": "Physical robot camera not yet implemented (requires ROS integration)"},
        )

    # Virtual navigation handlers (from robot_navigation.py)
    async def _handle_virtual_navigation(
        self,
        robot: Any,
        action: str,
        start_position: Optional[Dict[str, float]],
        goal_position: Optional[Dict[str, float]],
        waypoint: Optional[Dict[str, float]],
        obstacle_position: Optional[Dict[str, float]],
        path_id: Optional[str],
    ) -> Dict[str, Any]:
        """Handle virtual robot navigation."""
        try:
            if robot.platform == "unity" and "unity" in self.mounted_servers:
                method_map = {
                    "plan_path": ("RobotNavigator", "PlanPath", {"robotId": robot.robot_id, "startPosition": start_position or {}, "goalPosition": goal_position or {}}),
                    "follow_path": ("RobotNavigator", "FollowPath", {"robotId": robot.robot_id, "pathId": path_id or ""}),
                    "set_waypoint": ("RobotNavigator", "SetWaypoint", {"robotId": robot.robot_id, "waypoint": waypoint or {}}),
                    "clear_waypoints": ("RobotNavigator", "ClearWaypoints", {"robotId": robot.robot_id}),
                    "get_path_status": ("RobotNavigator", "GetPathStatus", {"robotId": robot.robot_id, "pathId": path_id or ""}),
                    "avoid_obstacle": ("RobotNavigator", "AvoidObstacle", {"robotId": robot.robot_id, "obstaclePosition": obstacle_position or {}}),
                    "get_current_path": ("RobotNavigator", "GetCurrentPath", {"robotId": robot.robot_id}),
                }

                if action not in method_map:
                    return format_error_response(f"Unknown navigation action: {action}", error_type="validation_error")

                class_name, method_name, params = method_map[action]
                result = await call_mounted_server_tool(
                    self.mounted_servers,
                    "unity",
                    "execute_unity_method",
                    {
                        "class_name": class_name,
                        "method_name": method_name,
                        "parameters": params,
                    },
                )

                return format_success_response(
                    f"Navigation {action} executed for {robot.robot_id}",
                    robot_id=robot.robot_id,
                    category="navigation",
                    action=action,
                    data=result,
                )
            else:
                logger.info("Mock navigation", robot_id=robot.robot_id, action=action)
                mock_path = {
                    "path_id": path_id or "mock_path_123",
                    "waypoints": [
                        start_position or {"x": 0, "y": 0, "z": 0},
                        goal_position or {"x": 5, "y": 0, "z": 0},
                    ],
                    "status": "planned",
                }
                return format_success_response(
                    f"Mock navigation: {action} for {robot.robot_id}",
                    robot_id=robot.robot_id,
                    category="navigation",
                    action=action,
                    data={"note": "Mock mode - Unity not available", "path": mock_path},
                )

        except Exception as e:
            return handle_tool_error("_handle_virtual_navigation", e, robot_id=robot.robot_id, action=action)

    async def _handle_physical_navigation(
        self,
        robot: Any,
        action: str,
        start_position: Optional[Dict[str, float]],
        goal_position: Optional[Dict[str, float]],
        waypoint: Optional[Dict[str, float]],
        obstacle_position: Optional[Dict[str, float]],
        path_id: Optional[str],
    ) -> Dict[str, Any]:
        """Handle physical robot navigation."""
        logger.info("Physical robot navigation", robot_id=robot.robot_id, action=action)
        return format_success_response(
            f"Physical robot navigation: {action} for {robot.robot_id}",
            robot_id=robot.robot_id,
            category="navigation",
            action=action,
            data={"note": "Physical robot navigation not yet implemented (requires ROS navigation stack)"},
        )

    # Virtual manipulation handlers
    async def _handle_virtual_manipulation(
        self,
        robot: Any,
        action: str,
        joint_positions: Optional[Dict[str, float]],
        end_effector_pose: Optional[Dict[str, Any]],
        gripper_position: Optional[float],
        arm_id: Optional[str],
        force_limit: Optional[float],
        manipulation_speed: Optional[float],
    ) -> Dict[str, Any]:
        """Handle virtual robot manipulation (arms and grippers)."""
        try:
            if robot.platform == "unity" and "unity" in self.mounted_servers:
                method_map = {
                    "move_arm": (
                        "RobotManipulator",
                        "MoveArm",
                        {
                            "robotId": robot.robot_id,
                            "jointPositions": joint_positions or {},
                            "endEffectorPose": end_effector_pose or {},
                            "armId": arm_id or "default",
                            "speed": manipulation_speed or 0.5,
                        },
                    ),
                    "set_joint_positions": (
                        "RobotManipulator",
                        "SetJointPositions",
                        {"robotId": robot.robot_id, "jointPositions": joint_positions or {}, "armId": arm_id or "default"},
                    ),
                    "set_end_effector_pose": (
                        "RobotManipulator",
                        "SetEndEffectorPose",
                        {
                            "robotId": robot.robot_id,
                            "pose": end_effector_pose or {},
                            "armId": arm_id or "default",
                            "speed": manipulation_speed or 0.5,
                        },
                    ),
                    "get_arm_state": ("RobotManipulator", "GetArmState", {"robotId": robot.robot_id, "armId": arm_id or "default"}),
                    "open_gripper": ("RobotGripper", "OpenGripper", {"robotId": robot.robot_id, "armId": arm_id or "default", "speed": manipulation_speed or 0.5}),
                    "close_gripper": (
                        "RobotGripper",
                        "CloseGripper",
                        {"robotId": robot.robot_id, "armId": arm_id or "default", "speed": manipulation_speed or 0.5, "forceLimit": force_limit},
                    ),
                    "set_gripper_position": (
                        "RobotGripper",
                        "SetGripperPosition",
                        {"robotId": robot.robot_id, "position": gripper_position or 0.0, "armId": arm_id or "default", "speed": manipulation_speed or 0.5},
                    ),
                    "get_gripper_state": ("RobotGripper", "GetGripperState", {"robotId": robot.robot_id, "armId": arm_id or "default"}),
                    "move_to_pose": (
                        "RobotManipulator",
                        "MoveToPose",
                        {
                            "robotId": robot.robot_id,
                            "pose": end_effector_pose or {},
                            "armId": arm_id or "default",
                            "speed": manipulation_speed or 0.5,
                        },
                    ),
                    "home_arm": ("RobotManipulator", "HomeArm", {"robotId": robot.robot_id, "armId": arm_id or "default", "speed": manipulation_speed or 0.5}),
                }

                if action not in method_map:
                    return format_error_response(f"Unknown manipulation action: {action}", error_type="validation_error")

                class_name, method_name, params = method_map[action]
                result = await call_mounted_server_tool(
                    self.mounted_servers,
                    "unity",
                    "execute_unity_method",
                    {
                        "class_name": class_name,
                        "method_name": method_name,
                        "parameters": params,
                    },
                )

                return format_success_response(
                    f"Manipulation {action} executed for {robot.robot_id}",
                    robot_id=robot.robot_id,
                    category="manipulation",
                    action=action,
                    data=result,
                )
            else:
                logger.info("Mock manipulation", robot_id=robot.robot_id, action=action)
                mock_data = {}
                if action in ["get_arm_state", "get_gripper_state"]:
                    mock_data = {
                        "joint_positions": joint_positions or {"shoulder": 0.0, "elbow": 0.0, "wrist": 0.0},
                        "end_effector_pose": end_effector_pose or {"position": {"x": 0.0, "y": 0.5, "z": 0.0}, "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}},
                        "gripper_position": gripper_position or 0.0,
                    }
                return format_success_response(
                    f"Mock manipulation: {action} for {robot.robot_id}",
                    robot_id=robot.robot_id,
                    category="manipulation",
                    action=action,
                    data={"note": "Mock mode - Unity not available", **mock_data},
                )

        except Exception as e:
            return handle_tool_error("_handle_virtual_manipulation", e, robot_id=robot.robot_id, action=action)

    async def _handle_physical_manipulation(
        self,
        robot: Any,
        action: str,
        joint_positions: Optional[Dict[str, float]],
        end_effector_pose: Optional[Dict[str, Any]],
        gripper_position: Optional[float],
        arm_id: Optional[str],
        force_limit: Optional[float],
        manipulation_speed: Optional[float],
    ) -> Dict[str, Any]:
        """Handle physical robot manipulation (arms and grippers)."""
        logger.info("Physical robot manipulation", robot_id=robot.robot_id, action=action)
        return format_success_response(
            f"Physical robot manipulation: {action} for {robot.robot_id}",
            robot_id=robot.robot_id,
            category="manipulation",
            action=action,
            data={
                "note": "Physical robot manipulation not yet implemented (requires ROS MoveIt! or similar arm control stack)",
                "ros_topics": {
                    "joint_states": "/joint_states",
                    "arm_command": "/arm_controller/command",
                    "gripper_command": "/gripper_controller/command",
                },
            },
        )
