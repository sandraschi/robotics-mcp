"""Yahboom ROS 2 robot client - ROSMASTER M1/X3/X3 Plus and similar models.

Uses roslibpy for actual rosbridge WebSocket communication when available.
Falls back to clearly-labeled simulated mode when roslibpy is not installed
or when mock_mode is explicitly enabled.

Requires: pip install roslibpy (optional dependency)
"""

import asyncio
import contextlib
import math
import time
from typing import Any

import structlog
from pydantic import BaseModel

logger = structlog.get_logger(__name__)

# Try to import roslibpy - graceful degradation if not available
try:
    import roslibpy

    ROSLIBPY_AVAILABLE = True
except ImportError:
    ROSLIBPY_AVAILABLE = False
    logger.warning(
        "roslibpy not installed - Yahboom client will run in simulated mode only. Install with: pip install roslibpy"
    )


class YahboomRobotConfig(BaseModel):
    """Configuration for Yahboom robots."""

    robot_id: str
    ip_address: str
    port: int = 9090  # ROS bridge default
    ros_domain_id: int = 0
    model: str = "ROSMaster-M1"  # ROSMaster-M1, ROSMaster-X3, ROSMaster-X3-Plus
    camera_enabled: bool = True
    navigation_enabled: bool = True
    arm_enabled: bool = False  # Enable when arm addon purchased
    mock_mode: bool = True


class YahboomClient:
    """ROS 2 client for Yahboom robots (ROSMaster M1/X3/X3 Plus, etc.).

    Architecture:
    - When roslibpy is available and mock_mode=False: real rosbridge WebSocket connection
    - When mock_mode=True or roslibpy unavailable: simulated data with 'simulated: True' flag
    - All simulated responses are honestly labeled - no gaslighting
    """

    def __init__(self, config: YahboomRobotConfig):
        """Initialize Yahboom ROS 2 client.

        Args:
            config: Robot configuration.
        """
        self.config = config
        self.connected = False
        self._ros_client: Any = None  # roslibpy.Ros instance
        self._publishers: dict[str, Any] = {}
        self._subscribers: dict[str, Any] = {}
        self._latest_data: dict[str, Any] = {}  # Cache of latest topic data
        self._simulated = config.mock_mode or not ROSLIBPY_AVAILABLE

        # ROS 2 topics for ROSMaster series (M1/X3/X3 Plus)
        self.topics = {
            "cmd_vel": "/cmd_vel",
            "odom": "/odom",
            "scan": "/scan",
            "battery": "/battery",
            "camera": "/camera/image_raw",
            "imu": "/imu",
        }

        if self.config.arm_enabled:
            self.topics.update(
                {
                    "joint_states": "/joint_states",
                    "arm_cmd": "/arm_controller/command",
                    "gripper_cmd": "/gripper_controller/command",
                }
            )

    @property
    def is_simulated(self) -> bool:
        """Whether client is running in simulated mode."""
        return self._simulated

    async def connect(self) -> bool:
        """Connect to robot via ROS bridge WebSocket.

        Returns:
            True if connection successful.
        """
        if self._simulated:
            logger.info(
                "Yahboom client running in SIMULATED mode",
                robot_id=self.config.robot_id,
                reason="mock_mode=True" if self.config.mock_mode else "roslibpy not installed",
            )
            self.connected = True
            return True

        try:
            self._ros_client = roslibpy.Ros(
                host=self.config.ip_address,
                port=self.config.port,
            )

            # Run connection in thread (roslibpy uses blocking I/O)
            loop = asyncio.get_running_loop()
            await loop.run_in_executor(None, self._ros_client.run)

            if not self._ros_client.is_connected:
                logger.error(
                    "Failed to establish rosbridge connection",
                    robot_id=self.config.robot_id,
                    address=f"{self.config.ip_address}:{self.config.port}",
                )
                return False

            # Set up publishers
            self._publishers["cmd_vel"] = roslibpy.Topic(
                self._ros_client,
                self.topics["cmd_vel"],
                "geometry_msgs/Twist",
            )
            self._publishers["cmd_vel"].advertise()

            # Set up subscribers for sensor data
            self._subscribe_to_topics()

            self.connected = True
            logger.info(
                "Connected to Yahboom robot via rosbridge",
                robot_id=self.config.robot_id,
                address=f"{self.config.ip_address}:{self.config.port}",
                model=self.config.model,
            )
            return True

        except Exception as e:
            logger.error(
                "Failed to connect to Yahboom robot",
                robot_id=self.config.robot_id,
                error=str(e),
                error_type=type(e).__name__,
            )
            self.connected = False
            return False

    def _subscribe_to_topics(self) -> None:
        """Subscribe to ROS topics for live data."""
        if not self._ros_client or not self._ros_client.is_connected:
            return

        # Odometry
        odom_sub = roslibpy.Topic(self._ros_client, self.topics["odom"], "nav_msgs/Odometry")
        odom_sub.subscribe(lambda msg: self._on_topic_data("odom", msg))
        self._subscribers["odom"] = odom_sub

        # IMU
        imu_sub = roslibpy.Topic(self._ros_client, self.topics["imu"], "sensor_msgs/Imu")
        imu_sub.subscribe(lambda msg: self._on_topic_data("imu", msg))
        self._subscribers["imu"] = imu_sub

        # LiDAR (if available)
        scan_sub = roslibpy.Topic(self._ros_client, self.topics["scan"], "sensor_msgs/LaserScan")
        scan_sub.subscribe(lambda msg: self._on_topic_data("scan", msg))
        self._subscribers["scan"] = scan_sub

        # Battery
        battery_sub = roslibpy.Topic(self._ros_client, self.topics["battery"], "sensor_msgs/BatteryState")
        battery_sub.subscribe(lambda msg: self._on_topic_data("battery", msg))
        self._subscribers["battery"] = battery_sub

        if self.config.arm_enabled:
            joint_sub = roslibpy.Topic(self._ros_client, self.topics["joint_states"], "sensor_msgs/JointState")
            joint_sub.subscribe(lambda msg: self._on_topic_data("joint_states", msg))
            self._subscribers["joint_states"] = joint_sub

    def _on_topic_data(self, topic_key: str, message: dict) -> None:
        """Callback for incoming ROS topic data."""
        self._latest_data[topic_key] = {
            "data": message,
            "timestamp": time.time(),
        }

    async def disconnect(self) -> None:
        """Disconnect from robot."""
        if self._ros_client and self._ros_client.is_connected:
            # Unsubscribe all
            for sub in self._subscribers.values():
                with contextlib.suppress(Exception):
                    sub.unsubscribe()
            # Unadvertise publishers
            for pub in self._publishers.values():
                with contextlib.suppress(Exception):
                    pub.unadvertise()
            # Close connection
            try:
                loop = asyncio.get_running_loop()
                await loop.run_in_executor(None, self._ros_client.terminate)
            except Exception as e:
                logger.warning("Error during rosbridge disconnect", error=str(e))

        self._ros_client = None
        self._publishers.clear()
        self._subscribers.clear()
        self._latest_data.clear()
        self.connected = False

    async def get_status(self) -> dict[str, Any]:
        """Get comprehensive robot status.

        Returns:
            Status dictionary with battery, position, sensors, etc.
        """
        if not self.connected:
            return {"error": "Not connected", "robot_id": self.config.robot_id}

        if self._simulated:
            return {
                "robot_id": self.config.robot_id,
                "model": self.config.model,
                "connected": True,
                "simulated": True,
                "battery": {"voltage": 11.8, "percentage": 85, "charging": False},
                "position": {"x": 0.0, "y": 0.0, "theta": 0.0},
                "sensors": {
                    "camera": self.config.camera_enabled,
                    "imu": True,
                    "lidar": False,
                },
                "capabilities": {
                    "navigation": self.config.navigation_enabled,
                    "arm": self.config.arm_enabled,
                    "camera_streaming": self.config.camera_enabled,
                },
            }

        # Real data from ROS topics
        status: dict[str, Any] = {
            "robot_id": self.config.robot_id,
            "model": self.config.model,
            "connected": True,
            "simulated": False,
        }

        # Extract battery from cached topic data
        battery_data = self._latest_data.get("battery", {}).get("data", {})
        status["battery"] = {
            "voltage": battery_data.get("voltage", 0.0),
            "percentage": battery_data.get("percentage", 0.0),
            "charging": battery_data.get("power_supply_status", 0) == 1,
        }

        # Extract position from odometry
        odom_data = self._latest_data.get("odom", {}).get("data", {})
        pose = odom_data.get("pose", {}).get("pose", {})
        position = pose.get("position", {})
        orientation = pose.get("orientation", {})
        # Convert quaternion to yaw
        qz = orientation.get("z", 0.0)
        qw = orientation.get("w", 1.0)
        theta = 2.0 * math.atan2(qz, qw)
        status["position"] = {
            "x": position.get("x", 0.0),
            "y": position.get("y", 0.0),
            "theta": theta,
        }

        # Sensor availability
        status["sensors"] = {
            "camera": self.config.camera_enabled,
            "imu": "imu" in self._latest_data,
            "lidar": "scan" in self._latest_data,
        }
        status["capabilities"] = {
            "navigation": self.config.navigation_enabled,
            "arm": self.config.arm_enabled,
            "camera_streaming": self.config.camera_enabled,
        }
        status["data_freshness"] = {
            key: time.time() - entry.get("timestamp", 0) for key, entry in self._latest_data.items()
        }

        return status

    async def move(self, linear: float, angular: float, duration: float | None = None) -> dict[str, Any]:
        """Move robot with velocity commands via /cmd_vel.

        Args:
            linear: Linear velocity (m/s). Clamped to [-1.0, 1.0] for safety.
            angular: Angular velocity (rad/s). Clamped to [-2.0, 2.0] for safety.
            duration: Movement duration in seconds (max 10s safety limit).

        Returns:
            Movement result.
        """
        if not self.connected:
            return {"error": "Not connected", "robot_id": self.config.robot_id}

        # Safety clamps
        linear = max(-1.0, min(1.0, linear))
        angular = max(-2.0, min(2.0, angular))
        if duration is not None:
            duration = min(duration, 10.0)

        if self._simulated:
            logger.info(
                "SIMULATED movement command",
                robot_id=self.config.robot_id,
                linear=linear,
                angular=angular,
                duration=duration,
            )
            if duration:
                await asyncio.sleep(min(duration, 1.0))
            return {
                "success": True,
                "robot_id": self.config.robot_id,
                "command": "move",
                "linear": linear,
                "angular": angular,
                "duration": duration,
                "simulated": True,
            }

        # Publish Twist message to /cmd_vel
        twist_msg = {
            "linear": {"x": linear, "y": 0.0, "z": 0.0},
            "angular": {"x": 0.0, "y": 0.0, "z": angular},
        }

        try:
            cmd_vel_pub = self._publishers.get("cmd_vel")
            if not cmd_vel_pub:
                return {"error": "cmd_vel publisher not initialized"}

            cmd_vel_pub.publish(roslibpy.Message(twist_msg))

            if duration:
                await asyncio.sleep(duration)
                # Send stop command after duration
                stop_msg = {
                    "linear": {"x": 0.0, "y": 0.0, "z": 0.0},
                    "angular": {"x": 0.0, "y": 0.0, "z": 0.0},
                }
                cmd_vel_pub.publish(roslibpy.Message(stop_msg))

            return {
                "success": True,
                "robot_id": self.config.robot_id,
                "command": "move",
                "linear": linear,
                "angular": angular,
                "duration": duration,
                "simulated": False,
            }
        except Exception as e:
            logger.error("Failed to publish move command", error=str(e))
            return {"error": f"Move command failed: {e}", "robot_id": self.config.robot_id}

    async def stop(self) -> dict[str, Any]:
        """Emergency stop - publish zero velocity."""
        if not self.connected:
            return {"error": "Not connected", "robot_id": self.config.robot_id}

        if self._simulated:
            logger.info("SIMULATED emergency stop", robot_id=self.config.robot_id)
            return {
                "success": True,
                "robot_id": self.config.robot_id,
                "command": "stop",
                "simulated": True,
            }

        try:
            cmd_vel_pub = self._publishers.get("cmd_vel")
            if cmd_vel_pub:
                stop_msg = {
                    "linear": {"x": 0.0, "y": 0.0, "z": 0.0},
                    "angular": {"x": 0.0, "y": 0.0, "z": 0.0},
                }
                cmd_vel_pub.publish(roslibpy.Message(stop_msg))
            return {
                "success": True,
                "robot_id": self.config.robot_id,
                "command": "stop",
                "simulated": False,
            }
        except Exception as e:
            logger.error("Failed to publish stop command", error=str(e))
            return {"error": f"Stop command failed: {e}", "robot_id": self.config.robot_id}

    async def get_camera_frame(self) -> bytes | None:
        """Get latest camera frame.

        Returns:
            JPEG image data or None if no camera.
        """
        if not self.config.camera_enabled:
            return None

        if self._simulated:
            return None  # No fake camera data - return None honestly

        # Real: request compressed image via ROS service
        if self._ros_client and self._ros_client.is_connected:
            try:
                roslibpy.Service(
                    self._ros_client,
                    "/camera/image_raw/compressed",
                    "sensor_msgs/CompressedImage",
                )
                # This would need proper async handling in production
                return None  # TODO: implement async image fetch
            except Exception as e:
                logger.warning("Failed to get camera frame", error=str(e))
        return None

    async def control_arm(self, joint_angles: dict[str, float]) -> dict[str, Any]:
        """Control robotic arm joints.

        Args:
            joint_angles: Dictionary of joint names to target angles (radians).

        Returns:
            Arm control result.
        """
        if not self.config.arm_enabled:
            return {"error": "Arm not enabled/configured", "robot_id": self.config.robot_id}

        if self._simulated:
            logger.info(
                "SIMULATED arm control",
                robot_id=self.config.robot_id,
                joints=joint_angles,
            )
            return {
                "success": True,
                "robot_id": self.config.robot_id,
                "command": "arm_control",
                "joint_angles": joint_angles,
                "simulated": True,
            }

        try:
            arm_pub = self._publishers.get("arm_cmd")
            if not arm_pub:
                # Create publisher on first use
                arm_pub = roslibpy.Topic(
                    self._ros_client,
                    self.topics["arm_cmd"],
                    "trajectory_msgs/JointTrajectory",
                )
                arm_pub.advertise()
                self._publishers["arm_cmd"] = arm_pub

            # Build JointTrajectory message
            joint_names = list(joint_angles.keys())
            positions = [joint_angles[j] for j in joint_names]
            msg = {
                "joint_names": joint_names,
                "points": [
                    {
                        "positions": positions,
                        "time_from_start": {"secs": 2, "nsecs": 0},
                    }
                ],
            }
            arm_pub.publish(roslibpy.Message(msg))

            return {
                "success": True,
                "robot_id": self.config.robot_id,
                "command": "arm_control",
                "joint_angles": joint_angles,
                "simulated": False,
            }
        except Exception as e:
            logger.error("Failed to control arm", error=str(e))
            return {"error": f"Arm control failed: {e}", "robot_id": self.config.robot_id}

    async def control_gripper(self, action: str, force: float | None = None) -> dict[str, Any]:
        """Control gripper.

        Args:
            action: "open", "close", "stop"
            force: Grip force (0.0-1.0)

        Returns:
            Gripper control result.
        """
        if not self.config.arm_enabled:
            return {
                "error": "Gripper not available (arm not configured)",
                "robot_id": self.config.robot_id,
            }

        if self._simulated:
            logger.info(
                "SIMULATED gripper control",
                robot_id=self.config.robot_id,
                action=action,
                force=force,
            )
            return {
                "success": True,
                "robot_id": self.config.robot_id,
                "command": "gripper_control",
                "action": action,
                "force": force,
                "simulated": True,
            }

        try:
            gripper_pub = self._publishers.get("gripper_cmd")
            if not gripper_pub:
                gripper_pub = roslibpy.Topic(
                    self._ros_client,
                    self.topics["gripper_cmd"],
                    "control_msgs/GripperCommand",
                )
                gripper_pub.advertise()
                self._publishers["gripper_cmd"] = gripper_pub

            position_map = {"open": 1.0, "close": 0.0, "stop": 0.5}
            msg = {
                "command": {
                    "position": position_map.get(action, 0.5),
                    "max_effort": force or 0.5,
                }
            }
            gripper_pub.publish(roslibpy.Message(msg))

            return {
                "success": True,
                "robot_id": self.config.robot_id,
                "command": "gripper_control",
                "action": action,
                "force": force,
                "simulated": False,
            }
        except Exception as e:
            logger.error("Failed to control gripper", error=str(e))
            return {"error": f"Gripper control failed: {e}", "robot_id": self.config.robot_id}

    async def navigate_to_pose(self, x: float, y: float, theta: float) -> dict[str, Any]:
        """Navigate to specific pose using ROS Navigation stack (Nav2).

        Sends a goal to the /navigate_to_pose action server.

        Args:
            x: Target X position (meters)
            y: Target Y position (meters)
            theta: Target orientation (radians)

        Returns:
            Navigation result.
        """
        if not self.config.navigation_enabled:
            return {"error": "Navigation not enabled", "robot_id": self.config.robot_id}

        if self._simulated:
            logger.info(
                "SIMULATED navigation",
                robot_id=self.config.robot_id,
                target={"x": x, "y": y, "theta": theta},
            )
            return {
                "success": True,
                "robot_id": self.config.robot_id,
                "command": "navigate",
                "target_pose": {"x": x, "y": y, "theta": theta},
                "simulated": True,
                "note": "Simulated - install roslibpy and disable mock_mode for real navigation",
            }

        try:
            # Use ROS action client for Nav2
            # Convert theta to quaternion
            qz = math.sin(theta / 2.0)
            qw = math.cos(theta / 2.0)

            goal_msg = {
                "pose": {
                    "header": {"frame_id": "map"},
                    "pose": {
                        "position": {"x": x, "y": y, "z": 0.0},
                        "orientation": {"x": 0.0, "y": 0.0, "z": qz, "w": qw},
                    },
                }
            }

            # Publish to /goal_pose topic (Nav2 simple commander approach)
            goal_pub = roslibpy.Topic(
                self._ros_client,
                "/goal_pose",
                "geometry_msgs/PoseStamped",
            )
            goal_pub.advertise()
            goal_pub.publish(roslibpy.Message(goal_msg["pose"]))

            # Wait briefly then unadvertise
            await asyncio.sleep(0.5)
            goal_pub.unadvertise()

            return {
                "success": True,
                "robot_id": self.config.robot_id,
                "command": "navigate",
                "target_pose": {"x": x, "y": y, "theta": theta},
                "simulated": False,
                "note": "Goal published to /goal_pose - Nav2 will handle path planning",
            }
        except Exception as e:
            logger.error("Failed to send navigation goal", error=str(e))
            return {"error": f"Navigation failed: {e}", "robot_id": self.config.robot_id}

    async def call_ros_service(self, service_name: str, service_type: str, args: dict | None = None) -> dict[str, Any]:
        """Call an arbitrary ROS service (for extensibility).

        Args:
            service_name: Full service name (e.g. /rosout/get_loggers)
            service_type: Service type (e.g. rcl_interfaces/GetParameters)
            args: Service request arguments

        Returns:
            Service response as dict.
        """
        if self._simulated:
            return {
                "error": "Cannot call ROS services in simulated mode",
                "simulated": True,
                "robot_id": self.config.robot_id,
            }

        if not self._ros_client or not self._ros_client.is_connected:
            return {"error": "Not connected to rosbridge", "robot_id": self.config.robot_id}

        try:
            service = roslibpy.Service(self._ros_client, service_name, service_type)
            request = roslibpy.ServiceRequest(args or {})

            loop = asyncio.get_running_loop()
            response = await loop.run_in_executor(None, lambda: service.call(request))
            return {"success": True, "response": response, "simulated": False}
        except Exception as e:
            logger.error("ROS service call failed", service=service_name, error=str(e))
            return {"error": f"Service call failed: {e}", "robot_id": self.config.robot_id}
