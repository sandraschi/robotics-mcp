"""Gazebo simulation client - ROS bridge for simulated robots in Gazebo."""

import asyncio
from typing import Any

import structlog
from pydantic import BaseModel

logger = structlog.get_logger(__name__)

# Optional roslibpy - only required when mock_mode=False
try:
    import roslibpy
except ImportError:
    roslibpy = None  # type: ignore[assignment]


class GazeboRobotConfig(BaseModel):
    """Configuration for Gazebo simulated robots."""

    robot_id: str
    host: str = "localhost"
    port: int = 9090  # rosbridge WebSocket default
    model: str = "turtlebot3"  # turtlebot3, diff_drive, etc.
    cmd_vel_topic: str = "/cmd_vel"
    odom_topic: str = "/odom"
    scan_topic: str = "/scan"
    camera_enabled: bool = True
    camera_topic: str = "/camera/image_raw"
    mock_mode: bool = True


class GazeboClient:
    """ROS bridge client for robots simulated in Gazebo.

    Connects via rosbridge WebSocket (port 9090) to control differential-drive
    and similar robots. Same topic layout as Yahboom (cmd_vel, odom, scan).
    """

    def __init__(self, config: GazeboRobotConfig):
        """Initialize Gazebo ROS bridge client.

        Args:
            config: Robot configuration.
        """
        self.config = config
        self.connected = False
        self._ros: Any = None
        self._cmd_vel_publisher: Any = None
        self._odom_data: dict[str, Any] = {}
        self._scan_data: dict[str, Any] = {}

    def _connect_sync(self) -> bool:
        """Synchronous ROS connection (run in thread)."""
        if roslibpy is None:
            logger.error("roslibpy not installed", hint="pip install roslibpy")
            return False
        try:
            self._ros = roslibpy.Ros(host=self.config.host, port=self.config.port)
            self._ros.run()
            self._cmd_vel_publisher = roslibpy.Topic(self._ros, self.config.cmd_vel_topic, "geometry_msgs/Twist")
            self._cmd_vel_publisher.advertise()
            self.connected = True
            return True
        except Exception as e:
            logger.error(
                "Gazebo ROS bridge connection failed",
                host=self.config.host,
                port=self.config.port,
                error=str(e),
            )
            return False

    def _disconnect_sync(self) -> None:
        """Synchronous disconnect."""
        try:
            if self._cmd_vel_publisher:
                self._cmd_vel_publisher.unadvertise()
            if self._ros:
                self._ros.terminate()
        except Exception as e:
            logger.warning("Gazebo disconnect error", error=str(e))
        finally:
            self._ros = None
            self._cmd_vel_publisher = None
            self.connected = False

    def _publish_cmd_vel_sync(self, linear: float, angular: float) -> None:
        """Publish Twist to cmd_vel (synchronous)."""
        if not self._cmd_vel_publisher or not self.connected:
            return
        msg = roslibpy.Message(
            {
                "linear": {"x": linear, "y": 0.0, "z": 0.0},
                "angular": {"x": 0.0, "y": 0.0, "z": angular},
            }
        )
        self._cmd_vel_publisher.publish(msg)

    async def connect(self) -> bool:
        """Connect to Gazebo via rosbridge WebSocket.

        Returns:
            True if connection successful.
        """
        if self.config.mock_mode:
            logger.info("Gazebo running in mock mode", robot_id=self.config.robot_id)
            self.connected = True
            return True

        return await asyncio.to_thread(self._connect_sync)

    async def disconnect(self) -> None:
        """Disconnect from rosbridge."""
        if self.config.mock_mode:
            self.connected = False
            return
        await asyncio.to_thread(self._disconnect_sync)

    async def get_status(self) -> dict[str, Any]:
        """Get robot status from Gazebo/ROS.

        Returns:
            Status dictionary.
        """
        if not self.connected:
            return {"error": "Not connected"}

        if self.config.mock_mode:
            return {
                "robot_id": self.config.robot_id,
                "model": self.config.model,
                "connected": True,
                "platform": "gazebo",
                "battery": {"percentage": 100, "charging": False},
                "position": {"x": 0.0, "y": 0.0, "theta": 0.0},
                "sensors": {
                    "camera": self.config.camera_enabled,
                    "lidar": True,
                },
                "capabilities": {
                    "navigation": True,
                    "camera_streaming": self.config.camera_enabled,
                },
                "mock": True,
            }

        return {
            "robot_id": self.config.robot_id,
            "model": self.config.model,
            "connected": True,
            "platform": "gazebo",
            "odom": self._odom_data,
            "mock": False,
        }

    async def move(self, linear: float, angular: float, duration: float | None = None) -> dict[str, Any]:
        """Publish velocity command.

        Args:
            linear: Linear velocity (m/s).
            angular: Angular velocity (rad/s).
            duration: Optional movement duration (cap at 1s in mock).

        Returns:
            Movement result.
        """
        if not self.connected:
            return {"error": "Not connected"}

        if self.config.mock_mode:
            if duration:
                await asyncio.sleep(min(duration, 1.0))
            return {
                "success": True,
                "robot_id": self.config.robot_id,
                "command": "move",
                "linear": linear,
                "angular": angular,
                "duration": duration,
                "mock": True,
            }

        await asyncio.to_thread(self._publish_cmd_vel_sync, linear, angular)
        if duration:
            await asyncio.sleep(min(duration, 1.0))
            await asyncio.to_thread(self._publish_cmd_vel_sync, 0.0, 0.0)

        return {
            "success": True,
            "robot_id": self.config.robot_id,
            "command": "move",
            "linear": linear,
            "angular": angular,
        }

    async def stop(self) -> dict[str, Any]:
        """Emergency stop (zero velocity)."""
        if not self.connected:
            return {"error": "Not connected"}

        if not self.config.mock_mode:
            await asyncio.to_thread(self._publish_cmd_vel_sync, 0.0, 0.0)

        return {
            "success": True,
            "robot_id": self.config.robot_id,
            "command": "stop",
            "mock": self.config.mock_mode,
        }

    def _spawn_model_sync(self, model_name: str, sdf_xml: str, x: float, y: float, z: float) -> dict[str, Any]:
        """Spawn a model in Gazebo via ROS service (synchronous).

        Uses /spawn_sdf_model (Gazebo Classic) or /world/<world>/create (Ignition/Gz).
        """
        if roslibpy is None or not self._ros:
            return {"success": False, "error": "rosbridge not connected", "message": "rosbridge not connected"}

        classic_err = None
        gz_err = None

        # Try Gazebo Classic service first
        try:
            service = roslibpy.Service(self._ros, "/gazebo/spawn_sdf_model", "gazebo_msgs/SpawnModel")
            request = roslibpy.ServiceRequest(
                {
                    "model_name": model_name,
                    "model_xml": sdf_xml,
                    "robot_namespace": "",
                    "initial_pose": {
                        "position": {"x": x, "y": y, "z": z},
                        "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                    },
                    "reference_frame": "world",
                }
            )
            result = service.call(request, timeout=15)
            return {
                "success": result.get("success", False),
                "status_message": result.get("status_message", ""),
                "service": "/gazebo/spawn_sdf_model",
            }
        except Exception as e1:
            classic_err = e1
            logger.info("Gazebo Classic spawn failed, trying Gz service", error=str(classic_err))

        # Try Gz Sim service (Ignition/Gazebo Sim)
        try:
            service = roslibpy.Service(self._ros, "/world/default/create", "gz/msgs/EntityFactory")
            request = roslibpy.ServiceRequest(
                {
                    "sdf": sdf_xml,
                    "name": model_name,
                    "pose": {
                        "position": {"x": x, "y": y, "z": z},
                    },
                }
            )
            result = service.call(request, timeout=15)
            return {
                "success": True,
                "result": result,
                "service": "/world/default/create",
            }
        except Exception as e2:
            gz_err = e2
            return {
                "success": False,
                "error": f"Both spawn services failed. Classic: {classic_err}, Gz: {gz_err}",
                "message": f"Both spawn services failed. Classic: {classic_err}, Gz: {gz_err}",
            }

    async def spawn_model(
        self,
        model_name: str,
        sdf_xml: str,
        x: float = 0.0,
        y: float = 0.0,
        z: float = 0.0,
    ) -> dict[str, Any]:
        """Spawn a model in the Gazebo simulation.

        Args:
            model_name: Unique name for the spawned model
            sdf_xml: SDF XML content
            x, y, z: Spawn position in world frame

        Returns:
            Spawn result dict.
        """
        if not self.connected:
            return {"error": "Not connected to Gazebo rosbridge"}

        if self.config.mock_mode:
            return {
                "success": True,
                "model_name": model_name,
                "position": {"x": x, "y": y, "z": z},
                "mock": True,
                "simulated": True,
            }

        return await asyncio.to_thread(self._spawn_model_sync, model_name, sdf_xml, x, y, z)

    def _delete_model_sync(self, model_name: str) -> dict[str, Any]:
        """Delete a model from Gazebo simulation (synchronous)."""
        if roslibpy is None or not self._ros:
            return {"success": False, "error": "rosbridge not connected", "message": "rosbridge not connected"}

        try:
            service = roslibpy.Service(self._ros, "/gazebo/delete_model", "gazebo_msgs/DeleteModel")
            request = roslibpy.ServiceRequest({"model_name": model_name})
            result = service.call(request, timeout=10)
            return {
                "success": result.get("success", False),
                "status_message": result.get("status_message", ""),
            }
        except Exception as e:
            return {"success": False, "error": str(e), "message": str(e)}

    async def delete_model(self, model_name: str) -> dict[str, Any]:
        """Delete a spawned model from the simulation.

        Args:
            model_name: Name of model to delete.

        Returns:
            Deletion result.
        """
        if not self.connected:
            return {"error": "Not connected"}

        if self.config.mock_mode:
            return {"success": True, "model_name": model_name, "mock": True}

        return await asyncio.to_thread(self._delete_model_sync, model_name)
