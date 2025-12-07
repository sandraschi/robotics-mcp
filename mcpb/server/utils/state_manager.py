"""State management for robot connections."""

from typing import Any, Dict, Optional

import structlog

logger = structlog.get_logger(__name__)


class RobotState:
    """Robot state information."""

    def __init__(
        self,
        robot_id: str,
        robot_type: str,
        platform: Optional[str] = None,
        connected: bool = False,
        metadata: Optional[Dict[str, Any]] = None,
    ):
        """Initialize robot state.

        Args:
            robot_id: Unique robot identifier.
            robot_type: Type of robot (e.g., "scout", "go2", "g1").
            platform: Platform for virtual robots ("unity", "vrchat").
            connected: Whether robot is currently connected.
            metadata: Additional robot metadata.
        """
        self.robot_id = robot_id
        self.robot_type = robot_type
        self.platform = platform
        self.connected = connected
        self.metadata = metadata or {}
        self.is_virtual = platform is not None

    def to_dict(self) -> Dict[str, Any]:
        """Convert robot state to dictionary.

        Returns:
            Robot state as dictionary.
        """
        return {
            "robot_id": self.robot_id,
            "robot_type": self.robot_type,
            "platform": self.platform,
            "connected": self.connected,
            "is_virtual": self.is_virtual,
            "metadata": self.metadata,
        }


class RobotStateManager:
    """Manage robot connections and state."""

    def __init__(self):
        """Initialize state manager."""
        self.robots: Dict[str, RobotState] = {}
        logger.info("Robot state manager initialized")

    def register_robot(
        self,
        robot_id: str,
        robot_type: str,
        platform: Optional[str] = None,
        metadata: Optional[Dict[str, Any]] = None,
    ) -> RobotState:
        """Register a robot.

        Args:
            robot_id: Unique robot identifier.
            robot_type: Type of robot.
            platform: Platform for virtual robots.
            metadata: Additional metadata.

        Returns:
            Created robot state.

        Raises:
            ValueError: If robot_id already exists.
        """
        if robot_id in self.robots:
            raise ValueError(f"Robot {robot_id} already registered")

        robot = RobotState(robot_id, robot_type, platform, connected=False, metadata=metadata)
        self.robots[robot_id] = robot
        logger.info("Robot registered", robot_id=robot_id, robot_type=robot_type, platform=platform)
        return robot

    def get_robot(self, robot_id: str) -> Optional[RobotState]:
        """Get robot state by ID.

        Args:
            robot_id: Robot identifier.

        Returns:
            Robot state or None if not found.
        """
        return self.robots.get(robot_id)

    def list_robots(self, robot_type: Optional[str] = None, is_virtual: Optional[bool] = None) -> list[RobotState]:
        """List all registered robots.

        Args:
            robot_type: Filter by robot type.
            is_virtual: Filter by virtual/physical.

        Returns:
            List of robot states.
        """
        robots = list(self.robots.values())

        if robot_type:
            robots = [r for r in robots if r.robot_type == robot_type]

        if is_virtual is not None:
            robots = [r for r in robots if r.is_virtual == is_virtual]

        return robots

    def update_robot_status(self, robot_id: str, connected: bool) -> None:
        """Update robot connection status.

        Args:
            robot_id: Robot identifier.
            connected: Connection status.
        """
        robot = self.get_robot(robot_id)
        if robot:
            robot.connected = connected
            logger.info("Robot status updated", robot_id=robot_id, connected=connected)
        else:
            logger.warning("Robot not found for status update", robot_id=robot_id)

    def unregister_robot(self, robot_id: str) -> None:
        """Unregister a robot.

        Args:
            robot_id: Robot identifier.
        """
        if robot_id in self.robots:
            del self.robots[robot_id]
            logger.info("Robot unregistered", robot_id=robot_id)
        else:
            logger.warning("Robot not found for unregister", robot_id=robot_id)

