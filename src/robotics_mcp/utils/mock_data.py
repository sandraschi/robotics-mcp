"""Mock data fixtures for testing without hardware."""

import random
from typing import Any, Dict, List

import numpy as np


def mock_lidar_scan(num_points: int = 360, range_min: float = 0.1, range_max: float = 12.0) -> Dict[str, Any]:
    """Generate mock LiDAR scan data.

    Args:
        num_points: Number of scan points.
        range_min: Minimum range in meters.
        range_max: Maximum range in meters.

    Returns:
        Mock LiDAR scan data.
    """
    angles = np.linspace(0, 2 * np.pi, num_points)
    ranges = np.random.uniform(range_min, range_max, num_points)

    # Add some obstacles (closer ranges)
    obstacle_indices = random.sample(range(num_points), num_points // 10)
    for idx in obstacle_indices:
        ranges[idx] = random.uniform(range_min, 2.0)

    return {
        "ranges": ranges.tolist(),
        "angles": angles.tolist(),
        "intensities": np.random.uniform(0, 1, num_points).tolist(),
        "range_min": range_min,
        "range_max": range_max,
        "timestamp": 0.0,
    }


def mock_robot_status(robot_id: str, robot_type: str = "scout") -> Dict[str, Any]:
    """Generate mock robot status.

    Args:
        robot_id: Robot identifier.
        robot_type: Type of robot.

    Returns:
        Mock robot status.
    """
    return {
        "robot_id": robot_id,
        "robot_type": robot_type,
        "battery_level": random.uniform(0.5, 1.0),
        "position": {"x": random.uniform(-5, 5), "y": random.uniform(-5, 5), "theta": random.uniform(0, 2 * np.pi)},
        "velocity": {"linear": random.uniform(0, 0.3), "angular": random.uniform(-2, 2)},
        "connected": True,
        "status": "idle",
    }


def mock_sensor_data() -> Dict[str, Any]:
    """Generate mock sensor data.

    Returns:
        Mock sensor data.
    """
    return {
        "tof": {"distance": random.uniform(0.1, 3.0), "valid": True},
        "imu": {
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
            "angular_velocity": {"x": 0.0, "y": 0.0, "z": 0.0},
            "linear_acceleration": {"x": 0.0, "y": 0.0, "z": 9.8},
        },
        "light": {"ch0": random.randint(0, 65535), "ch1": random.randint(0, 65535)},
    }


def mock_map_data(width: int = 100, height: int = 100, resolution: float = 0.05) -> Dict[str, Any]:
    """Generate mock occupancy grid map.

    Args:
        width: Map width in pixels.
        height: Map height in pixels.
        resolution: Map resolution in meters per pixel.

    Returns:
        Mock map data.
    """
    # Create a simple map with walls around edges and some obstacles
    grid = np.ones((height, width), dtype=np.int8) * -1  # Unknown

    # Walls around edges
    grid[0, :] = 100  # Occupied
    grid[-1, :] = 100
    grid[:, 0] = 100
    grid[:, -1] = 100

    # Some obstacles
    for _ in range(5):
        x = random.randint(10, width - 10)
        y = random.randint(10, height - 10)
        size = random.randint(3, 8)
        grid[y - size : y + size, x - size : x + size] = 100

    # Free space
    grid[10:-10, 10:-10] = 0

    return {
        "width": width,
        "height": height,
        "resolution": resolution,
        "origin": {"x": -width * resolution / 2, "y": -height * resolution / 2, "theta": 0.0},
        "data": grid.tolist(),
    }

