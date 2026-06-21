"""Simulated sensor data for development/testing when hardware is unavailable.

Provides physics-based synthetic data when real hardware or MCP servers
(Unity raycast, ROS LiDAR) are not connected. Clearly labeled as simulated.
"""

import random
from typing import Any

import numpy as np


def simulated_lidar_scan(num_points: int = 360, range_min: float = 0.1, range_max: float = 12.0) -> dict[str, Any]:
    """Generate simulated LiDAR scan data for development without Unity/ROS.

    Args:
        num_points: Number of scan points.
        range_min: Minimum range in meters.
        range_max: Maximum range in meters.

    Returns:
        Simulated LiDAR scan data.
    """
    angles = np.linspace(0, 2 * np.pi, num_points)
    ranges = np.random.uniform(range_min, range_max, num_points)

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


def simulated_robot_status(robot_id: str, robot_type: str = "scout") -> dict[str, Any]:
    """Generate simulated robot status for development without hardware.

    Args:
        robot_id: Robot identifier.
        robot_type: Type of robot.

    Returns:
        Simulated robot status.
    """
    return {
        "robot_id": robot_id,
        "robot_type": robot_type,
        "battery_level": random.uniform(0.5, 1.0),
        "position": {
            "x": random.uniform(-5, 5),
            "y": random.uniform(-5, 5),
            "theta": random.uniform(0, 2 * np.pi),
        },
        "velocity": {"linear": random.uniform(0, 0.3), "angular": random.uniform(-2, 2)},
        "connected": True,
        "status": "idle",
    }


def simulated_sensor_data() -> dict[str, Any]:
    """Generate simulated sensor data for development without hardware.

    Returns:
        Simulated sensor data.
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


def simulated_map_data(width: int = 100, height: int = 100, resolution: float = 0.05) -> dict[str, Any]:
    """Generate simulated occupancy grid map for development without hardware.

    Args:
        width: Map width in pixels.
        height: Map height in pixels.
        resolution: Map resolution in meters per pixel.

    Returns:
        Simulated map data.
    """
    grid = np.ones((height, width), dtype=np.int8) * -1

    grid[0, :] = 100
    grid[-1, :] = 100
    grid[:, 0] = 100
    grid[:, -1] = 100

    for _ in range(5):
        x = random.randint(10, width - 10)
        y = random.randint(10, height - 10)
        size = random.randint(3, 8)
        grid[y - size : y + size, x - size : x + size] = 100

    grid[10:-10, 10:-10] = 0

    return {
        "width": width,
        "height": height,
        "resolution": resolution,
        "origin": {"x": -width * resolution / 2, "y": -height * resolution / 2, "theta": 0.0},
        "data": grid.tolist(),
    }
