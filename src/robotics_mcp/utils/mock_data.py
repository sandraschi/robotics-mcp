"""Deprecated: Use simulated_data.py instead.

This module is kept for backward compatibility. Prefer simulated_data.
"""

from robotics_mcp.utils.simulated_data import (
    simulated_lidar_scan as mock_lidar_scan,
)
from robotics_mcp.utils.simulated_data import (
    simulated_map_data as mock_map_data,
)
from robotics_mcp.utils.simulated_data import (
    simulated_robot_status as mock_robot_status,
)
from robotics_mcp.utils.simulated_data import (
    simulated_sensor_data as mock_sensor_data,
)

__all__ = ["mock_lidar_scan", "mock_map_data", "mock_robot_status", "mock_sensor_data"]
