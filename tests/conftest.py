"""Pytest configuration and fixtures."""

import pytest
from pathlib import Path
from typing import Dict, Any

from robotics_mcp.utils.config_loader import ConfigLoader
from robotics_mcp.utils.state_manager import RobotStateManager


@pytest.fixture
def mock_config(tmp_path: Path) -> Dict[str, Any]:
    """Create mock configuration."""
    return {
        "robotics": {
            "moorebot_scout": {
                "enabled": False,
                "robot_id": "scout_01",
                "ip_address": "192.168.1.100",
                "port": 9090,
                "mock_mode": True,
            },
            "virtual": {
                "enabled": True,
                "platform": "unity",
            },
        },
        "server": {
            "enable_http": True,
            "http_port": 12230,
        },
    }


@pytest.fixture
def state_manager() -> RobotStateManager:
    """Create robot state manager."""
    return RobotStateManager()


@pytest.fixture
def mock_robot(state_manager: RobotStateManager) -> str:
    """Register a mock robot and return its ID."""
    robot = state_manager.register_robot("test_robot_01", "scout", platform="unity")
    return robot.robot_id

