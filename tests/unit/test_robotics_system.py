"""Unit tests for robotics_system tool."""

from unittest.mock import MagicMock

import pytest

from robotics_mcp.tools.robotics_system import RoboticsSystemTool
from robotics_mcp.utils.config_loader import ConfigLoader
from robotics_mcp.utils.state_manager import RobotStateManager
from tests.helpers import mock_ctx, payload


@pytest.fixture
def state_manager():
    """Create robot state manager."""
    return RobotStateManager()


@pytest.fixture
def mock_config():
    """Create mock config."""
    return MagicMock()


@pytest.fixture
def config_loader():
    """Create config loader."""
    return ConfigLoader()


@pytest.fixture
def mock_mounted_servers():
    """Create mock mounted servers."""
    return {}


@pytest.fixture
def robotics_system_tool(mock_mcp, state_manager, mock_config, config_loader, mock_mounted_servers):
    """Create robotics system tool instance."""
    tool = RoboticsSystemTool(mock_mcp, state_manager, mock_config, config_loader, mock_mounted_servers)
    tool.register()
    return tool


@pytest.mark.asyncio
async def test_robotics_system_help(robotics_system_tool):
    """Test help operation."""
    tool_func = robotics_system_tool.mcp.tool.registered_func

    result = await tool_func(
        mock_ctx(),
        operation="help",
    )

    assert result["status"] == "success"
    assert "tools" in result or "server_name" in result


@pytest.mark.asyncio
async def test_robotics_system_status(robotics_system_tool):
    """Test status operation."""
    tool_func = robotics_system_tool.mcp.tool.registered_func

    result = await tool_func(
        mock_ctx(),
        operation="status",
    )

    assert result["success"] is True
    data = payload(result)
    assert "robots_count" in data or "status" in data


@pytest.mark.asyncio
async def test_robotics_system_list_robots(robotics_system_tool, state_manager):
    """Test list_robots operation."""
    state_manager.register_robot("scout_01", "scout")
    state_manager.register_robot("vbot_01", "scout", platform="unity")

    tool_func = robotics_system_tool.mcp.tool.registered_func

    result = await tool_func(
        mock_ctx(),
        operation="list_robots",
    )

    assert result["success"] is True
    assert len(payload(result)["robots"]) >= 2


@pytest.mark.asyncio
async def test_robotics_system_list_robots_filtered(robotics_system_tool, state_manager):
    """Test list_robots with filters."""
    state_manager.register_robot("scout_01", "scout")
    state_manager.register_robot("vbot_01", "scout", platform="unity")

    tool_func = robotics_system_tool.mcp.tool.registered_func

    result = await tool_func(
        mock_ctx(),
        operation="list_robots",
        is_virtual=True,
    )

    assert result["success"] is True
    robots = payload(result)["robots"]
    assert all(r.get("is_virtual", False) for r in robots)


@pytest.mark.asyncio
async def test_robotics_system_invalid_operation(robotics_system_tool):
    """Test invalid operation."""
    tool_func = robotics_system_tool.mcp.tool.registered_func

    result = await tool_func(
        mock_ctx(),
        operation="invalid_op",
    )

    assert result["status"] == "error"
    assert "unknown operation" in result["message"].lower()
