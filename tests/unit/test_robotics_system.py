"""Unit tests for robotics_system tool."""

import pytest
from unittest.mock import MagicMock, patch

from robotics_mcp.tools.robotics_system import RoboticsSystemTool
from robotics_mcp.utils.state_manager import RobotStateManager
from robotics_mcp.utils.config_loader import ConfigLoader


@pytest.fixture
def mock_mcp():
    """Create mock MCP server."""
    mcp = MagicMock()
    mcp.tool = MagicMock(return_value=lambda f: f)
    # Mock _tools attribute
    mcp._tools = {
        "robotics_system": MagicMock(),
        "robot_control": MagicMock(),
    }
    return mcp


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
    tool_func = robotics_system_tool.mcp.tool.call_args[0][0]
    
    result = await tool_func(
        operation="help",
    )
    
    assert result["success"] is True
    assert "tools" in result["data"] or "server" in result["data"]


@pytest.mark.asyncio
async def test_robotics_system_status(robotics_system_tool):
    """Test status operation."""
    tool_func = robotics_system_tool.mcp.tool.call_args[0][0]
    
    result = await tool_func(
        operation="status",
    )
    
    assert result["success"] is True
    assert "status" in result["data"] or "robots" in result["data"]


@pytest.mark.asyncio
async def test_robotics_system_list_robots(robotics_system_tool, state_manager):
    """Test list_robots operation."""
    # Register some robots
    state_manager.register_robot("scout_01", "scout")
    state_manager.register_robot("vbot_01", "scout", platform="unity")
    
    tool_func = robotics_system_tool.mcp.tool.call_args[0][0]
    
    result = await tool_func(
        operation="list_robots",
    )
    
    assert result["success"] is True
    assert len(result["data"]["robots"]) >= 2


@pytest.mark.asyncio
async def test_robotics_system_list_robots_filtered(robotics_system_tool, state_manager):
    """Test list_robots with filters."""
    # Register robots
    state_manager.register_robot("scout_01", "scout")
    state_manager.register_robot("vbot_01", "scout", platform="unity")
    
    tool_func = robotics_system_tool.mcp.tool.call_args[0][0]
    
    result = await tool_func(
        operation="list_robots",
        is_virtual=True,
    )
    
    assert result["success"] is True
    robots = result["data"]["robots"]
    assert all(r.get("is_virtual", False) for r in robots)


@pytest.mark.asyncio
async def test_robotics_system_invalid_operation(robotics_system_tool):
    """Test invalid operation."""
    tool_func = robotics_system_tool.mcp.tool.call_args[0][0]
    
    result = await tool_func(
        operation="invalid_op",
    )
    
    assert result["success"] is False
    assert "unknown operation" in result["error"].lower()

