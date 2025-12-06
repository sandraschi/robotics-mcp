"""Unit tests for virtual robotics tool."""

import pytest
from robotics_mcp.tools.virtual_robotics import VirtualRoboticsTool
from robotics_mcp.utils.state_manager import RobotStateManager
from unittest.mock import Mock, AsyncMock, patch


@pytest.fixture
def mock_mcp():
    """Create mock MCP server."""
    mcp = Mock()
    return mcp


@pytest.fixture
def virtual_robotics_tool(mock_mcp, state_manager):
    """Create virtual robotics tool instance."""
    return VirtualRoboticsTool(mock_mcp, state_manager)


@pytest.mark.asyncio
async def test_spawn_robot_registration(virtual_robotics_tool, state_manager):
    """Test that spawning robot registers it in state manager."""
    result = await virtual_robotics_tool._spawn_robot(
        robot_type="scout",
        robot_id="test_scout",
        position={"x": 0, "y": 0, "z": 0},
        scale=1.0,
        platform="vrchat",
    )

    assert result["status"] == "success"
    robot = state_manager.get_robot("test_scout")
    assert robot is not None
    assert robot.robot_type == "scout"
    assert robot.platform == "vrchat"


@pytest.mark.asyncio
async def test_spawn_robot_auto_id(virtual_robotics_tool, state_manager):
    """Test auto-generation of robot ID."""
    result = await virtual_robotics_tool._spawn_robot(
        robot_type="scout",
        robot_id=None,
        position={"x": 0, "y": 0, "z": 0},
        scale=1.0,
        platform="unity",
    )

    assert result["status"] == "success"
    assert "robot_id" in result
    assert result["robot_id"].startswith("vbot_scout")


@pytest.mark.asyncio
async def test_get_status(virtual_robotics_tool, state_manager):
    """Test getting virtual robot status."""
    # Register robot first
    robot = state_manager.register_robot("test_scout", "scout", platform="unity")

    result = await virtual_robotics_tool._get_status("test_scout")

    assert result["status"] == "success"
    assert result["robot"]["robot_id"] == "test_scout"


@pytest.mark.asyncio
async def test_get_status_not_found(virtual_robotics_tool):
    """Test getting status for non-existent robot."""
    result = await virtual_robotics_tool._get_status("nonexistent")

    assert result["status"] == "error"
    assert "not found" in result["message"].lower()


@pytest.mark.asyncio
async def test_set_scale(virtual_robotics_tool, state_manager):
    """Test setting robot scale."""
    state_manager.register_robot("test_scout", "scout", platform="unity")

    result = await virtual_robotics_tool._set_scale("test_scout", scale=2.0)

    assert result["status"] == "success"
    assert result["scale"] == 2.0

