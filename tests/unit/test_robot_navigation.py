"""Unit tests for robot_navigation tool."""

import pytest
from unittest.mock import AsyncMock, MagicMock, patch

from robotics_mcp.tools.robot_navigation import RobotNavigationTool
from robotics_mcp.utils.state_manager import RobotStateManager


@pytest.fixture
def mock_mcp():
    """Create mock MCP server."""
    mcp = MagicMock()
    mcp.tool = MagicMock(return_value=lambda f: f)
    return mcp


@pytest.fixture
def state_manager():
    """Create robot state manager."""
    return RobotStateManager()


@pytest.fixture
def mock_mounted_servers():
    """Create mock mounted servers."""
    return {"unity": MagicMock()}


@pytest.fixture
def robot_navigation_tool(mock_mcp, state_manager, mock_mounted_servers):
    """Create robot navigation tool instance."""
    tool = RobotNavigationTool(mock_mcp, state_manager, mock_mounted_servers)
    tool.register()
    return tool


@pytest.fixture
def registered_robot(state_manager):
    """Register a test robot."""
    return state_manager.register_robot("test_robot_01", "scout", platform="unity")


@pytest.mark.asyncio
async def test_robot_navigation_robot_not_found(robot_navigation_tool):
    """Test navigation operation on non-existent robot."""
    tool_func = robot_navigation_tool.mcp.tool.call_args[0][0]
    
    result = await tool_func(
        robot_id="nonexistent",
        operation="plan_path",
        target_position={"x": 1.0, "y": 0.0, "z": 1.0},
    )
    
    assert result["success"] is False
    assert "not found" in result["error"].lower()


@pytest.mark.asyncio
async def test_robot_navigation_plan_path(robot_navigation_tool, registered_robot):
    """Test path planning."""
    tool_func = robot_navigation_tool.mcp.tool.call_args[0][0]
    
    with patch("robotics_mcp.tools.robot_navigation.Client") as mock_client:
        mock_client_instance = AsyncMock()
        mock_client.return_value.__aenter__.return_value = mock_client_instance
        mock_client_instance.call_tool = AsyncMock(return_value={"path": [{"x": 0, "y": 0, "z": 0}, {"x": 1, "y": 0, "z": 1}]})
        
        result = await tool_func(
            robot_id="test_robot_01",
            operation="plan_path",
            target_position={"x": 1.0, "y": 0.0, "z": 1.0},
        )
        
        assert result["success"] is True


@pytest.mark.asyncio
async def test_robot_navigation_follow_path(robot_navigation_tool, registered_robot):
    """Test following a path."""
    tool_func = robot_navigation_tool.mcp.tool.call_args[0][0]
    
    with patch("robotics_mcp.tools.robot_navigation.Client") as mock_client:
        mock_client_instance = AsyncMock()
        mock_client.return_value.__aenter__.return_value = mock_client_instance
        mock_client_instance.call_tool = AsyncMock(return_value={"success": True})
        
        result = await tool_func(
            robot_id="test_robot_01",
            operation="follow_path",
            speed=1.0,
        )
        
        assert result["success"] is True


@pytest.mark.asyncio
async def test_robot_navigation_stop(robot_navigation_tool, registered_robot):
    """Test stopping navigation."""
    tool_func = robot_navigation_tool.mcp.tool.call_args[0][0]
    
    with patch("robotics_mcp.tools.robot_navigation.Client") as mock_client:
        mock_client_instance = AsyncMock()
        mock_client.return_value.__aenter__.return_value = mock_client_instance
        mock_client_instance.call_tool = AsyncMock(return_value={"success": True})
        
        result = await tool_func(
            robot_id="test_robot_01",
            operation="stop_navigation",
        )
        
        assert result["success"] is True


@pytest.mark.asyncio
async def test_robot_navigation_get_status(robot_navigation_tool, registered_robot):
    """Test getting navigation status."""
    tool_func = robot_navigation_tool.mcp.tool.call_args[0][0]
    
    with patch("robotics_mcp.tools.robot_navigation.Client") as mock_client:
        mock_client_instance = AsyncMock()
        mock_client.return_value.__aenter__.return_value = mock_client_instance
        mock_client_instance.call_tool = AsyncMock(return_value={"status": "idle", "progress": 0.0})
        
        result = await tool_func(
            robot_id="test_robot_01",
            operation="get_navigation_status",
        )
        
        assert result["success"] is True


@pytest.mark.asyncio
async def test_robot_navigation_set_waypoint(robot_navigation_tool, registered_robot):
    """Test setting waypoint."""
    tool_func = robot_navigation_tool.mcp.tool.call_args[0][0]
    
    with patch("robotics_mcp.tools.robot_navigation.Client") as mock_client:
        mock_client_instance = AsyncMock()
        mock_client.return_value.__aenter__.return_value = mock_client_instance
        mock_client_instance.call_tool = AsyncMock(return_value={"success": True})
        
        result = await tool_func(
            robot_id="test_robot_01",
            operation="set_waypoint",
            target_position={"x": 2.0, "y": 0.0, "z": 2.0},
        )
        
        assert result["success"] is True

