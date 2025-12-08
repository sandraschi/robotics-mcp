"""Unit tests for robot_animation tool."""

import pytest
from unittest.mock import AsyncMock, MagicMock, patch

from robotics_mcp.tools.robot_animation import RobotAnimationTool
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
def robot_animation_tool(mock_mcp, state_manager, mock_mounted_servers):
    """Create robot animation tool instance."""
    tool = RobotAnimationTool(mock_mcp, state_manager, mock_mounted_servers)
    tool.register()
    return tool


@pytest.fixture
def registered_robot(state_manager):
    """Register a test robot."""
    return state_manager.register_robot("test_robot_01", "scout", platform="unity")


@pytest.mark.asyncio
async def test_robot_animation_robot_not_found(robot_animation_tool):
    """Test animation operation on non-existent robot."""
    tool_func = robot_animation_tool.mcp.tool.call_args[0][0]
    
    result = await tool_func(
        robot_id="nonexistent",
        action="get_animation_state",
    )
    
    assert result["success"] is False
    assert "not found" in result["error"].lower()


@pytest.mark.asyncio
async def test_robot_animation_get_state(robot_animation_tool, registered_robot):
    """Test getting animation state."""
    tool_func = robot_animation_tool.mcp.tool.call_args[0][0]
    
    with patch("robotics_mcp.tools.robot_animation.Client") as mock_client:
        mock_client_instance = AsyncMock()
        mock_client.return_value.__aenter__.return_value = mock_client_instance
        mock_client_instance.call_tool = AsyncMock(return_value={"state": "idle"})
        
        result = await tool_func(
            robot_id="test_robot_01",
            action="get_animation_state",
        )
        
        assert result["success"] is True


@pytest.mark.asyncio
async def test_robot_animation_animate_wheels(robot_animation_tool, registered_robot):
    """Test animating wheels."""
    tool_func = robot_animation_tool.mcp.tool.call_args[0][0]
    
    with patch("robotics_mcp.tools.robot_animation.Client") as mock_client:
        mock_client_instance = AsyncMock()
        mock_client.return_value.__aenter__.return_value = mock_client_instance
        mock_client_instance.call_tool = AsyncMock(return_value={"success": True})
        
        result = await tool_func(
            robot_id="test_robot_01",
            action="animate_wheels",
            wheel_speeds={"front_left": 1.0, "front_right": 1.0, "back_left": 1.0, "back_right": 1.0},
        )
        
        assert result["success"] is True


@pytest.mark.asyncio
async def test_robot_animation_play_animation(robot_animation_tool, registered_robot):
    """Test playing animation."""
    tool_func = robot_animation_tool.mcp.tool.call_args[0][0]
    
    with patch("robotics_mcp.tools.robot_animation.Client") as mock_client:
        mock_client_instance = AsyncMock()
        mock_client.return_value.__aenter__.return_value = mock_client_instance
        mock_client_instance.call_tool = AsyncMock(return_value={"success": True})
        
        result = await tool_func(
            robot_id="test_robot_01",
            action="play_animation",
            animation_name="walk",
            speed=1.0,
            loop=False,
        )
        
        assert result["success"] is True


@pytest.mark.asyncio
async def test_robot_animation_set_pose(robot_animation_tool, registered_robot):
    """Test setting robot pose."""
    tool_func = robot_animation_tool.mcp.tool.call_args[0][0]
    
    with patch("robotics_mcp.tools.robot_animation.Client") as mock_client:
        mock_client_instance = AsyncMock()
        mock_client.return_value.__aenter__.return_value = mock_client_instance
        mock_client_instance.call_tool = AsyncMock(return_value={"success": True})
        
        result = await tool_func(
            robot_id="test_robot_01",
            action="set_pose",
            pose="sit",
        )
        
        assert result["success"] is True


@pytest.mark.asyncio
async def test_robot_animation_stop_animation(robot_animation_tool, registered_robot):
    """Test stopping animation."""
    tool_func = robot_animation_tool.mcp.tool.call_args[0][0]
    
    with patch("robotics_mcp.tools.robot_animation.Client") as mock_client:
        mock_client_instance = AsyncMock()
        mock_client.return_value.__aenter__.return_value = mock_client_instance
        mock_client_instance.call_tool = AsyncMock(return_value={"success": True})
        
        result = await tool_func(
            robot_id="test_robot_01",
            action="stop_animation",
        )
        
        assert result["success"] is True


@pytest.mark.asyncio
async def test_robot_animation_mock_mode(robot_animation_tool, state_manager):
    """Test animation in mock mode (no Unity)."""
    # Register physical robot (no Unity)
    state_manager.register_robot("physical_robot", "scout", platform=None)
    
    tool_func = robot_animation_tool.mcp.tool.call_args[0][0]
    
    result = await tool_func(
        robot_id="physical_robot",
        action="get_animation_state",
    )
    
    # Should return mock response
    assert result["success"] is True
    assert "mock" in result["data"].get("note", "").lower()

