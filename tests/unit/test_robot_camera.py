"""Unit tests for robot_camera tool."""

import pytest
from unittest.mock import AsyncMock, MagicMock, patch

from robotics_mcp.tools.robot_camera import RobotCameraTool
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
def robot_camera_tool(mock_mcp, state_manager, mock_mounted_servers):
    """Create robot camera tool instance."""
    tool = RobotCameraTool(mock_mcp, state_manager, mock_mounted_servers)
    tool.register()
    return tool


@pytest.fixture
def registered_robot(state_manager):
    """Register a test robot."""
    return state_manager.register_robot("test_robot_01", "scout", platform="unity")


@pytest.mark.asyncio
async def test_robot_camera_robot_not_found(robot_camera_tool):
    """Test camera operation on non-existent robot."""
    tool_func = robot_camera_tool.mcp.tool.call_args[0][0]
    
    result = await tool_func(
        robot_id="nonexistent",
        operation="get_camera_status",
    )
    
    assert result["success"] is False
    assert "not found" in result["error"].lower()


@pytest.mark.asyncio
async def test_robot_camera_get_feed(robot_camera_tool, registered_robot):
    """Test getting camera feed."""
    tool_func = robot_camera_tool.mcp.tool.call_args[0][0]
    
    with patch("robotics_mcp.tools.robot_camera.Client") as mock_client:
        mock_client_instance = AsyncMock()
        mock_client.return_value.__aenter__.return_value = mock_client_instance
        mock_client_instance.call_tool = AsyncMock(return_value={"feed_url": "http://localhost:8080/feed"})
        
        result = await tool_func(
            robot_id="test_robot_01",
            operation="get_feed",
        )
        
        assert result["success"] is True


@pytest.mark.asyncio
async def test_robot_camera_capture_image(robot_camera_tool, registered_robot):
    """Test capturing image."""
    tool_func = robot_camera_tool.mcp.tool.call_args[0][0]
    
    with patch("robotics_mcp.tools.robot_camera.Client") as mock_client:
        mock_client_instance = AsyncMock()
        mock_client.return_value.__aenter__.return_value = mock_client_instance
        mock_client_instance.call_tool = AsyncMock(return_value={"image_path": "/tmp/capture.jpg"})
        
        result = await tool_func(
            robot_id="test_robot_01",
            operation="capture_image",
            format="jpeg",
            output_path="/tmp/capture.jpg",
        )
        
        assert result["success"] is True


@pytest.mark.asyncio
async def test_robot_camera_set_angle(robot_camera_tool, registered_robot):
    """Test setting camera angle."""
    tool_func = robot_camera_tool.mcp.tool.call_args[0][0]
    
    with patch("robotics_mcp.tools.robot_camera.Client") as mock_client:
        mock_client_instance = AsyncMock()
        mock_client.return_value.__aenter__.return_value = mock_client_instance
        mock_client_instance.call_tool = AsyncMock(return_value={"success": True})
        
        result = await tool_func(
            robot_id="test_robot_01",
            operation="set_camera_angle",
            pitch=30.0,
            yaw=45.0,
        )
        
        assert result["success"] is True


@pytest.mark.asyncio
async def test_robot_camera_get_status(robot_camera_tool, registered_robot):
    """Test getting camera status."""
    tool_func = robot_camera_tool.mcp.tool.call_args[0][0]
    
    with patch("robotics_mcp.tools.robot_camera.Client") as mock_client:
        mock_client_instance = AsyncMock()
        mock_client.return_value.__aenter__.return_value = mock_client_instance
        mock_client_instance.call_tool = AsyncMock(return_value={"pitch": 0.0, "yaw": 0.0, "resolution": "1920x1080"})
        
        result = await tool_func(
            robot_id="test_robot_01",
            operation="get_camera_status",
        )
        
        assert result["success"] is True

