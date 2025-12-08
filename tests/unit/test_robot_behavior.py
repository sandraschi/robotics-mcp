"""Unit tests for robot_behavior tool."""

import pytest
from unittest.mock import AsyncMock, MagicMock, patch

from robotics_mcp.tools.robot_behavior import RobotBehaviorTool
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
    return {
        "unity": MagicMock(),
    }


@pytest.fixture
def robot_behavior_tool(mock_mcp, state_manager, mock_mounted_servers):
    """Create robot behavior tool instance."""
    tool = RobotBehaviorTool(mock_mcp, state_manager, mock_mounted_servers)
    tool.register()
    return tool


@pytest.fixture
def registered_robot(state_manager):
    """Register a test robot."""
    return state_manager.register_robot("test_robot_01", "scout", platform="unity")


@pytest.mark.asyncio
async def test_robot_behavior_robot_not_found(robot_behavior_tool):
    """Test behavior operation on non-existent robot."""
    tool_func = robot_behavior_tool.mcp.tool.call_args[0][0]
    
    result = await tool_func(
        robot_id="nonexistent",
        category="animation",
        action="get_animation_state",
    )
    
    assert result["success"] is False
    assert "not found" in result["error"].lower()


@pytest.mark.asyncio
async def test_robot_behavior_animation_animate_wheels(robot_behavior_tool, registered_robot):
    """Test animation: animate_wheels."""
    tool_func = robot_behavior_tool.mcp.tool.call_args[0][0]
    
    with patch("robotics_mcp.tools.robot_behavior.call_mounted_server_tool") as mock_call:
        mock_call.return_value = {"success": True}
        
        result = await tool_func(
            robot_id="test_robot_01",
            category="animation",
            action="animate_wheels",
            wheel_speeds={"front_left": 1.0, "front_right": 1.0, "back_left": 1.0, "back_right": 1.0},
        )
        
        assert result["success"] is True


@pytest.mark.asyncio
async def test_robot_behavior_animation_play(robot_behavior_tool, registered_robot):
    """Test animation: play_animation."""
    tool_func = robot_behavior_tool.mcp.tool.call_args[0][0]
    
    with patch("robotics_mcp.tools.robot_behavior.call_mounted_server_tool") as mock_call:
        mock_call.return_value = {"success": True}
        
        result = await tool_func(
            robot_id="test_robot_01",
            category="animation",
            action="play_animation",
            animation_name="walk",
            animation_speed=1.0,
            loop=False,
        )
        
        assert result["success"] is True


@pytest.mark.asyncio
async def test_robot_behavior_camera_get_feed(robot_behavior_tool, registered_robot):
    """Test camera: get_camera_feed."""
    tool_func = robot_behavior_tool.mcp.tool.call_args[0][0]
    
    with patch("robotics_mcp.tools.robot_behavior.call_mounted_server_tool") as mock_call:
        mock_call.return_value = {"feed_url": "http://localhost:8080/feed"}
        
        result = await tool_func(
            robot_id="test_robot_01",
            category="camera",
            action="get_camera_feed",
        )
        
        assert result["success"] is True


@pytest.mark.asyncio
async def test_robot_behavior_camera_capture(robot_behavior_tool, registered_robot):
    """Test camera: capture_image."""
    tool_func = robot_behavior_tool.mcp.tool.call_args[0][0]
    
    with patch("robotics_mcp.tools.robot_behavior.call_mounted_server_tool") as mock_call:
        mock_call.return_value = {"image_path": "/tmp/capture.jpg"}
        
        result = await tool_func(
            robot_id="test_robot_01",
            category="camera",
            action="capture_image",
            output_path="/tmp/capture.jpg",
        )
        
        assert result["success"] is True


@pytest.mark.asyncio
async def test_robot_behavior_navigation_plan_path(robot_behavior_tool, registered_robot):
    """Test navigation: plan_path."""
    tool_func = robot_behavior_tool.mcp.tool.call_args[0][0]
    
    with patch("robotics_mcp.tools.robot_behavior.call_mounted_server_tool") as mock_call:
        mock_call.return_value = {"path": [{"x": 0, "y": 0, "z": 0}, {"x": 1, "y": 0, "z": 1}]}
        
        result = await tool_func(
            robot_id="test_robot_01",
            category="navigation",
            action="plan_path",
            start_position={"x": 0.0, "y": 0.0, "z": 0.0},
            goal_position={"x": 1.0, "y": 0.0, "z": 1.0},
        )
        
        assert result["success"] is True


@pytest.mark.asyncio
async def test_robot_behavior_navigation_follow_path(robot_behavior_tool, registered_robot):
    """Test navigation: follow_path."""
    tool_func = robot_behavior_tool.mcp.tool.call_args[0][0]
    
    with patch("robotics_mcp.tools.robot_behavior.call_mounted_server_tool") as mock_call:
        mock_call.return_value = {"success": True}
        
        result = await tool_func(
            robot_id="test_robot_01",
            category="navigation",
            action="follow_path",
            path_id="path_123",
        )
        
        assert result["success"] is True


@pytest.mark.asyncio
async def test_robot_behavior_manipulation_set_joint(robot_behavior_tool, registered_robot):
    """Test manipulation: set_joint_positions."""
    tool_func = robot_behavior_tool.mcp.tool.call_args[0][0]
    
    with patch("robotics_mcp.tools.robot_behavior.call_mounted_server_tool") as mock_call:
        mock_call.return_value = {"success": True}
        
        result = await tool_func(
            robot_id="test_robot_01",
            category="manipulation",
            action="set_joint_positions",
            joint_positions={"shoulder": 45.0, "elbow": 90.0},
        )
        
        assert result["success"] is True


@pytest.mark.asyncio
async def test_robot_behavior_invalid_category(robot_behavior_tool, registered_robot):
    """Test invalid category."""
    tool_func = robot_behavior_tool.mcp.tool.call_args[0][0]
    
    result = await tool_func(
        robot_id="test_robot_01",
        category="invalid_category",
        action="some_action",
    )
    
    assert result["success"] is False
    assert "invalid" in result["error"].lower() or "unknown" in result["error"].lower()


@pytest.mark.asyncio
async def test_robot_behavior_invalid_action(robot_behavior_tool, registered_robot):
    """Test invalid action."""
    tool_func = robot_behavior_tool.mcp.tool.call_args[0][0]
    
    result = await tool_func(
        robot_id="test_robot_01",
        category="animation",
        action="invalid_action",
    )
    
    assert result["success"] is False
    assert "invalid" in result["error"].lower() or "unknown" in result["error"].lower()

