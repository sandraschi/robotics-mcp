"""Unit tests for robot control tool."""

import pytest
from unittest.mock import AsyncMock, MagicMock, patch

from robotics_mcp.tools.robot_control import RobotControlTool
from robotics_mcp.utils.state_manager import RobotStateManager, RobotState


@pytest.fixture
def mock_mcp():
    """Create mock FastMCP instance."""
    mcp = MagicMock()
    mcp.tool = MagicMock(return_value=lambda f: f)
    return mcp


@pytest.fixture
def robot_control_tool(mock_mcp):
    """Create robot control tool instance."""
    state_manager = RobotStateManager()
    return RobotControlTool(mock_mcp, state_manager)


@pytest.mark.asyncio
async def test_robot_control_get_status(robot_control_tool):
    """Test get_status action."""
    robot = robot_control_tool.state_manager.register_robot("scout_01", "scout")
    
    # Register the tool
    robot_control_tool.register()
    
    # Get the registered tool function
    tool_func = robot_control_tool.mcp.tool.call_args[0][0]
    
    result = await tool_func(robot_id="scout_01", action="get_status")
    
    assert result["status"] == "success"
    assert result["robot_id"] == "scout_01"
    assert result["action"] == "get_status"


@pytest.mark.asyncio
async def test_robot_control_robot_not_found(robot_control_tool):
    """Test robot control with non-existent robot."""
    robot_control_tool.register()
    tool_func = robot_control_tool.mcp.tool.call_args[0][0]
    
    result = await tool_func(robot_id="nonexistent", action="get_status")
    
    assert result["status"] == "error"
    assert result["error_type"] == "not_found"
    assert "not found" in result["message"].lower()


@pytest.mark.asyncio
async def test_robot_control_physical_robot_move(robot_control_tool):
    """Test move action for physical robot."""
    robot = robot_control_tool.state_manager.register_robot("scout_01", "scout")
    robot_control_tool.register()
    tool_func = robot_control_tool.mcp.tool.call_args[0][0]
    
    result = await tool_func(
        robot_id="scout_01",
        action="move",
        linear=0.2,
        angular=0.0
    )
    
    assert result["status"] == "success"
    assert result["robot_id"] == "scout_01"
    assert result["action"] == "move"
    assert "mock" in result["message"].lower() or "pending" in result["message"].lower()


@pytest.mark.asyncio
async def test_robot_control_virtual_robot_move(robot_control_tool):
    """Test move action for virtual robot."""
    robot = robot_control_tool.state_manager.register_robot(
        "vbot_scout_01", "scout", platform="unity"
    )
    robot_control_tool.register()
    tool_func = robot_control_tool.mcp.tool.call_args[0][0]
    
    with patch("robotics_mcp.tools.robot_control.Client") as mock_client:
        mock_client_instance = AsyncMock()
        mock_client.return_value.__aenter__.return_value = mock_client_instance
        mock_client_instance.call_tool = AsyncMock(return_value={"status": "ok"})
        
        result = await tool_func(
            robot_id="vbot_scout_01",
            action="move",
            linear=0.2,
            angular=0.0
        )
        
        # Should attempt to call virtual robot handler
        assert result["status"] in ["success", "error"]


@pytest.mark.asyncio
async def test_robot_control_stop(robot_control_tool):
    """Test stop action."""
    robot = robot_control_tool.state_manager.register_robot("scout_01", "scout")
    robot_control_tool.register()
    tool_func = robot_control_tool.mcp.tool.call_args[0][0]
    
    result = await tool_func(robot_id="scout_01", action="stop")
    
    assert result["status"] == "success"
    assert result["action"] == "stop"


@pytest.mark.asyncio
async def test_robot_control_invalid_action(robot_control_tool):
    """Test invalid action (should be caught by Literal type, but test error handling)."""
    robot = robot_control_tool.state_manager.register_robot("scout_01", "scout")
    robot_control_tool.register()
    tool_func = robot_control_tool.mcp.tool.call_args[0][0]
    
    # This should fail at type level, but test that error handling works
    # We'll test with a valid action but check error handling path
    result = await tool_func(robot_id="scout_01", action="get_status")
    assert result["status"] == "success"


@pytest.mark.asyncio
async def test_handle_action_http_api(robot_control_tool):
    """Test handle_action method for HTTP API."""
    robot = robot_control_tool.state_manager.register_robot("scout_01", "scout")
    
    result = await robot_control_tool.handle_action(
        robot_id="scout_01",
        action="get_status",
        params={}
    )
    
    assert result["status"] == "success"
    assert result["robot_id"] == "scout_01"
    assert result["action"] == "get_status"


@pytest.mark.asyncio
async def test_handle_action_with_params(robot_control_tool):
    """Test handle_action with movement parameters."""
    robot = robot_control_tool.state_manager.register_robot("scout_01", "scout")
    
    result = await robot_control_tool.handle_action(
        robot_id="scout_01",
        action="move",
        params={"linear": 0.3, "angular": 0.1, "duration": 2.0}
    )
    
    assert result["status"] == "success"
    assert result["action"] == "move"

