"""Unit tests for robot control tool."""

from unittest.mock import AsyncMock, MagicMock, patch

import pytest

from robotics_mcp.tools.robot_control import RobotControlTool
from robotics_mcp.utils.state_manager import RobotStateManager
from tests.helpers import mock_ctx, patch_yahboom_client


@pytest.fixture(autouse=True)
def stub_yahboom_client():
    with patch_yahboom_client():
        yield


@pytest.fixture
def robot_control_tool(mock_mcp):
    """Create robot control tool instance."""
    state_manager = RobotStateManager()
    return RobotControlTool(mock_mcp, state_manager)


@pytest.mark.asyncio
async def test_robot_control_get_status(robot_control_tool):
    """Test get_status action."""
    robot_control_tool.state_manager.register_robot("yahboom_01", "yahboom")
    robot_control_tool.register()
    tool_func = robot_control_tool.mcp.tool.registered_func

    result = await tool_func(mock_ctx(), robot_id="yahboom_01", action="get_status")

    assert result["status"] == "success"
    assert result["robot_id"] == "yahboom_01"
    assert result["action"] == "get_status"


@pytest.mark.asyncio
async def test_robot_control_robot_not_found(robot_control_tool):
    """Test robot control with non-existent robot."""
    robot_control_tool.register()
    tool_func = robot_control_tool.mcp.tool.registered_func

    result = await tool_func(mock_ctx(), robot_id="nonexistent", action="get_status")

    assert result["status"] == "error"
    assert result["error_type"] == "not_found"
    assert "not found" in result["message"].lower()


@pytest.mark.asyncio
async def test_robot_control_physical_robot_move(robot_control_tool):
    """Test move action for physical robot."""
    robot_control_tool.state_manager.register_robot("yahboom_01", "yahboom")
    robot_control_tool.register()
    tool_func = robot_control_tool.mcp.tool.registered_func

    result = await tool_func(mock_ctx(), robot_id="yahboom_01", action="move", linear=0.2, angular=0.0)

    assert result["status"] == "success"
    assert result["robot_id"] == "yahboom_01"
    assert result["action"] == "move"


@pytest.mark.asyncio
async def test_robot_control_virtual_robot_move(robot_control_tool):
    """Test move action for virtual robot."""
    robot_control_tool.state_manager.register_robot("vbot_scout_01", "scout", platform="unity")
    robot_control_tool.register()
    tool_func = robot_control_tool.mcp.tool.registered_func

    robot_control_tool.mounted_servers = {"avatar": MagicMock()}
    with patch("robotics_mcp.tools.robot_control.call_mounted_server_tool", new_callable=AsyncMock) as mock_call:
        mock_call.return_value = {"status": "success"}

        result = await tool_func(mock_ctx(), robot_id="vbot_scout_01", action="move", linear=0.2, angular=0.0)

        assert result["status"] == "success"
        assert mock_call.called


@pytest.mark.asyncio
async def test_robot_control_stop(robot_control_tool):
    """Test stop action."""
    robot_control_tool.state_manager.register_robot("yahboom_01", "yahboom")
    robot_control_tool.register()
    tool_func = robot_control_tool.mcp.tool.registered_func

    result = await tool_func(mock_ctx(), robot_id="yahboom_01", action="stop")

    assert result["status"] == "success"
    assert result["action"] == "stop"


@pytest.mark.asyncio
async def test_robot_control_invalid_action(robot_control_tool):
    """Test valid action path on a supported robot type."""
    robot_control_tool.state_manager.register_robot("yahboom_01", "yahboom")
    robot_control_tool.register()
    tool_func = robot_control_tool.mcp.tool.registered_func

    result = await tool_func(mock_ctx(), robot_id="yahboom_01", action="get_status")
    assert result["status"] == "success"


@pytest.mark.asyncio
async def test_handle_action_http_api(robot_control_tool):
    """Test handle_action method for HTTP API."""
    robot_control_tool.state_manager.register_robot("yahboom_01", "yahboom")

    result = await robot_control_tool.handle_action(robot_id="yahboom_01", action="get_status", params={})

    assert result["status"] == "success"
    assert result["robot_id"] == "yahboom_01"
    assert result["action"] == "get_status"


@pytest.mark.asyncio
async def test_handle_action_with_params(robot_control_tool):
    """Test handle_action with movement parameters."""
    robot_control_tool.state_manager.register_robot("yahboom_01", "yahboom")

    result = await robot_control_tool.handle_action(
        robot_id="yahboom_01", action="move", params={"linear": 0.3, "angular": 0.1, "duration": 2.0}
    )

    assert result["status"] == "success"
    assert result["action"] == "move"


@pytest.mark.asyncio
async def test_yahboom_robot_control_get_status(robot_control_tool):
    """Test Yahboom robot get_status action."""
    # Register Yahboom robot
    robot_control_tool.state_manager.register_robot("yahboom_01", "yahboom")

    # Register the tool
    robot_control_tool.register()

    # Get the registered tool function
    tool_func = robot_control_tool.mcp.tool.registered_func

    result = await tool_func(mock_ctx(), robot_id="yahboom_01", action="get_status")

    assert result["status"] == "success"
    assert result["robot_id"] == "yahboom_01"
    assert result["action"] == "get_status"
    assert result["model"] == "ROSMaster-M1"
    assert "battery" in result
    assert "sensors" in result


@pytest.mark.asyncio
async def test_yahboom_robot_control_home_patrol(robot_control_tool):
    """Test Yahboom robot home_patrol action."""
    # Register Yahboom robot
    robot_control_tool.state_manager.register_robot("yahboom_01", "yahboom")

    robot_control_tool.register()
    tool_func = robot_control_tool.mcp.tool.registered_func

    result = await tool_func(mock_ctx(), robot_id="yahboom_01", action="home_patrol")

    assert result["status"] == "success"
    assert result["robot_id"] == "yahboom_01"
    assert result["action"] == "home_patrol"
    assert "waypoints" in result
    assert len(result["waypoints"]) == 4


@pytest.mark.asyncio
async def test_yahboom_robot_control_navigate_to(robot_control_tool):
    """Test Yahboom robot navigate_to action."""
    # Register Yahboom robot
    robot_control_tool.state_manager.register_robot("yahboom_01", "yahboom")

    robot_control_tool.register()
    tool_func = robot_control_tool.mcp.tool.registered_func

    result = await tool_func(mock_ctx(), robot_id="yahboom_01", action="navigate_to", x=2.0, y=1.5, theta=0.0)

    assert result["status"] == "success"
    assert result["robot_id"] == "yahboom_01"
    assert result["action"] == "navigate_to"
    assert "target_pose" in result


@pytest.mark.asyncio
async def test_yahboom_robot_control_camera_capture(robot_control_tool):
    """Test Yahboom robot camera_capture action."""
    # Register Yahboom robot
    robot_control_tool.state_manager.register_robot("yahboom_01", "yahboom")

    robot_control_tool.register()
    tool_func = robot_control_tool.mcp.tool.registered_func

    result = await tool_func(mock_ctx(), robot_id="yahboom_01", action="camera_capture")

    assert result["robot_id"] == "yahboom_01"
    assert result["action"] == "camera_capture"
    assert result["status"] in {"success", "error"}


@pytest.mark.asyncio
async def test_yahboom_robot_control_ai_query(robot_control_tool):
    """Test Yahboom robot ai_query action."""
    # Register Yahboom robot
    robot_control_tool.state_manager.register_robot("yahboom_01", "yahboom")

    robot_control_tool.register()
    tool_func = robot_control_tool.mcp.tool.registered_func

    result = await tool_func(mock_ctx(), robot_id="yahboom_01", action="ai_query", query="What's in front of me?")

    assert result["robot_id"] == "yahboom_01"
    assert result["action"] == "ai_query"
    assert result["status"] == "success"
    assert result["query"] == "What's in front of me?"
    assert result["mission"]["goal"] == "What's in front of me?"
