"""Unit tests for robot_camera tool."""

from unittest.mock import MagicMock

import pytest

from robotics_mcp.tools.robot_camera import RobotCameraTool
from robotics_mcp.utils.state_manager import RobotStateManager
from tests.helpers import patch_fastmcp_client


@pytest.fixture
def state_manager():
    return RobotStateManager()


@pytest.fixture
def mock_mounted_servers():
    return {"unity": MagicMock()}


@pytest.fixture
def robot_camera_tool(mock_mcp, state_manager, mock_mounted_servers):
    tool = RobotCameraTool(mock_mcp, state_manager, mock_mounted_servers)
    tool.register()
    return tool


@pytest.fixture
def registered_robot(state_manager):
    return state_manager.register_robot("test_robot_01", "scout", platform="unity")


@pytest.mark.asyncio
async def test_robot_camera_robot_not_found(robot_camera_tool):
    tool_func = robot_camera_tool.mcp.tool.registered_func
    result = await tool_func(robot_id="nonexistent", action="get_camera_status")
    assert result["status"] == "error"
    assert "not found" in result.get("message", result.get("error", "")).lower()


@pytest.mark.asyncio
async def test_robot_camera_get_feed(robot_camera_tool, registered_robot):
    tool_func = robot_camera_tool.mcp.tool.registered_func
    with patch_fastmcp_client("robotics_mcp.tools.robot_camera"):
        result = await tool_func(robot_id="test_robot_01", action="get_virtual_camera")
    assert result["status"] == "success"


@pytest.mark.asyncio
async def test_robot_camera_capture_image(robot_camera_tool, registered_robot):
    tool_func = robot_camera_tool.mcp.tool.registered_func
    with patch_fastmcp_client("robotics_mcp.tools.robot_camera"):
        result = await tool_func(
            robot_id="test_robot_01",
            action="capture_image",
            output_path="/tmp/capture.jpg",
        )
    assert result["status"] == "success"


@pytest.mark.asyncio
async def test_robot_camera_set_angle(robot_camera_tool, registered_robot):
    tool_func = robot_camera_tool.mcp.tool.registered_func
    with patch_fastmcp_client("robotics_mcp.tools.robot_camera"):
        result = await tool_func(
            robot_id="test_robot_01",
            action="set_camera_angle",
            angle_x=30.0,
            angle_y=45.0,
        )
    assert result["status"] == "success"


@pytest.mark.asyncio
async def test_robot_camera_get_status(robot_camera_tool, registered_robot):
    tool_func = robot_camera_tool.mcp.tool.registered_func
    with patch_fastmcp_client("robotics_mcp.tools.robot_camera"):
        result = await tool_func(robot_id="test_robot_01", action="get_camera_status")
    assert result["status"] == "success"
