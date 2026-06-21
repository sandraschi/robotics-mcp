"""Unit tests for robot_navigation tool."""

from unittest.mock import MagicMock

import pytest

from robotics_mcp.tools.robot_navigation import RobotNavigationTool
from robotics_mcp.utils.state_manager import RobotStateManager
from tests.helpers import patch_fastmcp_client


@pytest.fixture
def state_manager():
    return RobotStateManager()


@pytest.fixture
def mock_mounted_servers():
    return {"unity": MagicMock()}


@pytest.fixture
def robot_navigation_tool(mock_mcp, state_manager, mock_mounted_servers):
    tool = RobotNavigationTool(mock_mcp, state_manager, mock_mounted_servers)
    tool.register()
    return tool


@pytest.fixture
def registered_robot(state_manager):
    return state_manager.register_robot("test_robot_01", "scout", platform="unity")


@pytest.mark.asyncio
async def test_robot_navigation_robot_not_found(robot_navigation_tool):
    tool_func = robot_navigation_tool.mcp.tool.registered_func
    result = await tool_func(
        robot_id="nonexistent",
        action="plan_path",
        goal_position={"x": 1.0, "y": 0.0, "z": 1.0},
    )
    assert result["status"] == "error"
    assert "not found" in result.get("message", result.get("error", "")).lower()


@pytest.mark.asyncio
async def test_robot_navigation_plan_path(robot_navigation_tool, registered_robot):
    tool_func = robot_navigation_tool.mcp.tool.registered_func
    with patch_fastmcp_client("robotics_mcp.tools.robot_navigation"):
        result = await tool_func(
            robot_id="test_robot_01",
            action="plan_path",
            goal_position={"x": 1.0, "y": 0.0, "z": 1.0},
        )
    assert result["status"] == "success"


@pytest.mark.asyncio
async def test_robot_navigation_follow_path(robot_navigation_tool, registered_robot):
    tool_func = robot_navigation_tool.mcp.tool.registered_func
    with patch_fastmcp_client("robotics_mcp.tools.robot_navigation"):
        result = await tool_func(robot_id="test_robot_01", action="follow_path", path_id="path_01")
    assert result["status"] == "success"


@pytest.mark.asyncio
async def test_robot_navigation_stop(robot_navigation_tool, registered_robot):
    tool_func = robot_navigation_tool.mcp.tool.registered_func
    with patch_fastmcp_client("robotics_mcp.tools.robot_navigation"):
        result = await tool_func(robot_id="test_robot_01", action="clear_waypoints")
    assert result["status"] == "success"


@pytest.mark.asyncio
async def test_robot_navigation_get_status(robot_navigation_tool, registered_robot):
    tool_func = robot_navigation_tool.mcp.tool.registered_func
    with patch_fastmcp_client("robotics_mcp.tools.robot_navigation"):
        result = await tool_func(robot_id="test_robot_01", action="get_path_status")
    assert result["status"] == "success"


@pytest.mark.asyncio
async def test_robot_navigation_set_waypoint(robot_navigation_tool, registered_robot):
    tool_func = robot_navigation_tool.mcp.tool.registered_func
    with patch_fastmcp_client("robotics_mcp.tools.robot_navigation"):
        result = await tool_func(
            robot_id="test_robot_01",
            action="set_waypoint",
            waypoint={"x": 2.0, "y": 0.0, "z": 2.0},
        )
    assert result["status"] == "success"
