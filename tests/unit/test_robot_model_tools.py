"""Unit tests for robot_model_tools."""

import pytest

from robotics_mcp.tools.robot_model_tools import RobotModelTools
from robotics_mcp.utils.state_manager import RobotStateManager
from tests.helpers import mock_ctx


@pytest.fixture
def state_manager():
    return RobotStateManager()


@pytest.fixture
def mock_mounted_servers():
    return {}


@pytest.fixture
def robot_model_tools(mock_mcp, state_manager, mock_mounted_servers):
    tool = RobotModelTools(mock_mcp, state_manager, mock_mounted_servers)
    tool.register()
    return tool


@pytest.mark.asyncio
async def test_robot_model_create_invalid_type(robot_model_tools):
    tool_func = robot_model_tools.mcp.tool.registered_func
    result = await tool_func(
        mock_ctx(),
        operation="create",
        robot_type="invalid_type",
        output_path="/tmp/test.fbx",
    )
    assert result["status"] == "error"
    msg = result.get("message", result.get("error", "")).lower()
    assert "not supported" in msg or "invalid" in msg


@pytest.mark.asyncio
async def test_robot_model_import_missing_file(robot_model_tools):
    tool_func = robot_model_tools.mcp.tool.registered_func
    result = await tool_func(
        mock_ctx(),
        operation="import",
        robot_type="scout",
        model_path="/nonexistent/file.fbx",
    )
    assert result["status"] == "error"
    assert "not found" in result.get("message", result.get("error", "")).lower()


@pytest.mark.asyncio
async def test_robot_model_invalid_operation(robot_model_tools):
    tool_func = robot_model_tools.mcp.tool.registered_func
    result = await tool_func(mock_ctx(), operation="invalid_op")
    assert result["status"] == "error"
    assert "unknown operation" in result.get("message", result.get("error", "")).lower()
