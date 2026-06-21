"""Unit tests for robot_virtual tool."""

from unittest.mock import AsyncMock, MagicMock, patch

import pytest

from robotics_mcp.tools.robot_virtual import SUPPORTED_ROBOT_TYPES, RobotVirtualTool
from robotics_mcp.utils.state_manager import RobotStateManager
from tests.helpers import mock_ctx


@pytest.fixture
def state_manager():
    """Create robot state manager."""
    return RobotStateManager()


@pytest.fixture
def mock_mounted_servers():
    """Create mock mounted servers."""
    return {
        "unity": MagicMock(),
        "vrchat": MagicMock(),
    }


@pytest.fixture
def robot_virtual_tool(mock_mcp, state_manager, mock_mounted_servers):
    """Create robot virtual tool instance."""
    tool = RobotVirtualTool(mock_mcp, state_manager, mock_mounted_servers)
    tool.register()
    return tool


@pytest.mark.asyncio
async def test_robot_virtual_create(robot_virtual_tool, state_manager):
    """Test creating a virtual robot."""
    tool_func = robot_virtual_tool.mcp.tool.registered_func

    with patch("robotics_mcp.tools.robot_virtual.call_mounted_server_tool", new_callable=AsyncMock) as mock_call:
        mock_call.return_value = {"status": "success", "robot_id": "vbot_scout_01"}

        result = await tool_func(
            mock_ctx(),
            operation="create",
            robot_type="scout",
            platform="unity",
            position={"x": 0.0, "y": 0.0, "z": 0.0},
            scale=1.0,
        )

        assert result["status"] == "success"
        assert "robot_id" in result


@pytest.mark.asyncio
async def test_robot_virtual_read(robot_virtual_tool, state_manager):
    """Test reading virtual robot."""
    # Register a robot first
    state_manager.register_robot("vbot_scout_01", "scout", platform="unity")

    tool_func = robot_virtual_tool.mcp.tool.registered_func

    result = await tool_func(
        mock_ctx(),
        operation="read",
        robot_id="vbot_scout_01",
    )

    assert result["status"] == "success"
    assert result["robot_id"] == "vbot_scout_01"


@pytest.mark.asyncio
async def test_robot_virtual_read_not_found(robot_virtual_tool):
    """Test reading non-existent robot."""
    tool_func = robot_virtual_tool.mcp.tool.registered_func

    result = await tool_func(
        mock_ctx(),
        operation="read",
        robot_id="nonexistent",
    )

    assert result["status"] == "error"
    assert "not found" in result.get("message", result.get("error", "")).lower()


@pytest.mark.asyncio
async def test_robot_virtual_update(robot_virtual_tool, state_manager):
    """Test updating virtual robot."""
    # Register a robot first
    state_manager.register_robot("vbot_scout_01", "scout", platform="unity")

    tool_func = robot_virtual_tool.mcp.tool.registered_func

    with patch("robotics_mcp.tools.robot_virtual.call_mounted_server_tool", new_callable=AsyncMock) as mock_call:
        mock_call.return_value = {"status": "success"}

        result = await tool_func(
            mock_ctx(),
            operation="update",
            robot_id="vbot_scout_01",
            position={"x": 1.0, "y": 2.0, "z": 3.0},
            scale=2.0,
        )

        assert result["status"] == "success"


@pytest.mark.asyncio
async def test_robot_virtual_delete(robot_virtual_tool, state_manager):
    """Test deleting virtual robot."""
    # Register a robot first
    state_manager.register_robot("vbot_scout_01", "scout", platform="unity")

    tool_func = robot_virtual_tool.mcp.tool.registered_func

    with patch("robotics_mcp.tools.robot_virtual.call_mounted_server_tool", new_callable=AsyncMock) as mock_call:
        mock_call.return_value = {"status": "success"}

        result = await tool_func(
            mock_ctx(),
            operation="delete",
            robot_id="vbot_scout_01",
        )

        assert result["status"] == "success"
        # Verify robot is removed
        assert state_manager.get_robot("vbot_scout_01") is None


@pytest.mark.asyncio
async def test_robot_virtual_list(robot_virtual_tool, state_manager):
    """Test listing virtual robots."""
    # Register multiple robots
    state_manager.register_robot("vbot_scout_01", "scout", platform="unity")
    state_manager.register_robot("vbot_go2_01", "go2", platform="unity")

    tool_func = robot_virtual_tool.mcp.tool.registered_func

    result = await tool_func(
        mock_ctx(),
        operation="list",
    )

    assert result["status"] == "success"
    assert len(result["robots"]) >= 2


@pytest.mark.asyncio
async def test_robot_virtual_spawn(robot_virtual_tool, state_manager):
    """Test spawning robot (alias for create)."""
    tool_func = robot_virtual_tool.mcp.tool.registered_func

    with patch("robotics_mcp.tools.robot_virtual.call_mounted_server_tool", new_callable=AsyncMock) as mock_call:
        mock_call.return_value = {"status": "success"}

        result = await tool_func(
            mock_ctx(),
            operation="spawn",
            robot_type="scout",
            platform="unity",
        )

        assert result["status"] == "success"


@pytest.mark.asyncio
async def test_robot_virtual_load_environment(robot_virtual_tool):
    """Test loading environment."""
    tool_func = robot_virtual_tool.mcp.tool.registered_func

    with patch("robotics_mcp.tools.robot_virtual.call_mounted_server_tool", new_callable=AsyncMock) as mock_call:
        mock_call.return_value = {"success": True, "environment_loaded": True}

        result = await tool_func(
            mock_ctx(),
            operation="load_environment",
            environment="test_environment",
            environment_path="/tmp/environment.glb",
            platform="unity",
        )

        assert result["status"] == "success"


@pytest.mark.asyncio
async def test_robot_virtual_get_status(robot_virtual_tool, state_manager):
    """Test getting virtual robot status."""
    # Register a robot first
    state_manager.register_robot("vbot_scout_01", "scout", platform="unity")

    tool_func = robot_virtual_tool.mcp.tool.registered_func

    result = await tool_func(
        mock_ctx(),
        operation="get_status",
        robot_id="vbot_scout_01",
    )

    assert result["status"] == "success"
    assert "robot_id" in result


@pytest.mark.asyncio
async def test_robot_virtual_set_scale(robot_virtual_tool, state_manager):
    """Test setting robot scale."""
    # Register a robot first
    state_manager.register_robot("vbot_scout_01", "scout", platform="unity")

    tool_func = robot_virtual_tool.mcp.tool.registered_func

    with patch("robotics_mcp.tools.robot_virtual.call_mounted_server_tool", new_callable=AsyncMock) as mock_call:
        mock_call.return_value = {"status": "success"}

        result = await tool_func(
            mock_ctx(),
            operation="set_scale",
            robot_id="vbot_scout_01",
            scale=2.0,
        )

        assert result["status"] == "success"


@pytest.mark.asyncio
async def test_robot_virtual_invalid_operation(robot_virtual_tool):
    """Test invalid operation."""
    tool_func = robot_virtual_tool.mcp.tool.registered_func

    result = await tool_func(
        mock_ctx(),
        operation="invalid_op",
    )

    assert result["status"] == "error"
    assert "unknown operation" in result.get("message", result.get("error", "")).lower()


@pytest.mark.asyncio
async def test_robot_virtual_all_supported_types(robot_virtual_tool, state_manager):
    """Test creating all supported robot types."""
    tool_func = robot_virtual_tool.mcp.tool.registered_func

    with patch("robotics_mcp.tools.robot_virtual.call_mounted_server_tool", new_callable=AsyncMock) as mock_call:
        mock_call.return_value = {"status": "success"}

        for robot_type in SUPPORTED_ROBOT_TYPES:
            kwargs: dict = {
                "operation": "create",
                "robot_type": robot_type,
                "platform": "unity",
            }
            if robot_type == "custom":
                kwargs["model_path"] = "/tmp/test_robot.glb"
            result = await tool_func(mock_ctx(), **kwargs)
            assert result["status"] == "success", f"Failed for robot_type: {robot_type}"
