"""Unit tests for vbot_crud tool."""

from unittest.mock import AsyncMock, MagicMock, patch

import pytest

from robotics_mcp.tools.vbot_crud import SUPPORTED_ROBOT_TYPES, VbotCrudTool
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
def vbot_crud_tool(mock_mcp, state_manager, mock_mounted_servers):
    """Create vbot CRUD tool instance."""
    tool = VbotCrudTool(mock_mcp, state_manager, mock_mounted_servers, unity_available=True)
    tool.register()
    return tool


@pytest.mark.asyncio
async def test_vbot_crud_create_scout(vbot_crud_tool, state_manager):
    """Test creating a Scout vbot."""
    # Get the registered tool function
    tool_func = vbot_crud_tool.mcp.tool.registered_func

    with patch("robotics_mcp.tools.vbot_crud.call_mounted_server_tool", new_callable=AsyncMock) as mock_call:
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
        assert result["robot_type"] == "scout"
        assert result["platform"] == "unity"


@pytest.mark.asyncio
async def test_vbot_crud_create_auto_id(vbot_crud_tool, state_manager):
    """Test creating vbot with auto-generated ID."""
    tool_func = vbot_crud_tool.mcp.tool.registered_func

    with patch("robotics_mcp.tools.vbot_crud.call_mounted_server_tool", new_callable=AsyncMock) as mock_call:
        mock_call.return_value = {"status": "success"}

        result = await tool_func(
            mock_ctx(),
            operation="create",
            robot_type="scout",
            platform="unity",
        )

        assert result["status"] == "success"
        assert "robot_id" in result


@pytest.mark.asyncio
async def test_vbot_crud_read(vbot_crud_tool, state_manager):
    """Test reading vbot details."""
    # Register a robot first
    state_manager.register_robot("vbot_scout_01", "scout", platform="unity")

    tool_func = vbot_crud_tool.mcp.tool.registered_func

    result = await tool_func(
        mock_ctx(),
        operation="read",
        robot_id="vbot_scout_01",
    )

    assert result["status"] == "success"
    assert result["robot_id"] == "vbot_scout_01"
    assert result["robot_type"] == "scout"


@pytest.mark.asyncio
async def test_vbot_crud_read_not_found(vbot_crud_tool):
    """Test reading non-existent vbot."""
    tool_func = vbot_crud_tool.mcp.tool.registered_func

    result = await tool_func(
        mock_ctx(),
        operation="read",
        robot_id="nonexistent",
    )

    assert result["status"] == "error"
    assert "not found" in result.get("message", result.get("error", "")).lower()


@pytest.mark.asyncio
async def test_vbot_crud_update(vbot_crud_tool, state_manager):
    """Test updating vbot properties."""
    # Register a robot first
    state_manager.register_robot("vbot_scout_01", "scout", platform="unity")

    tool_func = vbot_crud_tool.mcp.tool.registered_func

    with patch("robotics_mcp.tools.vbot_crud.call_mounted_server_tool", new_callable=AsyncMock) as mock_call:
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
async def test_vbot_crud_delete(vbot_crud_tool, state_manager):
    """Test deleting a vbot."""
    # Register a robot first
    state_manager.register_robot("vbot_scout_01", "scout", platform="unity")

    tool_func = vbot_crud_tool.mcp.tool.registered_func

    with patch("robotics_mcp.tools.vbot_crud.call_mounted_server_tool", new_callable=AsyncMock) as mock_call:
        mock_call.return_value = {"status": "success"}

        result = await tool_func(
            mock_ctx(),
            operation="delete",
            robot_id="vbot_scout_01",
        )

        assert result["status"] == "success"
        # Verify robot is removed from state manager
        assert state_manager.get_robot("vbot_scout_01") is None


@pytest.mark.asyncio
async def test_vbot_crud_list_all(vbot_crud_tool, state_manager):
    """Test listing all vbots."""
    # Register multiple robots
    state_manager.register_robot("vbot_scout_01", "scout", platform="unity")
    state_manager.register_robot("vbot_go2_01", "go2", platform="unity")

    tool_func = vbot_crud_tool.mcp.tool.registered_func

    result = await tool_func(
        mock_ctx(),
        operation="list",
    )

    assert result["status"] == "success"
    assert len(result["robots"]) >= 2


@pytest.mark.asyncio
async def test_vbot_crud_list_filtered(vbot_crud_tool, state_manager):
    """Test listing vbots with filters."""
    # Register robots
    state_manager.register_robot("vbot_scout_01", "scout", platform="unity")
    state_manager.register_robot("vbot_go2_01", "go2", platform="unity")

    tool_func = vbot_crud_tool.mcp.tool.registered_func

    result = await tool_func(
        mock_ctx(),
        operation="list",
        robot_type="scout",
    )

    assert result["status"] == "success"
    robots = result["robots"]
    assert all(r["robot_type"] == "scout" for r in robots)


@pytest.mark.asyncio
async def test_vbot_crud_invalid_operation(vbot_crud_tool):
    """Test invalid operation."""
    tool_func = vbot_crud_tool.mcp.tool.registered_func

    result = await tool_func(
        mock_ctx(),
        operation="invalid_op",
    )

    assert result["status"] == "error"
    assert "unknown operation" in result.get("message", result.get("error", "")).lower()


@pytest.mark.asyncio
async def test_vbot_crud_unsupported_robot_type(vbot_crud_tool):
    """Test unsupported robot type."""
    tool_func = vbot_crud_tool.mcp.tool.registered_func

    result = await tool_func(
        mock_ctx(),
        operation="create",
        robot_type="unsupported_type",
        platform="unity",
    )

    assert result["status"] == "error"
    msg = result.get("message", result.get("error", "")).lower()
    assert "unsupported" in msg or "not supported" in msg or "invalid" in msg


@pytest.mark.asyncio
async def test_vbot_crud_all_supported_types(vbot_crud_tool, state_manager):
    """Test creating all supported robot types."""
    tool_func = vbot_crud_tool.mcp.tool.registered_func

    with patch("robotics_mcp.tools.vbot_crud.call_mounted_server_tool", new_callable=AsyncMock) as mock_call:
        mock_call.return_value = {"status": "success"}

        for robot_type in [t for t in SUPPORTED_ROBOT_TYPES if t != "custom"]:
            result = await tool_func(
                mock_ctx(),
                operation="create",
                robot_type=robot_type,
                platform="unity",
            )
            assert result["status"] == "success", f"Failed for robot_type: {robot_type}"
