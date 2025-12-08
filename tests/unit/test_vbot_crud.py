"""Unit tests for vbot_crud tool."""

import pytest
from unittest.mock import AsyncMock, MagicMock, patch

from robotics_mcp.tools.vbot_crud import VbotCrudTool, SUPPORTED_ROBOT_TYPES
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
        "vrchat": MagicMock(),
    }


@pytest.fixture
def vbot_crud_tool(mock_mcp, state_manager, mock_mounted_servers):
    """Create vbot CRUD tool instance."""
    tool = VbotCrudTool(mock_mcp, state_manager, mock_mounted_servers)
    tool.register()
    return tool


@pytest.mark.asyncio
async def test_vbot_crud_create_scout(vbot_crud_tool, state_manager):
    """Test creating a Scout vbot."""
    # Get the registered tool function
    tool_func = vbot_crud_tool.mcp.tool.call_args[0][0]
    
    with patch("robotics_mcp.tools.vbot_crud.Client") as mock_client:
        mock_client_instance = AsyncMock()
        mock_client.return_value.__aenter__.return_value = mock_client_instance
        mock_client_instance.call_tool = AsyncMock(return_value={"success": True, "robot_id": "vbot_scout_01"})
        
        result = await tool_func(
            operation="create",
            robot_type="scout",
            platform="unity",
            position={"x": 0.0, "y": 0.0, "z": 0.0},
            scale=1.0,
        )
        
        assert result["success"] is True
        assert "robot_id" in result["data"]
        assert result["data"]["robot_type"] == "scout"
        assert result["data"]["platform"] == "unity"


@pytest.mark.asyncio
async def test_vbot_crud_create_auto_id(vbot_crud_tool, state_manager):
    """Test creating vbot with auto-generated ID."""
    tool_func = vbot_crud_tool.mcp.tool.call_args[0][0]
    
    with patch("robotics_mcp.tools.vbot_crud.Client") as mock_client:
        mock_client_instance = AsyncMock()
        mock_client.return_value.__aenter__.return_value = mock_client_instance
        mock_client_instance.call_tool = AsyncMock(return_value={"success": True})
        
        result = await tool_func(
            operation="create",
            robot_type="scout",
            platform="unity",
        )
        
        assert result["success"] is True
        assert "robot_id" in result["data"]


@pytest.mark.asyncio
async def test_vbot_crud_read(vbot_crud_tool, state_manager):
    """Test reading vbot details."""
    # Register a robot first
    robot = state_manager.register_robot("vbot_scout_01", "scout", platform="unity")
    
    tool_func = vbot_crud_tool.mcp.tool.call_args[0][0]
    
    result = await tool_func(
        operation="read",
        robot_id="vbot_scout_01",
    )
    
    assert result["success"] is True
    assert result["data"]["robot_id"] == "vbot_scout_01"
    assert result["data"]["robot_type"] == "scout"


@pytest.mark.asyncio
async def test_vbot_crud_read_not_found(vbot_crud_tool):
    """Test reading non-existent vbot."""
    tool_func = vbot_crud_tool.mcp.tool.call_args[0][0]
    
    result = await tool_func(
        operation="read",
        robot_id="nonexistent",
    )
    
    assert result["success"] is False
    assert "not found" in result["error"].lower()


@pytest.mark.asyncio
async def test_vbot_crud_update(vbot_crud_tool, state_manager):
    """Test updating vbot properties."""
    # Register a robot first
    state_manager.register_robot("vbot_scout_01", "scout", platform="unity")
    
    tool_func = vbot_crud_tool.mcp.tool.call_args[0][0]
    
    with patch("robotics_mcp.tools.vbot_crud.Client") as mock_client:
        mock_client_instance = AsyncMock()
        mock_client.return_value.__aenter__.return_value = mock_client_instance
        mock_client_instance.call_tool = AsyncMock(return_value={"success": True})
        
        result = await tool_func(
            operation="update",
            robot_id="vbot_scout_01",
            position={"x": 1.0, "y": 2.0, "z": 3.0},
            scale=2.0,
        )
        
        assert result["success"] is True


@pytest.mark.asyncio
async def test_vbot_crud_delete(vbot_crud_tool, state_manager):
    """Test deleting a vbot."""
    # Register a robot first
    state_manager.register_robot("vbot_scout_01", "scout", platform="unity")
    
    tool_func = vbot_crud_tool.mcp.tool.call_args[0][0]
    
    with patch("robotics_mcp.tools.vbot_crud.Client") as mock_client:
        mock_client_instance = AsyncMock()
        mock_client.return_value.__aenter__.return_value = mock_client_instance
        mock_client_instance.call_tool = AsyncMock(return_value={"success": True})
        
        result = await tool_func(
            operation="delete",
            robot_id="vbot_scout_01",
        )
        
        assert result["success"] is True
        # Verify robot is removed from state manager
        assert state_manager.get_robot("vbot_scout_01") is None


@pytest.mark.asyncio
async def test_vbot_crud_list_all(vbot_crud_tool, state_manager):
    """Test listing all vbots."""
    # Register multiple robots
    state_manager.register_robot("vbot_scout_01", "scout", platform="unity")
    state_manager.register_robot("vbot_go2_01", "go2", platform="unity")
    
    tool_func = vbot_crud_tool.mcp.tool.call_args[0][0]
    
    result = await tool_func(
        operation="list",
    )
    
    assert result["success"] is True
    assert len(result["data"]["robots"]) >= 2


@pytest.mark.asyncio
async def test_vbot_crud_list_filtered(vbot_crud_tool, state_manager):
    """Test listing vbots with filters."""
    # Register robots
    state_manager.register_robot("vbot_scout_01", "scout", platform="unity")
    state_manager.register_robot("vbot_go2_01", "go2", platform="unity")
    
    tool_func = vbot_crud_tool.mcp.tool.call_args[0][0]
    
    result = await tool_func(
        operation="list",
        robot_type="scout",
    )
    
    assert result["success"] is True
    robots = result["data"]["robots"]
    assert all(r["robot_type"] == "scout" for r in robots)


@pytest.mark.asyncio
async def test_vbot_crud_invalid_operation(vbot_crud_tool):
    """Test invalid operation."""
    tool_func = vbot_crud_tool.mcp.tool.call_args[0][0]
    
    result = await tool_func(
        operation="invalid_op",
    )
    
    assert result["success"] is False
    assert "unknown operation" in result["error"].lower()


@pytest.mark.asyncio
async def test_vbot_crud_unsupported_robot_type(vbot_crud_tool):
    """Test unsupported robot type."""
    tool_func = vbot_crud_tool.mcp.tool.call_args[0][0]
    
    result = await tool_func(
        operation="create",
        robot_type="unsupported_type",
        platform="unity",
    )
    
    assert result["success"] is False
    assert "not supported" in result["error"].lower() or "invalid" in result["error"].lower()


@pytest.mark.asyncio
async def test_vbot_crud_all_supported_types(vbot_crud_tool, state_manager):
    """Test creating all supported robot types."""
    tool_func = vbot_crud_tool.mcp.tool.call_args[0][0]
    
    with patch("robotics_mcp.tools.vbot_crud.Client") as mock_client:
        mock_client_instance = AsyncMock()
        mock_client.return_value.__aenter__.return_value = mock_client_instance
        mock_client_instance.call_tool = AsyncMock(return_value={"success": True})
        
        for robot_type in SUPPORTED_ROBOT_TYPES:
            result = await tool_func(
                operation="create",
                robot_type=robot_type,
                platform="unity",
            )
            assert result["success"] is True, f"Failed for robot_type: {robot_type}"

