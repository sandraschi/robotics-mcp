"""Unit tests for robot_virtual tool."""

import pytest
from unittest.mock import AsyncMock, MagicMock, patch

from robotics_mcp.tools.robot_virtual import RobotVirtualTool, SUPPORTED_ROBOT_TYPES
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
def robot_virtual_tool(mock_mcp, state_manager, mock_mounted_servers):
    """Create robot virtual tool instance."""
    tool = RobotVirtualTool(mock_mcp, state_manager, mock_mounted_servers)
    tool.register()
    return tool


@pytest.mark.asyncio
async def test_robot_virtual_create(robot_virtual_tool, state_manager):
    """Test creating a virtual robot."""
    tool_func = robot_virtual_tool.mcp.tool.call_args[0][0]
    
    with patch("robotics_mcp.tools.robot_virtual.Client") as mock_client:
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


@pytest.mark.asyncio
async def test_robot_virtual_read(robot_virtual_tool, state_manager):
    """Test reading virtual robot."""
    # Register a robot first
    state_manager.register_robot("vbot_scout_01", "scout", platform="unity")
    
    tool_func = robot_virtual_tool.mcp.tool.call_args[0][0]
    
    result = await tool_func(
        operation="read",
        robot_id="vbot_scout_01",
    )
    
    assert result["success"] is True
    assert result["data"]["robot_id"] == "vbot_scout_01"


@pytest.mark.asyncio
async def test_robot_virtual_read_not_found(robot_virtual_tool):
    """Test reading non-existent robot."""
    tool_func = robot_virtual_tool.mcp.tool.call_args[0][0]
    
    result = await tool_func(
        operation="read",
        robot_id="nonexistent",
    )
    
    assert result["success"] is False
    assert "not found" in result["error"].lower()


@pytest.mark.asyncio
async def test_robot_virtual_update(robot_virtual_tool, state_manager):
    """Test updating virtual robot."""
    # Register a robot first
    state_manager.register_robot("vbot_scout_01", "scout", platform="unity")
    
    tool_func = robot_virtual_tool.mcp.tool.call_args[0][0]
    
    with patch("robotics_mcp.tools.robot_virtual.Client") as mock_client:
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
async def test_robot_virtual_delete(robot_virtual_tool, state_manager):
    """Test deleting virtual robot."""
    # Register a robot first
    state_manager.register_robot("vbot_scout_01", "scout", platform="unity")
    
    tool_func = robot_virtual_tool.mcp.tool.call_args[0][0]
    
    with patch("robotics_mcp.tools.robot_virtual.Client") as mock_client:
        mock_client_instance = AsyncMock()
        mock_client.return_value.__aenter__.return_value = mock_client_instance
        mock_client_instance.call_tool = AsyncMock(return_value={"success": True})
        
        result = await tool_func(
            operation="delete",
            robot_id="vbot_scout_01",
        )
        
        assert result["success"] is True
        # Verify robot is removed
        assert state_manager.get_robot("vbot_scout_01") is None


@pytest.mark.asyncio
async def test_robot_virtual_list(robot_virtual_tool, state_manager):
    """Test listing virtual robots."""
    # Register multiple robots
    state_manager.register_robot("vbot_scout_01", "scout", platform="unity")
    state_manager.register_robot("vbot_go2_01", "go2", platform="unity")
    
    tool_func = robot_virtual_tool.mcp.tool.call_args[0][0]
    
    result = await tool_func(
        operation="list",
    )
    
    assert result["success"] is True
    assert len(result["data"]["robots"]) >= 2


@pytest.mark.asyncio
async def test_robot_virtual_spawn(robot_virtual_tool, state_manager):
    """Test spawning robot (alias for create)."""
    tool_func = robot_virtual_tool.mcp.tool.call_args[0][0]
    
    with patch("robotics_mcp.tools.robot_virtual.Client") as mock_client:
        mock_client_instance = AsyncMock()
        mock_client.return_value.__aenter__.return_value = mock_client_instance
        mock_client_instance.call_tool = AsyncMock(return_value={"success": True})
        
        result = await tool_func(
            operation="spawn",
            robot_type="scout",
            platform="unity",
        )
        
        assert result["success"] is True


@pytest.mark.asyncio
async def test_robot_virtual_load_environment(robot_virtual_tool):
    """Test loading environment."""
    tool_func = robot_virtual_tool.mcp.tool.call_args[0][0]
    
    with patch("robotics_mcp.tools.robot_virtual.Client") as mock_client:
        mock_client_instance = AsyncMock()
        mock_client.return_value.__aenter__.return_value = mock_client_instance
        mock_client_instance.call_tool = AsyncMock(return_value={"success": True, "environment_loaded": True})
        
        result = await tool_func(
            operation="load_environment",
            environment="test_environment",
            environment_path="/tmp/environment.glb",
            platform="unity",
        )
        
        assert result["success"] is True


@pytest.mark.asyncio
async def test_robot_virtual_get_status(robot_virtual_tool, state_manager):
    """Test getting virtual robot status."""
    # Register a robot first
    state_manager.register_robot("vbot_scout_01", "scout", platform="unity")
    
    tool_func = robot_virtual_tool.mcp.tool.call_args[0][0]
    
    result = await tool_func(
        operation="get_status",
        robot_id="vbot_scout_01",
    )
    
    assert result["success"] is True
    assert "robot_id" in result["data"]


@pytest.mark.asyncio
async def test_robot_virtual_set_scale(robot_virtual_tool, state_manager):
    """Test setting robot scale."""
    # Register a robot first
    state_manager.register_robot("vbot_scout_01", "scout", platform="unity")
    
    tool_func = robot_virtual_tool.mcp.tool.call_args[0][0]
    
    with patch("robotics_mcp.tools.robot_virtual.Client") as mock_client:
        mock_client_instance = AsyncMock()
        mock_client.return_value.__aenter__.return_value = mock_client_instance
        mock_client_instance.call_tool = AsyncMock(return_value={"success": True})
        
        result = await tool_func(
            operation="set_scale",
            robot_id="vbot_scout_01",
            scale=2.0,
        )
        
        assert result["success"] is True


@pytest.mark.asyncio
async def test_robot_virtual_invalid_operation(robot_virtual_tool):
    """Test invalid operation."""
    tool_func = robot_virtual_tool.mcp.tool.call_args[0][0]
    
    result = await tool_func(
        operation="invalid_op",
    )
    
    assert result["success"] is False
    assert "unknown operation" in result["error"].lower()


@pytest.mark.asyncio
async def test_robot_virtual_all_supported_types(robot_virtual_tool, state_manager):
    """Test creating all supported robot types."""
    tool_func = robot_virtual_tool.mcp.tool.call_args[0][0]
    
    with patch("robotics_mcp.tools.robot_virtual.Client") as mock_client:
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

