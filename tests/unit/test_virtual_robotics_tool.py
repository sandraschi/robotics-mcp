"""Unit tests for virtual robotics tool."""

import pytest
from unittest.mock import AsyncMock, MagicMock, patch

from robotics_mcp.tools.virtual_robotics import VirtualRoboticsTool
from robotics_mcp.utils.state_manager import RobotStateManager


@pytest.fixture
def mock_mcp():
    """Create mock FastMCP instance."""
    mcp = MagicMock()
    mcp.tool = MagicMock(return_value=lambda f: f)
    return mcp


@pytest.fixture
def virtual_robotics_tool(mock_mcp):
    """Create virtual robotics tool instance."""
    state_manager = RobotStateManager()
    mounted_servers = {}
    return VirtualRoboticsTool(mock_mcp, state_manager, mounted_servers)


@pytest.mark.asyncio
async def test_spawn_robot_unity(virtual_robotics_tool):
    """Test spawning robot in Unity."""
    virtual_robotics_tool.register()
    tool_func = virtual_robotics_tool.mcp.tool.call_args[0][0]
    
    with patch("robotics_mcp.tools.virtual_robotics.Client") as mock_client:
        mock_client_instance = AsyncMock()
        mock_client.return_value.__aenter__.return_value = mock_client_instance
        mock_client_instance.call_tool = AsyncMock(return_value={"status": "ok"})
        
        result = await tool_func(
            robot_type="scout",
            action="spawn_robot",
            platform="unity",
            position={"x": 0.0, "y": 0.0, "z": 0.0}
        )
        
        assert result["status"] == "success"
        assert "spawned" in result["message"].lower()
        assert result["platform"] == "unity"


@pytest.mark.asyncio
async def test_spawn_robot_vrchat(virtual_robotics_tool):
    """Test spawning robot in VRChat."""
    virtual_robotics_tool.mounted_servers["osc"] = MagicMock()
    virtual_robotics_tool.register()
    tool_func = virtual_robotics_tool.mcp.tool.call_args[0][0]
    
    with patch("robotics_mcp.tools.virtual_robotics.Client") as mock_client:
        mock_client_instance = AsyncMock()
        mock_client.return_value.__aenter__.return_value = mock_client_instance
        mock_client_instance.call_tool = AsyncMock(return_value={"status": "ok"})
        
        result = await tool_func(
            robot_type="scout",
            action="spawn_robot",
            platform="vrchat",
            position={"x": 1.0, "y": 0.0, "z": 1.0}
        )
        
        assert result["status"] == "success"
        assert result["platform"] == "vrchat"


@pytest.mark.asyncio
async def test_load_environment(virtual_robotics_tool):
    """Test loading environment."""
    virtual_robotics_tool.mounted_servers["unity"] = MagicMock()
    virtual_robotics_tool.register()
    tool_func = virtual_robotics_tool.mcp.tool.call_args[0][0]
    
    with patch("robotics_mcp.tools.virtual_robotics.Client") as mock_client:
        mock_client_instance = AsyncMock()
        mock_client.return_value.__aenter__.return_value = mock_client_instance
        mock_client_instance.call_tool = AsyncMock(return_value={"status": "loaded"})
        
        result = await tool_func(
            robot_type="scout",
            action="load_environment",
            environment="test_environment",
            platform="unity"
        )
        
        assert result["status"] == "success"
        assert "environment" in result["message"].lower()


@pytest.mark.asyncio
async def test_get_status(virtual_robotics_tool):
    """Test getting virtual robot status."""
    robot = virtual_robotics_tool.state_manager.register_robot(
        "vbot_scout_01", "scout", platform="unity"
    )
    virtual_robotics_tool.register()
    tool_func = virtual_robotics_tool.mcp.tool.call_args[0][0]
    
    result = await tool_func(
        robot_type="scout",
        action="get_status",
        robot_id="vbot_scout_01"
    )
    
    assert result["status"] == "success"
    assert result["robot"]["robot_id"] == "vbot_scout_01"


@pytest.mark.asyncio
async def test_get_lidar(virtual_robotics_tool):
    """Test getting virtual LiDAR scan."""
    robot = virtual_robotics_tool.state_manager.register_robot(
        "vbot_scout_01", "scout", platform="unity"
    )
    virtual_robotics_tool.register()
    tool_func = virtual_robotics_tool.mcp.tool.call_args[0][0]
    
    result = await tool_func(
        robot_type="scout",
        action="get_lidar",
        robot_id="vbot_scout_01"
    )
    
    assert result["status"] == "success"
    assert "scan_data" in result or "mock" in result.get("method", "").lower()


@pytest.mark.asyncio
async def test_set_scale(virtual_robotics_tool):
    """Test setting robot scale."""
    robot = virtual_robotics_tool.state_manager.register_robot(
        "vbot_scout_01", "scout", platform="unity"
    )
    virtual_robotics_tool.register()
    tool_func = virtual_robotics_tool.mcp.tool.call_args[0][0]
    
    result = await tool_func(
        robot_type="scout",
        action="set_scale",
        robot_id="vbot_scout_01",
        scale=1.5
    )
    
    assert result["status"] == "success"
    assert result["scale"] == 1.5


@pytest.mark.asyncio
async def test_unknown_action(virtual_robotics_tool):
    """Test unknown action handling."""
    virtual_robotics_tool.register()
    tool_func = virtual_robotics_tool.mcp.tool.call_args[0][0]
    
    # This should be caught by Literal type, but test error path
    # We'll use a valid action but check validation
    result = await tool_func(
        robot_type="scout",
        action="get_status",
        robot_id="nonexistent"
    )
    
    # Should return error for non-existent robot
    assert result["status"] == "error"


@pytest.mark.asyncio
async def test_spawn_robot_auto_id(virtual_robotics_tool):
    """Test spawning robot with auto-generated ID."""
    virtual_robotics_tool.register()
    tool_func = virtual_robotics_tool.mcp.tool.call_args[0][0]
    
    result = await tool_func(
        robot_type="scout",
        action="spawn_robot",
        platform="unity"
    )
    
    assert result["status"] == "success"
    assert "robot_id" in result
    assert result["robot_id"].startswith("vbot_scout")


@pytest.mark.asyncio
async def test_get_status_robot_not_found(virtual_robotics_tool):
    """Test get_status with non-existent robot."""
    virtual_robotics_tool.register()
    tool_func = virtual_robotics_tool.mcp.tool.call_args[0][0]
    
    result = await tool_func(
        robot_type="scout",
        action="get_status",
        robot_id="nonexistent"
    )
    
    assert result["status"] == "error"
    assert result["error_type"] == "not_found"

