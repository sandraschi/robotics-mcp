"""Integration tests for server tools (help, status, list_robots)."""

import pytest
from robotics_mcp.server import RoboticsMCP, RoboticsConfig


@pytest.fixture
def server():
    """Create server instance for testing."""
    config = RoboticsConfig(enable_http=False)
    return RoboticsMCP(config)


@pytest.mark.integration
@pytest.mark.asyncio
async def test_help_tool(server):
    """Test help tool."""
    # Get help tool
    tools = server.mcp.list_tools()
    assert "help" in tools
    
    # Call help tool
    help_info = await server.mcp.call_tool("help", {})
    
    assert help_info["server_name"] == "Robotics-MCP"
    assert help_info["version"] == "0.1.0"
    assert "tools" in help_info
    assert "features" in help_info
    assert len(help_info["tools"]) > 0


@pytest.mark.integration
@pytest.mark.asyncio
async def test_get_status_tool(server):
    """Test get_status tool."""
    # Register a robot
    server.state_manager.register_robot("test_robot", "scout")
    
    # Call status tool
    status = await server.mcp.call_tool("get_status", {})
    
    assert status["status"] == "success"
    assert status["data"]["version"] == "0.1.0"
    assert status["data"]["status"] == "healthy"
    assert len(status["data"]["robots"]) == 1
    assert "mounted_servers" in status["data"]


@pytest.mark.integration
@pytest.mark.asyncio
async def test_list_robots_tool(server):
    """Test list_robots tool."""
    # Register multiple robots
    server.state_manager.register_robot("scout_01", "scout")
    server.state_manager.register_robot("vbot_01", "scout", platform="unity")
    server.state_manager.register_robot("go2_01", "go2")
    
    # List all robots
    result = await server.mcp.call_tool("list_robots", {})
    
    assert result["status"] == "success"
    assert result["data"]["count"] == 3
    assert len(result["data"]["robots"]) == 3
    
    # Filter by type
    result = await server.mcp.call_tool("list_robots", {"robot_type": "scout"})
    assert result["data"]["count"] == 2
    
    # Filter by virtual
    result = await server.mcp.call_tool("list_robots", {"is_virtual": True})
    assert result["data"]["count"] == 1
    assert result["data"]["robots"][0]["is_virtual"] is True


@pytest.mark.integration
@pytest.mark.asyncio
async def test_robot_control_tool(server):
    """Test robot_control tool."""
    # Register robot
    server.state_manager.register_robot("test_robot", "scout")
    
    # Get status
    result = await server.mcp.call_tool(
        "robot_control",
        {
            "robot_id": "test_robot",
            "action": "get_status"
        }
    )
    
    assert result["status"] == "success"
    assert result["robot_id"] == "test_robot"
    assert result["action"] == "get_status"


@pytest.mark.integration
@pytest.mark.asyncio
async def test_virtual_robotics_tool(server):
    """Test virtual_robotics tool."""
    # Spawn robot
    result = await server.mcp.call_tool(
        "virtual_robotics",
        {
            "robot_type": "scout",
            "action": "spawn_robot",
            "platform": "unity",
            "position": {"x": 0.0, "y": 0.0, "z": 0.0}
        }
    )
    
    assert result["status"] == "success"
    assert "robot_id" in result
    assert result["platform"] == "unity"
    
    # Get status
    robot_id = result["robot_id"]
    result = await server.mcp.call_tool(
        "virtual_robotics",
        {
            "robot_type": "scout",
            "action": "get_status",
            "robot_id": robot_id
        }
    )
    
    assert result["status"] == "success"
    assert result["robot"]["robot_id"] == robot_id


@pytest.mark.integration
@pytest.mark.asyncio
async def test_tool_error_handling(server):
    """Test error handling in tools."""
    # Try to control non-existent robot
    result = await server.mcp.call_tool(
        "robot_control",
        {
            "robot_id": "nonexistent",
            "action": "get_status"
        }
    )
    
    assert result["status"] == "error"
    assert result["error_type"] == "not_found"
    assert "not found" in result["message"].lower()


@pytest.mark.integration
@pytest.mark.asyncio
async def test_server_with_http_enabled():
    """Test server with HTTP enabled."""
    config = RoboticsConfig(enable_http=True, http_port=8081)
    server = RoboticsMCP(config)
    
    assert server.http_app is not None
    assert server.config.enable_http is True
    
    # Test that tools still work
    result = await server.mcp.call_tool("help", {})
    assert result["server_name"] == "Robotics-MCP"

