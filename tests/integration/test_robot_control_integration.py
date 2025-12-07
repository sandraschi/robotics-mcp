"""Integration tests for robot control tool."""

import pytest
from robotics_mcp.server import RoboticsMCP, RoboticsConfig
from robotics_mcp.tools.robot_control import RobotControlTool


@pytest.fixture
def server():
    """Create server instance for integration testing."""
    config = RoboticsConfig(enable_http=False)
    return RoboticsMCP(config)


@pytest.mark.integration
@pytest.mark.asyncio
async def test_robot_control_full_workflow(server):
    """Test complete robot control workflow."""
    # Register robot
    robot = server.state_manager.register_robot("test_scout_01", "scout", platform="unity")
    assert robot.robot_id == "test_scout_01"
    
    # Get status via tool
    result = await server.robot_control.handle_action(
        robot_id="test_scout_01",
        action="get_status",
        params={}
    )
    
    assert result["status"] == "success"
    assert result["robot_id"] == "test_scout_01"
    assert result["action"] == "get_status"


@pytest.mark.integration
@pytest.mark.asyncio
async def test_robot_control_move_workflow(server):
    """Test robot movement workflow."""
    robot = server.state_manager.register_robot("test_scout_02", "scout")
    
    # Move robot
    result = await server.robot_control.handle_action(
        robot_id="test_scout_02",
        action="move",
        params={"linear": 0.2, "angular": 0.0}
    )
    
    assert result["status"] == "success"
    assert result["action"] == "move"
    
    # Stop robot
    result = await server.robot_control.handle_action(
        robot_id="test_scout_02",
        action="stop",
        params={}
    )
    
    assert result["status"] == "success"
    assert result["action"] == "stop"


@pytest.mark.integration
@pytest.mark.asyncio
async def test_robot_control_virtual_robot_workflow(server):
    """Test virtual robot control workflow."""
    robot = server.state_manager.register_robot(
        "vbot_test_01", "scout", platform="unity"
    )
    
    # Get status
    result = await server.robot_control.handle_action(
        robot_id="vbot_test_01",
        action="get_status",
        params={}
    )
    
    assert result["status"] == "success"
    assert result["robot"]["is_virtual"] is True
    assert result["robot"]["platform"] == "unity"


@pytest.mark.integration
@pytest.mark.asyncio
async def test_robot_control_error_handling(server):
    """Test error handling in robot control."""
    # Try to control non-existent robot
    result = await server.robot_control.handle_action(
        robot_id="nonexistent",
        action="get_status",
        params={}
    )
    
    assert result["status"] == "error"
    assert result["error_type"] == "not_found"


@pytest.mark.integration
@pytest.mark.asyncio
async def test_robot_control_multiple_robots(server):
    """Test controlling multiple robots."""
    # Register multiple robots
    server.state_manager.register_robot("scout_01", "scout")
    server.state_manager.register_robot("scout_02", "scout")
    server.state_manager.register_robot("vbot_01", "scout", platform="unity")
    
    # List all robots
    robots = server.state_manager.list_robots()
    assert len(robots) == 3
    
    # Control each robot
    for robot in robots:
        result = await server.robot_control.handle_action(
            robot_id=robot.robot_id,
            action="get_status",
            params={}
        )
        assert result["status"] == "success"
        assert result["robot_id"] == robot.robot_id

