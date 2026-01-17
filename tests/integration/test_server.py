"""Integration tests for robotics-mcp server."""

import pytest
from robotics_mcp.server import RoboticsMCP, RoboticsConfig


@pytest.mark.integration
def test_server_initialization():
    """Test server initialization."""
    config = RoboticsConfig(enable_http=False)
    server = RoboticsMCP(config)
    
    assert server.mcp is not None
    assert server.state_manager is not None
    assert server.config_data is not None


@pytest.mark.integration
def test_server_with_http():
    """Test server with HTTP enabled."""
    config = RoboticsConfig(enable_http=True, http_port=12231)
    server = RoboticsMCP(config)
    
    assert server.http_app is not None
    assert server.config.enable_http is True


@pytest.mark.integration
def test_robot_registration(server: RoboticsMCP):
    """Test robot registration via server."""
    robot = server.state_manager.register_robot("test_robot", "scout", platform="unity")
    
    assert robot.robot_id == "test_robot"
    assert robot.is_virtual is True
    
    robots = server.state_manager.list_robots()
    assert len(robots) == 1


@pytest.fixture
def server():
    """Create server instance for testing."""
    config = RoboticsConfig(enable_http=False)
    return RoboticsMCP(config)

