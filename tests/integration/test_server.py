"""Integration tests for robotics-mcp server."""

import pytest

from robotics_mcp.server import RoboticsConfig, RoboticsMCP


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
@pytest.mark.asyncio
async def test_robot_registration(initialized_server):
    """Test robot registration via server."""
    robot = initialized_server.state_manager.register_robot("test_robot", "scout", platform="unity")

    assert robot.robot_id == "test_robot"
    assert robot.is_virtual is True

    robots = initialized_server.state_manager.list_robots()
    assert any(r.robot_id == "test_robot" for r in robots)
