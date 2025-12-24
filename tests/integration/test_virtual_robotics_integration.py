"""Integration tests for virtual robotics."""

import pytest
from robotics_mcp.server import RoboticsMCP, RoboticsConfig
from robotics_mcp.utils.state_manager import RobotStateManager


@pytest.mark.integration
@pytest.mark.asyncio
async def test_spawn_virtual_robot():
    """Test spawning virtual robot."""
    config = RoboticsConfig(enable_http=False)
    server = RoboticsMCP(config)

    # Spawn robot
    result = await server.virtual_robotics._spawn_robot(
        robot_type="scout",
        robot_id="test_scout_01",
        position={"x": 0, "y": 0, "z": 0},
        scale=1.0,
        platform="vrchat",
    )

    assert result["status"] == "success"
    assert result["robot_id"] == "test_scout_01"
    assert result["platform"] == "vrchat"

    # Verify robot is registered
    robot = server.state_manager.get_robot("test_scout_01")
    assert robot is not None
    assert robot.is_virtual is True
    assert robot.platform == "vrchat"


@pytest.mark.integration
@pytest.mark.asyncio
async def test_virtual_robot_movement():
    """Test virtual robot movement."""
    config = RoboticsConfig(enable_http=False)
    server = RoboticsMCP(config)

    # Register robot first
    robot = server.state_manager.register_robot("test_scout_01", "scout", platform="vrchat")

    # Test movement
    result = await server.robot_control._handle_virtual_robot(
        robot, "move", linear=0.2, angular=0.0, duration=None
    )

    assert result["status"] == "success"
    assert result["action"] == "move"


@pytest.mark.integration
@pytest.mark.asyncio
async def test_load_marble_environment():
    """Test loading Marble environment."""
    config = RoboticsConfig(enable_http=False)
    server = RoboticsMCP(config)

    result = await server.virtual_robotics._load_environment(
        environment="test_apartment",
        platform="unity",
    )

    assert result["status"] == "success"
    assert result["environment"] == "test_apartment"


@pytest.mark.integration
@pytest.mark.asyncio
async def test_virtual_lidar():
    """Test virtual LiDAR scan."""
    config = RoboticsConfig(enable_http=False)
    server = RoboticsMCP(config)

    # Register robot
    server.state_manager.register_robot("test_scout_01", "scout", platform="unity")

    result = await server.virtual_robotics._get_lidar("test_scout_01")

    assert result["status"] == "success"
    assert "scan_data" in result


@pytest.fixture
def server():
    """Create server instance for testing."""
    config = RoboticsConfig(enable_http=False)
    return RoboticsMCP(config)

