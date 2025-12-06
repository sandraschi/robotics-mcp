"""Unit tests for state manager."""

import pytest

from robotics_mcp.utils.state_manager import RobotStateManager, RobotState


def test_register_robot(state_manager: RobotStateManager):
    """Test robot registration."""
    robot = state_manager.register_robot("scout_01", "scout")
    assert robot.robot_id == "scout_01"
    assert robot.robot_type == "scout"
    assert not robot.is_virtual


def test_register_virtual_robot(state_manager: RobotStateManager):
    """Test virtual robot registration."""
    robot = state_manager.register_robot("vbot_scout_01", "scout", platform="unity")
    assert robot.robot_id == "vbot_scout_01"
    assert robot.is_virtual
    assert robot.platform == "unity"


def test_get_robot(state_manager: RobotStateManager):
    """Test getting robot by ID."""
    state_manager.register_robot("scout_01", "scout")
    robot = state_manager.get_robot("scout_01")
    assert robot is not None
    assert robot.robot_id == "scout_01"


def test_list_robots(state_manager: RobotStateManager):
    """Test listing robots."""
    state_manager.register_robot("scout_01", "scout")
    state_manager.register_robot("vbot_scout_01", "scout", platform="unity")
    
    all_robots = state_manager.list_robots()
    assert len(all_robots) == 2
    
    virtual_robots = state_manager.list_robots(is_virtual=True)
    assert len(virtual_robots) == 1
    assert virtual_robots[0].robot_id == "vbot_scout_01"


def test_duplicate_robot_id(state_manager: RobotStateManager):
    """Test duplicate robot ID registration."""
    state_manager.register_robot("scout_01", "scout")
    with pytest.raises(ValueError, match="already registered"):
        state_manager.register_robot("scout_01", "scout")


def test_update_robot_status(state_manager: RobotStateManager):
    """Test updating robot connection status."""
    state_manager.register_robot("scout_01", "scout")
    state_manager.update_robot_status("scout_01", connected=True)
    
    robot = state_manager.get_robot("scout_01")
    assert robot.connected is True


def test_unregister_robot(state_manager: RobotStateManager):
    """Test unregistering robot."""
    state_manager.register_robot("scout_01", "scout")
    state_manager.unregister_robot("scout_01")
    
    robot = state_manager.get_robot("scout_01")
    assert robot is None

