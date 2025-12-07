"""Extended unit tests for state manager."""

import pytest

from robotics_mcp.utils.state_manager import RobotStateManager, RobotState


def test_robot_state_to_dict():
    """Test robot state serialization."""
    robot = RobotState(
        robot_id="test_01",
        robot_type="scout",
        platform="unity",
        connected=True,
        metadata={"key": "value"}
    )
    
    data = robot.to_dict()
    
    assert data["robot_id"] == "test_01"
    assert data["robot_type"] == "scout"
    assert data["platform"] == "unity"
    assert data["connected"] is True
    assert data["is_virtual"] is True
    assert data["metadata"]["key"] == "value"


def test_robot_state_physical():
    """Test physical robot state (no platform)."""
    robot = RobotState(
        robot_id="scout_01",
        robot_type="scout",
        connected=False
    )
    
    assert robot.is_virtual is False
    assert robot.platform is None
    assert robot.connected is False


def test_robot_state_virtual():
    """Test virtual robot state."""
    robot = RobotState(
        robot_id="vbot_01",
        robot_type="scout",
        platform="unity"
    )
    
    assert robot.is_virtual is True
    assert robot.platform == "unity"


def test_state_manager_empty():
    """Test empty state manager."""
    manager = RobotStateManager()
    
    assert len(manager.list_robots()) == 0
    assert manager.get_robot("nonexistent") is None


def test_state_manager_register_multiple():
    """Test registering multiple robots."""
    manager = RobotStateManager()
    
    manager.register_robot("scout_01", "scout")
    manager.register_robot("scout_02", "scout")
    manager.register_robot("go2_01", "go2")
    manager.register_robot("vbot_01", "scout", platform="unity")
    
    robots = manager.list_robots()
    assert len(robots) == 4


def test_state_manager_filter_by_type():
    """Test filtering robots by type."""
    manager = RobotStateManager()
    
    manager.register_robot("scout_01", "scout")
    manager.register_robot("scout_02", "scout")
    manager.register_robot("go2_01", "go2")
    
    scouts = manager.list_robots(robot_type="scout")
    assert len(scouts) == 2
    assert all(r.robot_type == "scout" for r in scouts)
    
    go2s = manager.list_robots(robot_type="go2")
    assert len(go2s) == 1
    assert go2s[0].robot_type == "go2"


def test_state_manager_filter_by_virtual():
    """Test filtering robots by virtual/physical."""
    manager = RobotStateManager()
    
    manager.register_robot("scout_01", "scout")
    manager.register_robot("vbot_01", "scout", platform="unity")
    manager.register_robot("vbot_02", "scout", platform="vrchat")
    
    physical = manager.list_robots(is_virtual=False)
    assert len(physical) == 1
    assert physical[0].robot_id == "scout_01"
    
    virtual = manager.list_robots(is_virtual=True)
    assert len(virtual) == 2
    assert all(r.is_virtual for r in virtual)


def test_state_manager_filter_combined():
    """Test combined filtering."""
    manager = RobotStateManager()
    
    manager.register_robot("scout_01", "scout")
    manager.register_robot("vbot_scout_01", "scout", platform="unity")
    manager.register_robot("vbot_go2_01", "go2", platform="unity")
    
    # Filter: scout + virtual
    result = manager.list_robots(robot_type="scout", is_virtual=True)
    assert len(result) == 1
    assert result[0].robot_id == "vbot_scout_01"


def test_state_manager_update_status():
    """Test updating robot connection status."""
    manager = RobotStateManager()
    manager.register_robot("scout_01", "scout")
    
    robot = manager.get_robot("scout_01")
    assert robot.connected is False
    
    manager.update_robot_status("scout_01", connected=True)
    robot = manager.get_robot("scout_01")
    assert robot.connected is True
    
    manager.update_robot_status("scout_01", connected=False)
    robot = manager.get_robot("scout_01")
    assert robot.connected is False


def test_state_manager_update_status_nonexistent():
    """Test updating status of non-existent robot (should not crash)."""
    manager = RobotStateManager()
    
    # Should not raise exception
    manager.update_robot_status("nonexistent", connected=True)
    
    robot = manager.get_robot("nonexistent")
    assert robot is None


def test_state_manager_unregister_nonexistent():
    """Test unregistering non-existent robot (should not crash)."""
    manager = RobotStateManager()
    
    # Should not raise exception
    manager.unregister_robot("nonexistent")
    
    assert len(manager.list_robots()) == 0


def test_state_manager_register_with_metadata():
    """Test registering robot with metadata."""
    manager = RobotStateManager()
    
    metadata = {
        "position": {"x": 1.0, "y": 2.0, "z": 0.0},
        "scale": 1.0,
        "spawned": True
    }
    
    robot = manager.register_robot(
        "vbot_01",
        "scout",
        platform="unity",
        metadata=metadata
    )
    
    assert robot.metadata == metadata
    assert robot.metadata["position"]["x"] == 1.0


def test_state_manager_duplicate_id_raises():
    """Test that duplicate robot ID raises ValueError."""
    manager = RobotStateManager()
    manager.register_robot("scout_01", "scout")
    
    with pytest.raises(ValueError, match="already registered"):
        manager.register_robot("scout_01", "scout")


def test_state_manager_metadata_default():
    """Test that metadata defaults to empty dict."""
    manager = RobotStateManager()
    robot = manager.register_robot("scout_01", "scout")
    
    assert robot.metadata == {}
    assert isinstance(robot.metadata, dict)

