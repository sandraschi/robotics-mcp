"""Integration tests for virtual robotics."""

from unittest.mock import AsyncMock, MagicMock, patch

import pytest


@pytest.mark.integration
@pytest.mark.asyncio
async def test_spawn_virtual_robot(initialized_server):
    with patch(
        "robotics_mcp.tools.robot_virtual.call_mounted_server_tool",
        new_callable=AsyncMock,
    ) as mock_call:
        mock_call.return_value = {"status": "success"}
        result = await initialized_server.robot_virtual._handle_create(
            robot_type="yahboom",
            robot_id="test_yahboom_01",
            platform="resonite",
            position={"x": 0, "y": 0, "z": 0},
            scale=1.0,
            metadata=None,
            model_path=None,
        )

    assert result["status"] == "success"
    assert result["robot_id"] == "test_yahboom_01"
    robot = initialized_server.state_manager.get_robot("test_yahboom_01")
    assert robot is not None
    assert robot.is_virtual is True
    assert robot.platform == "resonite"


@pytest.mark.integration
@pytest.mark.asyncio
async def test_virtual_robot_movement(initialized_server):
    initialized_server.state_manager.register_robot("test_scout_01", "scout", platform="unity")
    initialized_server.robot_control.mounted_servers = {"avatar": MagicMock()}
    with patch(
        "robotics_mcp.tools.robot_control.call_mounted_server_tool",
        new_callable=AsyncMock,
    ) as mock_call:
        mock_call.return_value = {"status": "success"}
        result = await initialized_server.robot_control.handle_action(
            robot_id="test_scout_01",
            action="move",
            params={"linear": 0.2, "angular": 0.0},
        )

    assert result is not None
    assert result.get("status") == "success"
    assert result.get("action") == "move"
    assert mock_call.called


@pytest.mark.integration
@pytest.mark.asyncio
async def test_load_marble_environment(initialized_server):
    initialized_server.robot_virtual.mounted_servers = {"unity": MagicMock()}
    with patch(
        "robotics_mcp.tools.robot_virtual.call_mounted_server_tool",
        new_callable=AsyncMock,
    ) as mock_call:
        mock_call.return_value = {"status": "success"}
        result = await initialized_server.robot_virtual._handle_load_environment(
            environment="test_apartment",
            platform="unity",
            environment_path=None,
            project_path=None,
            include_colliders=True,
        )

    assert result["status"] == "success"


@pytest.mark.integration
@pytest.mark.asyncio
async def test_virtual_lidar(initialized_server):
    initialized_server.state_manager.register_robot("test_scout_01", "scout", platform="unity")
    initialized_server.robot_virtual.mounted_servers = {"unity": MagicMock()}
    with patch(
        "robotics_mcp.tools.robot_virtual.call_mounted_server_tool",
        new_callable=AsyncMock,
    ) as mock_call:
        mock_call.return_value = {"status": "success", "data": {"scan_data": []}}
        result = await initialized_server.robot_virtual._handle_get_lidar("test_scout_01")

    assert result["status"] == "success"
