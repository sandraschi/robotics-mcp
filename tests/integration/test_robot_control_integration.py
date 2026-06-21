"""Integration tests for robot control tool."""

import pytest

from tests.helpers import patch_yahboom_client


@pytest.fixture(autouse=True)
def stub_yahboom_client():
    with patch_yahboom_client():
        yield


@pytest.mark.integration
@pytest.mark.asyncio
async def test_robot_control_full_workflow(initialized_server):
    robot_id = "integ_yahboom_full"
    initialized_server.state_manager.register_robot(robot_id, "yahboom")

    result = await initialized_server.robot_control.handle_action(robot_id=robot_id, action="get_status", params={})

    assert result["status"] == "success"
    assert result["robot_id"] == robot_id
    assert result["action"] == "get_status"


@pytest.mark.integration
@pytest.mark.asyncio
async def test_robot_control_move_workflow(initialized_server):
    robot_id = "integ_yahboom_move"
    initialized_server.state_manager.register_robot(robot_id, "yahboom")

    result = await initialized_server.robot_control.handle_action(
        robot_id=robot_id, action="move", params={"linear": 0.2, "angular": 0.0}
    )
    assert result["status"] == "success"
    assert result["action"] == "move"

    result = await initialized_server.robot_control.handle_action(robot_id=robot_id, action="stop", params={})
    assert result["status"] == "success"
    assert result["action"] == "stop"


@pytest.mark.integration
@pytest.mark.asyncio
async def test_robot_control_virtual_robot_workflow(initialized_server):
    from unittest.mock import AsyncMock, patch

    initialized_server.state_manager.register_robot("vbot_test_01", "scout", platform="unity")

    with patch(
        "robotics_mcp.tools.robot_control.call_mounted_server_tool",
        new_callable=AsyncMock,
    ) as mock_call:
        mock_call.return_value = {"status": "success", "data": {"robot": {"is_virtual": True, "platform": "unity"}}}
        result = await initialized_server.robot_control.handle_action(
            robot_id="vbot_test_01", action="get_status", params={}
        )

    assert result["status"] == "success"


@pytest.mark.integration
@pytest.mark.asyncio
async def test_robot_control_error_handling(initialized_server):
    result = await initialized_server.robot_control.handle_action(
        robot_id="nonexistent", action="get_status", params={}
    )
    assert result["status"] == "error"
    assert result["error_type"] == "not_found"


@pytest.mark.integration
@pytest.mark.asyncio
async def test_robot_control_multiple_robots(initialized_server):
    initialized_server.state_manager.register_robot("integ_multi_01", "yahboom")
    initialized_server.state_manager.register_robot("integ_multi_02", "yahboom")

    for robot_id in ("integ_multi_01", "integ_multi_02"):
        result = await initialized_server.robot_control.handle_action(robot_id=robot_id, action="get_status", params={})
        assert result["status"] == "success"
        assert result["robot_id"] == robot_id
