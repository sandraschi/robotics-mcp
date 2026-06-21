"""Integration tests for server tools (help, status, list_robots)."""

from unittest.mock import MagicMock

import pytest

from tests.helpers import patch_yahboom_client, payload


@pytest.fixture(autouse=True)
def stub_yahboom_client():
    with patch_yahboom_client():
        yield


@pytest.mark.integration
@pytest.mark.asyncio
async def test_help_tool(initialized_server):

    result = await initialized_server.robotics_system._handle_help()

    assert result["status"] == "success"

    data = result if "server_name" in result else payload(result)

    assert data.get("server_name") == "Robotics-MCP" or "server" in data


@pytest.mark.integration
@pytest.mark.asyncio
async def test_get_status_tool(initialized_server):

    initialized_server.state_manager.register_robot("test_robot", "scout")

    result = await initialized_server.robotics_system._handle_status()

    assert result["success"] is True

    assert payload(result)["robots_count"] >= 1


@pytest.mark.integration
@pytest.mark.asyncio
async def test_list_robots_tool(initialized_server):

    initialized_server.state_manager.register_robot("scout_01", "scout")

    initialized_server.state_manager.register_robot("vbot_01", "scout", platform="unity")

    initialized_server.state_manager.register_robot("go2_01", "go2")

    result = await initialized_server.robotics_system._handle_list_robots(None, None)

    assert result["success"] is True

    assert payload(result)["count"] >= 3

    result = await initialized_server.robotics_system._handle_list_robots("scout", None)

    assert payload(result)["count"] >= 2

    result = await initialized_server.robotics_system._handle_list_robots(None, True)

    assert payload(result)["count"] >= 1

    assert payload(result)["robots"][0]["is_virtual"] is True


@pytest.mark.integration
@pytest.mark.asyncio
async def test_robot_control_tool(initialized_server):

    robot_id = "integ_server_yahboom"
    initialized_server.state_manager.register_robot(robot_id, "yahboom")

    result = await initialized_server.robot_control.handle_action(robot_id=robot_id, action="get_status", params={})

    assert result["status"] == "success"
    assert result["robot_id"] == robot_id

    assert result["action"] == "get_status"


@pytest.mark.integration
@pytest.mark.asyncio
async def test_virtual_robotics_tool(initialized_server):

    from unittest.mock import AsyncMock, patch

    initialized_server.robot_virtual.mounted_servers = {"unity": MagicMock()}
    with patch(
        "robotics_mcp.tools.robot_virtual.call_mounted_server_tool",
        new_callable=AsyncMock,
    ) as mock_call:
        mock_call.return_value = {"status": "success"}

        result = await initialized_server.robot_virtual._handle_create(
            robot_type="scout",
            robot_id=None,
            platform="unity",
            position={"x": 0.0, "y": 0.0, "z": 0.0},
            scale=1.0,
            metadata=None,
            model_path=None,
        )

    assert result["status"] == "success"

    assert "robot_id" in result

    robot_id = result["robot_id"]

    read = await initialized_server.robot_virtual._handle_read(robot_id)

    assert read["status"] == "success"

    assert read["robot_id"] == robot_id


@pytest.mark.integration
@pytest.mark.asyncio
async def test_tool_error_handling(initialized_server):

    result = await initialized_server.robot_control.handle_action(
        robot_id="nonexistent", action="get_status", params={}
    )

    assert result["status"] == "error"

    assert result["error_type"] == "not_found"


@pytest.mark.integration
@pytest.mark.asyncio
async def test_server_with_http_enabled():

    from robotics_mcp.server import RoboticsConfig, RoboticsMCP

    config = RoboticsConfig(enable_http=True, http_port=12231)

    server = RoboticsMCP(config)

    await server.initialize_async()

    assert server.http_app is not None

    result = await server.robotics_system._handle_help()

    assert result["status"] == "success"
