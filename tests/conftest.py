"""Shared pytest fixtures for robotics-mcp."""

from __future__ import annotations

import pytest
from fastapi.testclient import TestClient

from robotics_mcp.server import RoboticsConfig, RoboticsMCP
from robotics_mcp.utils.auth import DEFAULT_BENNY_TOKEN
from robotics_mcp.utils.state_manager import RobotStateManager
from tests.helpers import make_mock_mcp
from tests.helpers import mock_ctx as _mock_ctx


@pytest.fixture
def mock_mcp():
    return make_mock_mcp()


@pytest.fixture
def mock_ctx():
    return _mock_ctx()


@pytest.fixture
def state_manager() -> RobotStateManager:
    return RobotStateManager()


@pytest.fixture
def mcp_server():
    """Industrialized MCP Server fixture (HTTP routes + tools initialized)."""
    import asyncio

    config = RoboticsConfig(enable_http=True, http_port=10892, log_level="DEBUG")
    server = RoboticsMCP(config=config)
    asyncio.run(server.initialize_async())
    return server


@pytest.fixture
async def initialized_server():
    """Server with tools registered (initialize_async completed)."""
    config = RoboticsConfig(enable_http=False)
    server = RoboticsMCP(config)
    await server.initialize_async()
    return server


@pytest.fixture
def api_client(mcp_server):
    """FastAPI Test Client fixture."""
    return TestClient(mcp_server.http_app, raise_server_exceptions=False)


@pytest.fixture
def auth_headers():
    """Standard SOTA Wurst-Auth headers."""
    return {"X-Wurst-Auth": DEFAULT_BENNY_TOKEN}


@pytest.fixture
def mock_robot(mcp_server):
    """Registers a mock robot for testing."""
    robot_id = "test-bot-9000"
    mcp_server.state_manager.register_robot(
        robot_id=robot_id, robot_type="test_mock", platform="mock", metadata={"name": "Testing Sentinel"}
    )
    return robot_id
