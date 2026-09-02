"""Test helpers (not fixtures)."""

from __future__ import annotations

from contextlib import contextmanager
from typing import Any
from unittest.mock import AsyncMock, MagicMock, patch

SPAWN_MOCK = {"status": "success"}


def make_mock_mcp() -> MagicMock:
    def tool_decorator(*args, **kwargs):
        def decorator(func):
            tool_decorator.registered_func = func
            return func

        return decorator

    mcp = MagicMock()
    mcp.tool = tool_decorator
    mcp._tools = {}
    # list_tools() is a real async API (awaited by robotics_system.py's help operation) - a
    # plain MagicMock attribute isn't awaitable and raised TypeError, which was being silently
    # swallowed into an error-path response rather than failing the test loudly. Found
    # 2026-09-03 via test_robotics_system_help, which had been exercising the error path this
    # whole time instead of the success path it was meant to test.
    mcp.list_tools = AsyncMock(return_value=[])
    return mcp


def registered_tool(tool_instance):
    return tool_instance.mcp.tool.registered_func


def mock_ctx() -> MagicMock:
    return MagicMock()


def is_success(result: dict[str, Any]) -> bool:
    if "status" in result:
        return result["status"] == "success"
    return result.get("success") is True


def is_error(result: dict[str, Any]) -> bool:
    if "status" in result:
        return result["status"] == "error"
    return result.get("success") is False


def payload(result: dict[str, Any]) -> dict[str, Any]:
    """Return nested payload for either response format."""
    if isinstance(result.get("result"), dict):
        return result["result"]
    if isinstance(result.get("data"), dict):
        return result["data"]
    skip = {
        "status",
        "message",
        "error_type",
        "error",
        "success",
        "operation",
        "summary",
        "recommendations",
        "next_steps",
        "error_code",
        "recovery_options",
        "suggestions",
    }
    return {k: v for k, v in result.items() if k not in skip}


class _FakeYahboomMcpClient:
    """Test double for YahboomMcpClient — simulates a live yahboom-mcp gateway."""

    def __init__(self, base_url: str | None = None, timeout: float = 10.0):
        self.base_url = (base_url or "http://127.0.0.1:10892").rstrip("/")
        self._timeout = timeout

    def is_reachable(self, health: dict[str, Any]) -> bool:
        return health.get("success") is True or bool(health.get("system"))

    def is_live_telemetry(self, telemetry: dict[str, Any]) -> bool:
        return telemetry.get("source") == "live"

    def build_status_payload(self, health: dict[str, Any], telemetry: dict[str, Any]) -> dict[str, Any]:
        return {
            "model": "ROSMaster-M1",
            "battery": telemetry.get("battery"),
            "voltage": telemetry.get("voltage"),
            "sensors": {
                "imu": telemetry.get("imu"),
                "velocity": telemetry.get("velocity"),
                "scan": telemetry.get("scan"),
            },
            "robot_connection": health.get("robot_connection") or {},
            "telemetry_status": telemetry.get("status"),
            "source": telemetry.get("source"),
        }

    async def health(self) -> dict[str, Any]:
        return {
            "success": True,
            "status": "online",
            "system": {"yahboom_mcp": "ok"},
            "robot_connection": {"connected": True},
        }

    async def telemetry(self) -> dict[str, Any]:
        return {
            "success": True,
            "status": "live",
            "source": "live",
            "battery": 85.0,
            "voltage": 12.4,
            "imu": "ok",
            "velocity": {"linear": 0.0, "angular": 0.0},
            "scan": {"ranges": []},
        }

    async def move(self, linear: float = 0.0, angular: float = 0.0, linear_y: float = 0.0) -> dict[str, Any]:
        return {"success": True, "status": "success", "linear": linear, "angular": angular, "linear_y": linear_y}

    async def stop_all(self) -> dict[str, Any]:
        return {"success": True, "status": "success", "stopped": True}

    async def agent_mission(
        self,
        goal: str,
        *,
        provider: str = "auto",
        publish_to_ros: bool = True,
        speak: bool = False,
    ) -> dict[str, Any]:
        plan: dict[str, Any] = {"goal": goal, "provider": provider, "speak": speak}
        if "patrol" in goal.lower():
            plan["waypoints"] = [
                {"x": 0.0, "y": 0.0, "description": "living room"},
                {"x": 3.0, "y": 0.0, "description": "kitchen"},
                {"x": 3.0, "y": 2.0, "description": "bedroom"},
                {"x": 0.0, "y": 2.0, "description": "return to base"},
            ]
        return {"success": True, "status": "success", "plan": plan, "goal": goal}

    async def snapshot(self) -> dict[str, Any]:
        return {"success": True, "status": "success", "frame": b"fakeframe", "frame_size": 9}


@contextmanager
def patch_yahboom_mcp_client(module: str = "robotics_mcp.tools.robot_control"):
    with patch(f"{module}.YahboomMcpClient", _FakeYahboomMcpClient):
        yield


@contextmanager
def patch_yahboom_client(module: str = "robotics_mcp.tools.robot_control"):
    """Backward-compatible alias — patches YahboomMcpClient."""
    with patch_yahboom_mcp_client(module):
        yield


@contextmanager
def patch_fastmcp_client(module: str, return_value: Any | None = None):
    """Patch fastmcp Client used by legacy tool modules."""
    mock_client_instance = AsyncMock()
    mock_client_instance.call_tool = AsyncMock(return_value=return_value or {"status": "success"})
    with patch(f"{module}.Client") as mock_client:
        mock_client.return_value.__aenter__.return_value = mock_client_instance
        yield mock_client_instance
