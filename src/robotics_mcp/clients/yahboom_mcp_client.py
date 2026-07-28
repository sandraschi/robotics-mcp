"""HTTP client for yahboom-mcp (ROS 2 Yahboom gateway on port 10892)."""

from __future__ import annotations

import os
from typing import Any

import aiohttp
import structlog

logger = structlog.get_logger(__name__)

DEFAULT_YAHBOOM_MCP_URL = "http://127.0.0.1:10892"


def yahboom_mcp_url() -> str:
    return os.environ.get("YAHBOOM_MCP_URL", DEFAULT_YAHBOOM_MCP_URL).rstrip("/")


def mcp_call_succeeded(result: dict[str, Any]) -> bool:
    if result.get("success") is True:
        return True
    if result.get("status") == "success":
        return True
    if result.get("status") == "online":
        return True
    return False


class YahboomMcpClient:
    """Proxy to the yahboom-mcp unified gateway (REST + MCP on same host)."""

    def __init__(self, base_url: str | None = None, timeout: float = 10.0):
        self.base_url = (base_url or yahboom_mcp_url()).rstrip("/")
        self._timeout = aiohttp.ClientTimeout(total=timeout)

    async def _request(
        self,
        method: str,
        path: str,
        *,
        params: dict[str, Any] | None = None,
        json_body: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        url = f"{self.base_url}{path}"
        try:
            async with aiohttp.ClientSession(timeout=self._timeout) as session:
                if method == "GET":
                    async with session.get(url, params=params) as response:
                        return await self._parse_response(response)
                async with session.post(url, params=params, json=json_body) as response:
                    return await self._parse_response(response)
        except aiohttp.ClientError as exc:
            logger.error("yahboom-mcp request failed", method=method, path=path, error=str(exc))
            return {
                "success": False,
                "status": "error",
                "error": f"yahboom-mcp unreachable at {self.base_url}: {exc}",
                "message": f"yahboom-mcp unreachable at {self.base_url}: {exc}",
                "error_type": "connection_error",
            }
        except Exception as exc:
            logger.error("yahboom-mcp unexpected error", method=method, path=path, error=str(exc))
            return {"success": False, "status": "error", "error": str(exc), "message": str(exc), "error_type": "error"}

    async def _parse_response(self, response: aiohttp.ClientResponse) -> dict[str, Any]:
        if response.status >= 400:
            text = await response.text()
            try:
                payload = await response.json()
                detail = payload.get("detail", payload.get("error", text))
            except Exception:
                detail = text or f"HTTP {response.status}"
            return {
                "success": False,
                "status": "error",
                "error": str(detail),
                "message": str(detail),
                "error_type": "http_error",
                "status_code": response.status,
            }

        content_type = (response.headers.get("Content-Type") or "").lower()
        if "image" in content_type:
            data = await response.read()
            return {"success": True, "status": "success", "frame": data, "frame_size": len(data)}

        if response.status == 204:
            return {
                "success": False,
                "status": "error",
                "error": "No camera frame available yet",
                "message": "No camera frame available yet",
            }

        if not response.content_length and response.status == 200:
            body = await response.text()
            if not body:
                return {"success": True, "status": "success"}
            try:
                import json

                data = json.loads(body)
                return data if isinstance(data, dict) else {"success": True, "data": data}
            except Exception:
                return {"success": True, "status": "success", "message": body}

        try:
            data = await response.json()
            return data if isinstance(data, dict) else {"success": True, "data": data}
        except Exception:
            text = await response.text()
            return {"success": True, "status": "success", "message": text}

    def is_reachable(self, health: dict[str, Any]) -> bool:
        return mcp_call_succeeded(health) or bool(health.get("system"))

    def is_live_telemetry(self, telemetry: dict[str, Any]) -> bool:
        if telemetry.get("source") == "simulated":
            return False
        if telemetry.get("status") == "offline":
            return False
        return telemetry.get("source") == "live" or telemetry.get("status") == "live"

    async def health(self) -> dict[str, Any]:
        return await self._request("GET", "/api/v1/health")

    async def telemetry(self) -> dict[str, Any]:
        return await self._request("GET", "/api/v1/telemetry")

    async def move(self, linear: float = 0.0, angular: float = 0.0, linear_y: float = 0.0) -> dict[str, Any]:
        return await self._request(
            "POST",
            "/api/v1/control/move",
            params={"linear": linear, "angular": angular, "linear_y": linear_y},
        )

    async def stop_all(self) -> dict[str, Any]:
        return await self._request("POST", "/api/v1/stop_all")

    async def call_tool(
        self,
        operation: str,
        *,
        param1: str | float | None = None,
        param2: str | float | None = None,
        param3: str | float | None = None,
        payload: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        return await self._request(
            "POST",
            "/api/v1/control/tool",
            json_body={
                "operation": operation,
                "param1": param1,
                "param2": param2,
                "param3": param3,
                "payload": payload,
            },
        )

    async def agent_mission(
        self,
        goal: str,
        *,
        provider: str = "auto",
        publish_to_ros: bool = True,
        speak: bool = False,
    ) -> dict[str, Any]:
        return await self._request(
            "POST",
            "/api/v1/agent/mission",
            json_body={
                "goal": goal,
                "provider": provider,
                "publish_to_ros": publish_to_ros,
                "speak": speak,
            },
        )

    async def snapshot(self) -> dict[str, Any]:
        return await self._request("GET", "/api/v1/snapshot")

    def build_status_payload(self, health: dict[str, Any], telemetry: dict[str, Any]) -> dict[str, Any]:
        robot_conn = health.get("robot_connection") or {}
        return {
            "model": "ROSMaster-M1",
            "battery": telemetry.get("battery"),
            "voltage": telemetry.get("voltage"),
            "sensors": {
                "imu": telemetry.get("imu"),
                "velocity": telemetry.get("velocity"),
                "scan": telemetry.get("scan"),
            },
            "robot_connection": robot_conn,
            "telemetry_status": telemetry.get("status"),
            "source": telemetry.get("source"),
        }
