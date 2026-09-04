"""HTTP client for norirobotics-mcp (Nori A3 gateway on port 11970)."""

from __future__ import annotations

import os
from typing import Any

import aiohttp
import structlog

logger = structlog.get_logger(__name__)

DEFAULT_NORI_MCP_URL = "http://127.0.0.1:11970"


def nori_mcp_url() -> str:
    return os.environ.get("NORI_MCP_URL", DEFAULT_NORI_MCP_URL).rstrip("/")


def mcp_call_succeeded(result: dict[str, Any]) -> bool:
    if result.get("success") is True:
        return True
    if result.get("status") == "success":
        return True
    if result.get("status") == "online":
        return True
    return False


class NoriMcpClient:
    """Proxy to norirobotics-mcp's REST API for the Nori A3 (WebRTC/Supabase, mock-first)."""

    def __init__(self, base_url: str | None = None, timeout: float = 10.0):
        self.base_url = (base_url or nori_mcp_url()).rstrip("/")
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
            logger.error("norirobotics-mcp request failed", method=method, path=path, error=str(exc))
            return {
                "success": False,
                "status": "error",
                "error": f"norirobotics-mcp unreachable at {self.base_url}: {exc}",
                "message": f"norirobotics-mcp unreachable at {self.base_url}: {exc}",
                "error_type": "connection_error",
            }
        except Exception as exc:
            logger.error("norirobotics-mcp unexpected error", method=method, path=path, error=str(exc))
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
        return mcp_call_succeeded(health) or health.get("status") == "ok"

    async def health(self) -> dict[str, Any]:
        return await self._request("GET", "/api/health")

    async def hero(self) -> dict[str, Any]:
        return await self._request("GET", "/api/hero")

    async def session_status(self) -> dict[str, Any]:
        return await self._request("GET", "/api/session")

    async def session_connect(self, *, force_mock: bool = False) -> dict[str, Any]:
        # aiohttp's params= rejects raw bool values ("should be str, int or float") - stringify.
        return await self._request("POST", "/api/session/connect", params={"force_mock": str(force_mock).lower()})

    async def session_disconnect(self) -> dict[str, Any]:
        return await self._request("POST", "/api/session/disconnect")

    async def estop(self) -> dict[str, Any]:
        return await self._request("POST", "/api/control/estop")

    async def episode_start(self, task: str | None = None) -> dict[str, Any]:
        return await self._request("POST", "/api/recording/episode_start", json_body={"task": task})

    async def episode_stop(self) -> dict[str, Any]:
        return await self._request("POST", "/api/recording/episode_stop")

    def build_status_payload(self, session: dict[str, Any]) -> dict[str, Any]:
        # Key is "connection_status", not "status" - format_success_response's `data=` merge
        # would otherwise clobber its own top-level "status": "success" marker with this
        # nested connection-phase dict (found while auditing robot_control.py's Nori handler).
        return {
            "model": "Nori A3",
            "connected": session.get("connected"),
            "mock": session.get("mock"),
            "connection_status": session.get("status"),
            "telemetry": session.get("telemetry"),
            "camera_layout": session.get("camera_layout"),
        }
