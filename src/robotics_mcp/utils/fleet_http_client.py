"""HTTP clients for fleet MCP servers (avoid stdio mount hangs)."""

from __future__ import annotations

import logging
from typing import Any

import httpx

logger = logging.getLogger(__name__)

DEFAULT_GIMP_URL = "http://127.0.0.1:10773"
DEFAULT_AVATAR_URL = "http://127.0.0.1:10793"
DEFAULT_BLENDER_URL = "http://127.0.0.1:10849"
DEFAULT_UNITY_URL = "http://127.0.0.1:10831"
DEFAULT_INKSCAPE_URL = "http://127.0.0.1:10900"


async def check_fleet_health(
    base_url: str,
    *,
    health_paths: tuple[str, ...] = ("/api/v1/health", "/api/health", "/health"),
) -> bool:
    for path in health_paths:
        try:
            async with httpx.AsyncClient(timeout=5.0) as client:
                response = await client.get(base_url.rstrip("/") + path)
                if response.status_code == 200:
                    return True
        except httpx.HTTPError:
            continue
    return False


async def call_gimp_tool(
    tool: str,
    params: dict[str, Any],
    *,
    base_url: str = DEFAULT_GIMP_URL,
    timeout: float = 300.0,
) -> dict[str, Any]:
    """Call gimp-mcp POST /api/v1/tool."""
    url = base_url.rstrip("/") + "/api/v1/tool"
    try:
        async with httpx.AsyncClient(timeout=timeout) as client:
            response = await client.post(url, json={"tool": tool, "params": params})
            response.raise_for_status()
            body = response.json()
    except httpx.HTTPError as exc:
        logger.exception("gimp HTTP tool failed tool=%s", tool)
        return {"success": False, "error": str(exc), "message": str(exc), "tool": tool}

    if isinstance(body, dict) and isinstance(body.get("data"), dict):
        data = body["data"]
        if "success" not in data:
            data = {**data, "success": bool(body.get("success", True))}
        return data
    return (
        body
        if isinstance(body, dict)
        else {"success": False, "error": "Invalid gimp response", "message": "Invalid gimp response"}
    )


async def call_avatar_tool(
    tool_name: str,
    arguments: dict[str, Any],
    *,
    base_url: str = DEFAULT_AVATAR_URL,
    timeout: float = 120.0,
) -> dict[str, Any]:
    """Call avatar-mcp POST /api/v1/tools/execute."""
    url = base_url.rstrip("/") + "/api/v1/tools/execute"
    try:
        async with httpx.AsyncClient(timeout=timeout) as client:
            response = await client.post(
                url,
                json={"tool_name": tool_name, "arguments": arguments},
            )
            response.raise_for_status()
            body = response.json()
    except httpx.HTTPError as exc:
        logger.exception("avatar HTTP tool failed tool=%s", tool_name)
        return {"success": False, "status": "error", "error": str(exc), "message": str(exc), "tool": tool_name}

    if isinstance(body, dict) and body.get("status") == "success":
        result = body.get("result")
        if isinstance(result, dict):
            return {**result, "success": result.get("status") == "success" or result.get("success", True)}
        return {"success": True, "result": result}
    if isinstance(body, dict):
        return {**body, "success": body.get("status") == "success"}
    return {"success": False, "error": "Invalid avatar response", "message": "Invalid avatar response"}


async def call_inkscape_tool(
    tool: str,
    params: dict[str, Any],
    *,
    base_url: str = DEFAULT_INKSCAPE_URL,
    timeout: float = 300.0,
) -> dict[str, Any]:
    """Call inkscape-mcp POST /api/v1/tool."""
    url = base_url.rstrip("/") + "/api/v1/tool"
    try:
        async with httpx.AsyncClient(timeout=timeout) as client:
            response = await client.post(url, json={"tool": tool, "params": params})
            response.raise_for_status()
            body = response.json()
    except httpx.HTTPError as exc:
        logger.exception("inkscape HTTP tool failed tool=%s", tool)
        return {"success": False, "error": str(exc), "message": str(exc), "tool": tool}

    if isinstance(body, dict) and isinstance(body.get("data"), dict):
        data = body["data"]
        if "success" not in data:
            data = {**data, "success": bool(body.get("success", True))}
        return data
    return (
        body
        if isinstance(body, dict)
        else {"success": False, "error": "Invalid inkscape response", "message": "Invalid inkscape response"}
    )
