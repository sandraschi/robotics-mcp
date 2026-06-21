"""Tests for fleet HTTP client."""

from __future__ import annotations

from unittest.mock import AsyncMock, MagicMock, patch

import pytest


@pytest.mark.asyncio
async def test_call_gimp_tool_parses_data():
    from robotics_mcp.utils.fleet_http_client import call_gimp_tool

    mock_response = MagicMock()
    mock_response.status_code = 200
    mock_response.json.return_value = {"success": True, "data": {"success": True, "count": 2}}
    mock_response.raise_for_status = MagicMock()

    with patch("robotics_mcp.utils.fleet_http_client.httpx.AsyncClient") as client_cls:
        client = MagicMock()
        client.post = AsyncMock(return_value=mock_response)
        client.__aenter__.return_value = client
        client.__aexit__.return_value = None
        client_cls.return_value = client

        result = await call_gimp_tool("gimp_sim_art_tool", {"operation": "list_templates"})

    assert result.get("success") is True
    assert result.get("count") == 2
