"""OSC bridge unit tests."""

from __future__ import annotations

from unittest.mock import MagicMock, patch

import pytest

from robotics_mcp.osc_bridge import VBOOMY_RESET_PREFIX, OSCBridge


@pytest.mark.asyncio
async def test_send_message_records_payload() -> None:
    bridge = OSCBridge()
    mock_client = MagicMock()

    with patch.object(bridge, "_client_or_create", return_value=mock_client):
        ok = await bridge.send_message("/robot/vbot_yahboom_01/move", [0.2, 0.0])

    assert ok is True
    mock_client.send_message.assert_called_once_with("/robot/vbot_yahboom_01/move", [0.2, 0.0])
    assert bridge.messages_sent[-1] == ("/robot/vbot_yahboom_01/move", [0.2, 0.0])


@pytest.mark.asyncio
async def test_spawn_vbot_sends_spawn_and_reset() -> None:
    bridge = OSCBridge()
    calls: list[tuple[str, list]] = []

    async def fake_send(address: str, args: list) -> bool:
        calls.append((address, args))
        return True

    with patch.object(bridge, "send_message", side_effect=fake_send):
        ok = await bridge.spawn_vbot("vbot_yahboom_01", "yahboom", x=1.0, y=2.0, z=0.0, scale=1.0)

    assert ok is True
    assert calls[0][0] == "/resonite/vbot/spawn"
    assert calls[1] == (VBOOMY_RESET_PREFIX.format(robot_id="vbot_yahboom_01"), [1.0])
