"""UDP OSC bridge for virtual robots (Resonite / VRChat world receivers)."""

from __future__ import annotations

import asyncio
import logging
import os
from collections.abc import Callable
from typing import Any, ClassVar

from pythonosc.udp_client import SimpleUDPClient

logger = logging.getLogger(__name__)

# Fleet teleop contract — see teleoperator-mcp/docs/resonite/VBOOMY_OSC.md
VBOOMY_SPAWN_ADDRESS = "/resonite/vbot/spawn"
VBOOMY_RESET_PREFIX = "/robot/{robot_id}/reset"
VBOOMY_MOVE_PREFIX = "/robot/{robot_id}/move"
VBOOMY_STOP_PREFIX = "/robot/{robot_id}/stop"
VBOOMY_HEAD_PREFIX = "/robot/{robot_id}/head"
VBOOMY_ESTOP_ADDRESS = "/fleet/emergency_stop"


class OSCBridge:
    """Send OSC to Resonite (default 127.0.0.1:9000)."""

    ADDRESS_PATTERNS: ClassVar[dict[str, str]] = {
        "unity": "/unity/robot/{robot_type}/{param}",
        "vrchat": "/avatar/parameters/{robot_type}_{param}",
        "resonite": "/resonite/avatar/{robot_type}_{param}",
    }

    def __init__(self) -> None:
        self.host = os.environ.get("ROBOTICS_OSC_HOST", "127.0.0.1")
        self.port = int(os.environ.get("ROBOTICS_OSC_PORT", "9000"))
        self._client: SimpleUDPClient | None = None
        self._subscriptions: dict[str, list[Callable]] = {}
        self._connected = False
        self.messages_sent: list[tuple[str, list[Any]]] = []

    def _client_or_create(self) -> SimpleUDPClient:
        if self._client is None:
            self._client = SimpleUDPClient(self.host, self.port)
        return self._client

    async def connect(self) -> None:
        self._connected = True
        logger.info("OSC bridge ready at %s:%s", self.host, self.port)

    async def send_command(self, platform: str, robot_type: str, param: str, value: Any) -> bool:
        if not self._connected:
            await self.connect()
        address = self._get_address(platform, robot_type, param)
        return await self.send_message(address, [value])

    def _get_address(self, platform: str, robot_type: str, param: str) -> str:
        pattern = self.ADDRESS_PATTERNS.get(platform)
        if not pattern:
            raise ValueError(f"Unsupported platform: {platform}")
        return pattern.format(robot_type=robot_type, param=param)

    async def send_message(self, address: str, args: list[Any]) -> bool:
        if not self._connected:
            await self.connect()

        def _send() -> None:
            client = self._client_or_create()
            if args:
                client.send_message(address, args)
            else:
                client.send_message(address, [])

        try:
            loop = asyncio.get_event_loop()
            await loop.run_in_executor(None, _send)
            self.messages_sent.append((address, list(args)))
            logger.info("OSC send %s:%s %s %s", self.host, self.port, address, args)
            return True
        except Exception as exc:
            logger.warning("OSC send failed %s: %s", address, exc)
            return False

    async def spawn_vbot(
        self,
        robot_id: str,
        robot_type: str,
        *,
        x: float = 0.0,
        y: float = 0.0,
        z: float = 0.0,
        scale: float = 1.0,
    ) -> bool:
        ok = await self.send_message(
            VBOOMY_SPAWN_ADDRESS,
            [robot_id, robot_type, float(x), float(y), float(z), float(scale)],
        )
        await self.send_message(VBOOMY_RESET_PREFIX.format(robot_id=robot_id), [1.0])
        return ok

    async def subscribe(self, robot_id: str, callback: Callable) -> None:
        if robot_id not in self._subscriptions:
            self._subscriptions[robot_id] = []
        self._subscriptions[robot_id].append(callback)


osc_bridge = OSCBridge()
