"""DreameHome cloud client for Dreame D20 Pro Plus (no miio token).

Adapted from dreame-mcp. Requires Tasshack ref clone at DREAME_REF_PATH.
Env: DREAME_USER, DREAME_PASSWORD; optional: DREAME_COUNTRY, DREAME_DID, DREAME_AUTH_KEY, DREAME_REF_PATH.
"""

from __future__ import annotations

import asyncio
import base64
import contextlib
import importlib.util
import os
import sys
import types as _types
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import structlog

logger = structlog.get_logger(__name__)

_REF_DEFAULT = Path("D:/Dev/repos/tasshack_dreame_vacuum_ref")
_DREAME_PKG = "custom_components.dreame_vacuum.dreame"

_protocol_cls = None
_map_manager_cls = None


def _stub_miio() -> None:
    if "miio" in sys.modules:
        return
    miio_mod = _types.ModuleType("miio")
    proto_mod = _types.ModuleType("miio.miioprotocol")

    class _MiIOProtocol:
        def __init__(self, *a: Any, **kw: Any) -> None:
            pass

    proto_mod.MiIOProtocol = _MiIOProtocol
    sys.modules["miio"] = miio_mod
    sys.modules["miio.miioprotocol"] = proto_mod


def _stub_ha() -> None:
    for mod in [
        "homeassistant",
        "homeassistant.core",
        "homeassistant.helpers",
        "homeassistant.helpers.entity",
        "homeassistant.components",
    ]:
        if mod not in sys.modules:
            sys.modules[mod] = _types.ModuleType(mod)


def _load_module(name: str, path: Path) -> Any:
    spec = importlib.util.spec_from_file_location(name, str(path))
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


def _bootstrap_tasshack(ref_path: Path) -> None:
    global _protocol_cls, _map_manager_cls
    if _protocol_cls is not None:
        return

    dreame_dir = ref_path / "custom_components" / "dreame_vacuum" / "dreame"
    if not dreame_dir.exists():
        raise RuntimeError(f"Tasshack ref clone not found at: {dreame_dir}")

    _stub_miio()
    _stub_ha()

    for pkg in ["custom_components", "custom_components.dreame_vacuum", _DREAME_PKG]:
        if pkg not in sys.modules:
            sys.modules[pkg] = _types.ModuleType(pkg)

    exc_mod = _load_module(f"{_DREAME_PKG}.exceptions", dreame_dir / "exceptions.py")
    dreame_stub = sys.modules[_DREAME_PKG]
    dreame_stub.VERSION = "robotics-mcp-adapter"
    dreame_stub.DeviceException = exc_mod.DeviceException
    dreame_stub.DeviceUpdateFailedException = exc_mod.DeviceUpdateFailedException

    proto_mod = _load_module(f"{_DREAME_PKG}.protocol", dreame_dir / "protocol.py")
    _protocol_cls = proto_mod.DreameVacuumDreameHomeCloudProtocol

    try:
        _load_module(f"{_DREAME_PKG}.const", dreame_dir / "const.py")
        _load_module(f"{_DREAME_PKG}.types", dreame_dir / "types.py")
        _load_module(f"{_DREAME_PKG}.resources", dreame_dir / "resources.py")
        map_mod = _load_module(f"{_DREAME_PKG}.map", dreame_dir / "map.py")
        _map_manager_cls = getattr(map_mod, "DreameVacuumMapManager", None)
        logger.info("Tasshack map module loaded (map rendering available)")
    except Exception as e:
        logger.warning("Tasshack map load failed - map rendering unavailable", error=str(e))
        _map_manager_cls = None

    logger.info("Tasshack protocol loaded", ref_path=str(ref_path))


@dataclass
class DreameCloudStatus:
    """Status from DreameHome cloud."""

    state: str = "unknown"
    battery: int = 0
    fan_speed: str = "unknown"
    is_charging: bool = False
    is_cleaning: bool = False
    cleaned_area: float = 0.0
    cleaning_time: int = 0
    error: str | None = None
    raw: dict = field(default_factory=dict)


_PROP_STATE = {"did": None, "siid": 2, "piid": 1}
_PROP_ERROR = {"did": None, "siid": 2, "piid": 2}
_PROP_BATTERY = {"did": None, "siid": 3, "piid": 1}
_PROP_CHARGING = {"did": None, "siid": 3, "piid": 2}
_PROP_STATUS = {"did": None, "siid": 4, "piid": 1}
_PROP_TIME = {"did": None, "siid": 4, "piid": 2}
_PROP_AREA = {"did": None, "siid": 4, "piid": 3}
_PROP_FANSPEED = {"did": None, "siid": 4, "piid": 4}

_STATE_MAP = {
    0: "idle",
    1: "cleaning",
    2: "returning",
    3: "charging",
    4: "charging_error",
    5: "paused",
    6: "sweeping_and_mopping",
    7: "mopping",
    8: "drying",
    9: "self_cleaning",
    10: "remote_control",
    11: "fast_mapping",
    12: "pending",
    17: "docked",
}

_ACTION_MAP = {
    "start_clean": (2, 1),
    "pause": (2, 2),
    "go_home": (3, 1),
    "stop": (4, 2),
    "find_robot": (7, 1),
}


class DreameHomeClient:
    """Async wrapper around DreameVacuumDreameHomeCloudProtocol (no local token)."""

    def __init__(
        self,
        username: str,
        password: str,
        country: str = "eu",
        did: str | None = None,
        auth_key: str | None = None,
        ref_path: Path | None = None,
    ) -> None:
        self._username = username
        self._password = password
        self._country = country
        self._did = did
        self._auth_key = auth_key
        self._ref_path = ref_path or _REF_DEFAULT
        self._protocol = None
        self._executor = ThreadPoolExecutor(max_workers=2, thread_name_prefix="dreame")
        self._map_manager = None

    async def connect(self) -> bool:
        loop = asyncio.get_event_loop()
        return await loop.run_in_executor(self._executor, self._connect_sync)

    def _connect_sync(self) -> bool:
        try:
            _bootstrap_tasshack(self._ref_path)
        except Exception as e:
            logger.error("Bootstrap failed", error=str(e))
            return False

        self._protocol = _protocol_cls(
            username=self._username,
            password=self._password,
            country=self._country,
            account_type="dreame",
            auth_key=self._auth_key,
            did=str(self._did) if self._did else None,
        )

        ok = self._protocol.login()
        if not ok:
            logger.error("DreameHome login failed")
            return False
        logger.info("DreameHome login OK", auth_key_prefix=(self._protocol.auth_key or "")[:20])

        if not self._did:
            devices = self._protocol.get_devices()
            if not devices:
                logger.error("No devices from cloud")
                return False
            records = devices.get("page", {}).get("records", [])
            if not records:
                logger.error("Device list empty")
                return False
            device = records[0]
            self._did = str(device.get("did", ""))
            name = device.get("customName") or device.get("deviceInfo", {}).get("displayName", "?")
            logger.info("Auto-selected device", name=name, did=self._did)
            self._protocol._did = self._did

        try:
            info = self._protocol.connect()
            if info:
                logger.info("MQTT connected", host=getattr(self._protocol, "_host", None))
        except Exception as e:
            logger.warning("MQTT connect failed (polling still works)", error=str(e))

        if _map_manager_cls is not None:
            try:
                self._map_manager = _map_manager_cls(self._protocol)
                logger.info("Map manager initialized")
            except Exception as e:
                logger.warning("Map manager init failed", error=str(e))

        return True

    def disconnect(self) -> None:
        if self._protocol:
            with contextlib.suppress(Exception):
                self._protocol.disconnect()
        self._executor.shutdown(wait=False)

    @property
    def connected(self) -> bool:
        return self._protocol is not None and self._protocol.logged_in

    async def get_status(self) -> DreameCloudStatus:
        loop = asyncio.get_event_loop()
        return await loop.run_in_executor(self._executor, self._get_status_sync)

    def _get_status_sync(self) -> DreameCloudStatus:
        if not self._protocol:
            return DreameCloudStatus(error="Not connected")
        try:
            did = str(self._did)
            props = [
                {**_PROP_STATE, "did": did},
                {**_PROP_ERROR, "did": did},
                {**_PROP_BATTERY, "did": did},
                {**_PROP_CHARGING, "did": did},
                {**_PROP_STATUS, "did": did},
                {**_PROP_TIME, "did": did},
                {**_PROP_AREA, "did": did},
                {**_PROP_FANSPEED, "did": did},
            ]
            result = self._protocol.send("get_properties", props)
            if result is None:
                return DreameCloudStatus(error="No response from device")

            raw = {f"{r['siid']}.{r['piid']}": r.get("value") for r in result if "value" in r}
            state_code = raw.get("2.1", 0)
            battery = raw.get("3.1", 0)
            charging_code = raw.get("3.2", 0)
            fan_speed = raw.get("4.4", 0)
            time_s = raw.get("4.2", 0) or 0
            area_cm2 = raw.get("4.3", 0) or 0

            state_str = _STATE_MAP.get(state_code, f"state_{state_code}")
            is_charging = charging_code in (1, 2)
            is_cleaning = state_code in (1, 6, 7, 11)

            return DreameCloudStatus(
                state=state_str,
                battery=int(battery),
                fan_speed=str(fan_speed),
                is_charging=is_charging,
                is_cleaning=is_cleaning,
                cleaned_area=round(area_cm2 / 10000, 2),
                cleaning_time=int(time_s),
                raw=raw,
            )
        except Exception as e:
            logger.exception("get_status failed")
            return DreameCloudStatus(error=str(e))

    async def control(self, cmd: str) -> dict[str, Any]:
        loop = asyncio.get_event_loop()
        return await loop.run_in_executor(self._executor, self._control_sync, cmd)

    def _control_sync(self, cmd: str) -> dict[str, Any]:
        if not self._protocol:
            return {"success": False, "error": "Not connected", "message": "Not connected"}
        if cmd not in _ACTION_MAP:
            return {"success": False, "error": f"Unknown command: {cmd}", "message": f"Unknown command: {cmd}"}
        try:
            siid, aiid = _ACTION_MAP[cmd]
            result = self._protocol.send(
                "action",
                {"did": str(self._did), "siid": siid, "aiid": aiid, "in": []},
            )
            if result is not None:
                return {"success": True, "message": f"Sent {cmd}", "result": result}
            return {"success": False, "error": f"No response for {cmd}", "message": f"No response for {cmd}"}
        except Exception as e:
            logger.exception("control failed", cmd=cmd)
            return {"success": False, "error": str(e), "message": str(e)}

    async def get_map(self) -> dict[str, Any]:
        loop = asyncio.get_event_loop()
        return await loop.run_in_executor(self._executor, self._get_map_sync)

    def _get_map_sync(self) -> dict[str, Any]:
        if not self._protocol:
            return {"success": False, "error": "Not connected", "message": "Not connected"}
        try:
            object_name = self._protocol.object_name
            if not object_name or not object_name.strip("/"):
                return {"success": False, "error": "object_name unavailable", "message": "object_name unavailable"}

            raw = self._protocol.get_device_file(object_name, 0)
            if raw is None:
                return {
                    "success": False,
                    "error": "get_device_file returned None",
                    "message": "get_device_file returned None",
                }

            result: dict[str, Any] = {
                "success": True,
                "object_name": object_name,
                "raw_bytes": len(raw),
            }

            if self._map_manager is not None:
                try:
                    map_data = self._map_manager.decode_map(raw)
                    if map_data:
                        image = self._map_manager.render_map(map_data)
                        if image:
                            import io

                            buf = io.BytesIO()
                            image.save(buf, format="PNG")
                            result["image"] = base64.b64encode(buf.getvalue()).decode()
                        result["map_data"] = {
                            "rooms": len(getattr(map_data, "segments", {}) or {}),
                            "robot_position": _point_to_dict(getattr(map_data, "robot_position", None)),
                            "charger_position": _point_to_dict(getattr(map_data, "charger_position", None)),
                        }
                except Exception as e:
                    logger.warning("Map decode/render failed", error=str(e))
                    result["render_error"] = str(e)

            result["raw_b64"] = base64.b64encode(raw).decode()
            return result
        except Exception as e:
            logger.exception("get_map failed")
            return {"success": False, "error": str(e), "message": str(e)}


def _point_to_dict(point: Any) -> dict[str, Any] | None:
    if point is None:
        return None
    return {"x": getattr(point, "x", None), "y": getattr(point, "y", None)}


def client_from_env() -> DreameHomeClient | None:
    """Build DreameHomeClient from DREAME_USER, DREAME_PASSWORD, etc."""
    user = os.environ.get("DREAME_USER", "").strip()
    pwd = os.environ.get("DREAME_PASSWORD", "").strip()
    if not user or not pwd:
        return None

    ref_raw = os.environ.get("DREAME_REF_PATH", "").strip()
    ref_path = Path(ref_raw) if ref_raw else _REF_DEFAULT

    return DreameHomeClient(
        username=user,
        password=pwd,
        country=os.environ.get("DREAME_COUNTRY", "eu").strip(),
        did=os.environ.get("DREAME_DID", "").strip() or None,
        auth_key=os.environ.get("DREAME_AUTH_KEY", "").strip() or None,
        ref_path=ref_path,
    )
