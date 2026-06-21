"""Ensure Raspbot V2 config and optionally probe rosbridge.

Run: uv run python -m robotics_mcp.scripts.raspbot_setup --ensure-config [--ip IP] [--probe].
"""

import argparse
import socket
import sys

from robotics_mcp.utils.config_loader import ConfigLoader

DEFAULT_IP = "192.168.0.250"
DEFAULT_PORT = 9090


def ensure_config(ip: str, port: int) -> bool:
    """Create or update config so yahboom_raspbot_v2 is enabled with given ip/port. Returns True on success."""
    loader = ConfigLoader()
    config = loader.load()
    if "robotics" not in config:
        config["robotics"] = {}
    block = {
        "enabled": True,
        "robot_id": "yahboom_01",
        "ip_address": ip,
        "port": port,
        "mock_mode": False,
        "camera_enabled": True,
        "navigation_enabled": True,
        "arm_enabled": False,
    }
    config["robotics"]["yahboom_raspbot_v2"] = block
    loader.save(config)
    return True


def probe(ip: str, port: int, timeout: float = 3.0) -> bool:
    """Try TCP connect to ip:port. Returns True if connection succeeded."""
    try:
        with socket.create_connection((ip, port), timeout=timeout):
            return True
    except (TimeoutError, OSError):
        return False


def main() -> int:
    parser = argparse.ArgumentParser(description="Raspbot V2 config and connectivity")
    parser.add_argument("--ensure-config", action="store_true", help="Create/update config with yahboom_01 enabled")
    parser.add_argument("--ip", default=DEFAULT_IP, help=f"Robot IP (default {DEFAULT_IP})")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help=f"Rosbridge port (default {DEFAULT_PORT})")
    parser.add_argument("--probe", action="store_true", help="Test TCP connect to robot:port after config")
    args = parser.parse_args()

    if args.ensure_config:
        ensure_config(args.ip, args.port)
        print(f"Config updated: yahboom_raspbot_v2 enabled, ip={args.ip}, port={args.port}, mock_mode=false")

    if args.probe:
        if probe(args.ip, args.port):
            print(f"Probe OK: {args.ip}:{args.port} reachable")
        else:
            print(
                f"Probe FAIL: {args.ip}:{args.port} not reachable (connect to robot WiFi or start rosbridge on robot)"
            )
            return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())
