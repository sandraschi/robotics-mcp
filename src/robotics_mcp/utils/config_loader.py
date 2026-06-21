"""Configuration management for robotics-mcp."""

from pathlib import Path
from typing import Any

import structlog
import yaml

logger = structlog.get_logger(__name__)


class ConfigLoader:
    """Load and manage robotics-mcp configuration."""

    def __init__(self, config_path: Path | None = None):
        """Initialize config loader.

        Args:
            config_path: Path to config YAML file. If None, uses default location.
        """
        if config_path is None:
            config_path = Path.home() / ".robotics-mcp" / "config.yaml"
        self.config_path = Path(config_path)
        self.config: dict[str, Any] = {}

    def load(self) -> dict[str, Any]:
        """Load configuration from YAML file.

        Returns:
            Configuration dictionary.

        Raises:
            FileNotFoundError: If config file doesn't exist.
            yaml.YAMLError: If config file is invalid.
        """
        if not self.config_path.exists():
            logger.warning("Config file not found, using defaults", path=str(self.config_path))
            self.config = self._default_config()
            return self.config

        try:
            with open(self.config_path) as f:
                loaded = yaml.safe_load(f) or {}
            if not loaded:
                self.config = self._default_config()
            else:
                self.config = self._deep_merge(self._default_config(), loaded)
            logger.info("Config loaded", path=str(self.config_path))
            return self.config
        except yaml.YAMLError as e:
            logger.error("Failed to parse config file", error=str(e), path=str(self.config_path))
            raise

    @staticmethod
    def _deep_merge(base: dict[str, Any], override: dict[str, Any]) -> dict[str, Any]:
        merged = dict(base)
        for key, value in override.items():
            if isinstance(value, dict) and isinstance(merged.get(key), dict):
                merged[key] = ConfigLoader._deep_merge(merged[key], value)
            else:
                merged[key] = value
        return merged

    def _default_config(self) -> dict[str, Any]:
        """Return default configuration.

        Returns:
            Default configuration dictionary.
        """
        return {
            "robotics": {
                "yahboom_raspbot_v2": {
                    "enabled": False,
                    "robot_id": "yahboom_01",
                    "ip_address": "192.168.0.250",
                    "port": 9090,
                    "mock_mode": False,
                    "camera_enabled": True,
                    "navigation_enabled": True,
                    "arm_enabled": False,
                },
                "moorebot_scout": {
                    "enabled": False,
                    "robot_id": "scout_01",
                    "ip_address": "192.168.1.100",
                    "port": 9090,
                    "mock_mode": False,
                    "lidar": {
                        "enabled": False,
                        "type": "ydlidar_superlight",
                        "ros_topic": "/scan",
                    },
                },
                "virtual": {
                    "enabled": True,
                    "platform": "unity",
                    "unity": {"host": "localhost", "port": 8080},
                    "vrchat": {"enabled": False, "osc_port": 9000},
                },
                "mcp_integration": {
                    "osc_mcp": {"enabled": True, "prefix": "osc"},
                    "unity3d_mcp": {"enabled": True, "prefix": "unity"},
                    "vrchat_mcp": {"enabled": True, "prefix": "vrchat"},
                    "avatar_mcp": {"enabled": True, "prefix": "avatar"},
                },
            },
            "server": {
                "enable_http": True,
                "http_port": 12230,
                "log_level": "INFO",
            },
        }

    def save(self, config: dict[str, Any] | None = None) -> None:
        """Save configuration to YAML file.

        Args:
            config: Configuration to save. If None, saves current config.
        """
        if config is not None:
            self.config = config

        self.config_path.parent.mkdir(parents=True, exist_ok=True)

        with open(self.config_path, "w") as f:
            yaml.dump(self.config, f, default_flow_style=False, sort_keys=False)

        logger.info("Config saved", path=str(self.config_path))
