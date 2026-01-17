"""Configuration management for robotics-mcp."""

import yaml
from pathlib import Path
from typing import Any, Dict, Optional

import structlog

logger = structlog.get_logger(__name__)


class ConfigLoader:
    """Load and manage robotics-mcp configuration."""

    def __init__(self, config_path: Optional[Path] = None):
        """Initialize config loader.

        Args:
            config_path: Path to config YAML file. If None, uses default location.
        """
        if config_path is None:
            config_path = Path.home() / ".robotics-mcp" / "config.yaml"
        self.config_path = Path(config_path)
        self.config: Dict[str, Any] = {}

    def load(self) -> Dict[str, Any]:
        """Load configuration from YAML file.

        Returns:
            Configuration dictionary.

        Raises:
            FileNotFoundError: If config file doesn't exist.
            yaml.YAMLError: If config file is invalid.
        """
        if not self.config_path.exists():
            logger.warning("Config file not found, using defaults", path=str(self.config_path))
            return self._default_config()

        try:
            with open(self.config_path, "r") as f:
                self.config = yaml.safe_load(f) or {}
            logger.info("Config loaded", path=str(self.config_path))
            return self.config
        except yaml.YAMLError as e:
            logger.error("Failed to parse config file", error=str(e), path=str(self.config_path))
            raise

    def _default_config(self) -> Dict[str, Any]:
        """Return default configuration.

        Returns:
            Default configuration dictionary.
        """
        return {
            "robotics": {
                "moorebot_scout": {
                    "enabled": False,
                    "robot_id": "scout_01",
                    "ip_address": "192.168.1.100",
                    "port": 9090,
                    "mock_mode": True,
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

    def save(self, config: Optional[Dict[str, Any]] = None) -> None:
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

