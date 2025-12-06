"""Unit tests for config loader."""

import pytest
from pathlib import Path
import tempfile
import yaml

from robotics_mcp.utils.config_loader import ConfigLoader


def test_default_config():
    """Test default configuration."""
    loader = ConfigLoader()
    config = loader.load()
    
    assert "robotics" in config
    assert "server" in config
    assert config["robotics"]["moorebot_scout"]["mock_mode"] is True


def test_load_config_file(tmp_path: Path):
    """Test loading configuration from file."""
    config_file = tmp_path / "config.yaml"
    test_config = {
        "robotics": {
            "moorebot_scout": {
                "enabled": True,
                "robot_id": "test_scout",
            }
        }
    }
    
    with open(config_file, "w") as f:
        yaml.dump(test_config, f)
    
    loader = ConfigLoader(config_file)
    config = loader.load()
    
    assert config["robotics"]["moorebot_scout"]["robot_id"] == "test_scout"
    assert config["robotics"]["moorebot_scout"]["enabled"] is True


def test_save_config(tmp_path: Path):
    """Test saving configuration."""
    config_file = tmp_path / "config.yaml"
    loader = ConfigLoader(config_file)
    
    test_config = {
        "robotics": {
            "moorebot_scout": {
                "enabled": True,
            }
        }
    }
    
    loader.save(test_config)
    
    assert config_file.exists()
    loaded = loader.load()
    assert loaded["robotics"]["moorebot_scout"]["enabled"] is True

