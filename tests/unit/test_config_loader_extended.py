"""Extended unit tests for config loader."""

import pytest
import yaml
from pathlib import Path
import tempfile

from robotics_mcp.utils.config_loader import ConfigLoader


def test_config_loader_default_path():
    """Test config loader uses default path when None provided."""
    loader = ConfigLoader()
    assert loader.config_path == Path.home() / ".robotics-mcp" / "config.yaml"


def test_config_loader_custom_path():
    """Test config loader with custom path."""
    with tempfile.TemporaryDirectory() as tmpdir:
        custom_path = Path(tmpdir) / "custom_config.yaml"
        loader = ConfigLoader(custom_path)
        assert loader.config_path == custom_path


def test_config_loader_missing_file_returns_defaults():
    """Test that missing config file returns defaults."""
    with tempfile.TemporaryDirectory() as tmpdir:
        config_path = Path(tmpdir) / "nonexistent.yaml"
        loader = ConfigLoader(config_path)
        config = loader.load()
        
        assert "robotics" in config
        assert "server" in config
        assert config["robotics"]["moorebot_scout"]["mock_mode"] is True


def test_config_loader_invalid_yaml():
    """Test handling of invalid YAML."""
    with tempfile.TemporaryDirectory() as tmpdir:
        config_path = Path(tmpdir) / "invalid.yaml"
        config_path.write_text("invalid: yaml: content: [")
        
        loader = ConfigLoader(config_path)
        with pytest.raises(yaml.YAMLError):
            loader.load()


def test_config_loader_empty_file():
    """Test handling of empty YAML file."""
    with tempfile.TemporaryDirectory() as tmpdir:
        config_path = Path(tmpdir) / "empty.yaml"
        config_path.write_text("")
        
        loader = ConfigLoader(config_path)
        config = loader.load()
        
        # Should return defaults when file is empty
        assert "robotics" in config


def test_config_loader_save_creates_directory():
    """Test that save creates parent directory if needed."""
    with tempfile.TemporaryDirectory() as tmpdir:
        config_path = Path(tmpdir) / "nested" / "config.yaml"
        loader = ConfigLoader(config_path)
        
        test_config = {"test": "value"}
        loader.save(test_config)
        
        assert config_path.exists()
        assert config_path.parent.exists()


def test_config_loader_save_and_reload():
    """Test saving and reloading configuration."""
    with tempfile.TemporaryDirectory() as tmpdir:
        config_path = Path(tmpdir) / "config.yaml"
        loader = ConfigLoader(config_path)
        
        test_config = {
            "robotics": {
                "moorebot_scout": {
                    "enabled": True,
                    "robot_id": "test_scout",
                    "port": 9090
                }
            }
        }
        
        loader.save(test_config)
        
        # Reload
        loader2 = ConfigLoader(config_path)
        loaded = loader2.load()
        
        assert loaded["robotics"]["moorebot_scout"]["enabled"] is True
        assert loaded["robotics"]["moorebot_scout"]["robot_id"] == "test_scout"


def test_config_loader_default_config_structure():
    """Test default config has all required sections."""
    loader = ConfigLoader()
    config = loader.load()
    
    # Check robotics section
    assert "robotics" in config
    assert "moorebot_scout" in config["robotics"]
    assert "virtual" in config["robotics"]
    assert "mcp_integration" in config["robotics"]
    
    # Check server section
    assert "server" in config
    assert "enable_http" in config["server"]
    assert "http_port" in config["server"]
    assert "log_level" in config["server"]


def test_config_loader_partial_config_merges():
    """Test that partial config merges with defaults."""
    with tempfile.TemporaryDirectory() as tmpdir:
        config_path = Path(tmpdir) / "partial.yaml"
        partial_config = {
            "robotics": {
                "moorebot_scout": {
                    "enabled": True
                }
            }
        }
        
        with open(config_path, "w") as f:
            yaml.dump(partial_config, f)
        
        loader = ConfigLoader(config_path)
        config = loader.load()
        
        # Should have enabled=True from file
        assert config["robotics"]["moorebot_scout"]["enabled"] is True
        # Should have defaults for other fields
        assert "mock_mode" in config["robotics"]["moorebot_scout"]

