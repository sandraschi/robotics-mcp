"""Unit tests for spz_converter tool."""

import pytest
from unittest.mock import MagicMock, patch
from pathlib import Path

from robotics_mcp.tools.spz_converter import SPZConverterTool


@pytest.fixture
def mock_mcp():
    """Create mock MCP server."""
    mcp = MagicMock()
    mcp.tool = MagicMock(return_value=lambda f: f)
    return mcp


@pytest.fixture
def spz_converter_tool(mock_mcp):
    """Create SPZ converter tool instance."""
    tool = SPZConverterTool(mock_mcp)
    tool.register()
    return tool


@pytest.mark.asyncio
async def test_spz_converter_check_support(spz_converter_tool):
    """Test checking SPZ support."""
    tool_func = spz_converter_tool.mcp.tool.call_args[0][0]
    
    result = await tool_func(
        operation="check_spz_support",
    )
    
    assert result["success"] is True
    assert "tools_available" in result["data"]


@pytest.mark.asyncio
async def test_spz_converter_extract_info(spz_converter_tool, tmp_path):
    """Test extracting SPZ info."""
    # Create a fake .spz file (actually a zip)
    spz_file = tmp_path / "test.spz"
    import zipfile
    with zipfile.ZipFile(spz_file, "w") as zf:
        zf.writestr("metadata.json", '{"version": "1.0"}')
    
    tool_func = spz_converter_tool.mcp.tool.call_args[0][0]
    
    result = await tool_func(
        operation="extract_spz_info",
        spz_path=str(spz_file),
    )
    
    # Should handle the file (even if not a real .spz)
    assert result is not None


@pytest.mark.asyncio
async def test_spz_converter_install_plugin(spz_converter_tool, tmp_path):
    """Test installing Unity plugin."""
    # Create a fake Unity project
    project_path = tmp_path / "unity_project"
    manifest_path = project_path / "Packages" / "manifest.json"
    manifest_path.parent.mkdir(parents=True)
    
    import json
    with open(manifest_path, "w") as f:
        json.dump({"dependencies": {}}, f)
    
    tool_func = spz_converter_tool.mcp.tool.call_args[0][0]
    
    result = await tool_func(
        operation="install_unity_spz_plugin",
        unity_project_path=str(project_path),
    )
    
    assert result["success"] is True
    # Check that package was added to manifest
    with open(manifest_path) as f:
        manifest = json.load(f)
        assert "com.aras-p.gaussian-splatting" in manifest.get("dependencies", {})


@pytest.mark.asyncio
async def test_spz_converter_invalid_operation(spz_converter_tool):
    """Test invalid operation."""
    tool_func = spz_converter_tool.mcp.tool.call_args[0][0]
    
    result = await tool_func(
        operation="invalid_op",
    )
    
    assert result["success"] is False
    assert "unknown operation" in result["error"].lower()


@pytest.mark.asyncio
async def test_spz_converter_missing_path(spz_converter_tool):
    """Test missing required path."""
    tool_func = spz_converter_tool.mcp.tool.call_args[0][0]
    
    result = await tool_func(
        operation="extract_spz_info",
        # spz_path not provided
    )
    
    assert result["success"] is False
    assert "required" in result["error"].lower()

