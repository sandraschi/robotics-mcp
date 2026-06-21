"""Unit tests for spz_converter tool."""

import pytest

from robotics_mcp.tools.spz_converter import SPZConverterTool


@pytest.fixture
def spz_converter_tool(mock_mcp):
    tool = SPZConverterTool(mock_mcp)
    tool.register()
    return tool


@pytest.mark.asyncio
async def test_spz_converter_check_support(spz_converter_tool):
    tool_func = spz_converter_tool.mcp.tool.registered_func

    result = await tool_func(operation="check_spz_support")

    assert result["status"] == "success"
    assert "tools_available" in result


@pytest.mark.asyncio
async def test_spz_converter_extract_info(spz_converter_tool, tmp_path):
    spz_file = tmp_path / "test.spz"
    import zipfile

    with zipfile.ZipFile(spz_file, "w") as zf:
        zf.writestr("metadata.json", '{"version": "1.0"}')

    tool_func = spz_converter_tool.mcp.tool.registered_func
    result = await tool_func(operation="extract_spz_info", spz_path=str(spz_file))

    assert result is not None


@pytest.mark.asyncio
async def test_spz_converter_install_plugin(spz_converter_tool, tmp_path):
    project_path = tmp_path / "unity_project"
    manifest_path = project_path / "Packages" / "manifest.json"
    manifest_path.parent.mkdir(parents=True)

    import json

    with open(manifest_path, "w") as f:
        json.dump({"dependencies": {}}, f)

    tool_func = spz_converter_tool.mcp.tool.registered_func
    result = await tool_func(
        operation="install_unity_spz_plugin",
        unity_project_path=str(project_path),
    )

    assert result["status"] == "success"
    with open(manifest_path) as f:
        manifest = json.load(f)
        assert "com.aras-p.gaussian-splatting" in manifest.get("dependencies", {})


@pytest.mark.asyncio
async def test_spz_converter_invalid_operation(spz_converter_tool):
    tool_func = spz_converter_tool.mcp.tool.registered_func
    result = await tool_func(operation="invalid_op")

    assert result["status"] == "error"
    assert "unknown operation" in result.get("message", result.get("error", "")).lower()


@pytest.mark.asyncio
async def test_spz_converter_missing_path(spz_converter_tool):
    tool_func = spz_converter_tool.mcp.tool.registered_func
    result = await tool_func(operation="extract_spz_info")

    assert result["status"] == "error"
    assert "required" in result.get("message", result.get("error", "")).lower()
