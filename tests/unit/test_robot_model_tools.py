"""Unit tests for robot_model_tools."""

import pytest
from unittest.mock import AsyncMock, MagicMock, patch
from pathlib import Path

from robotics_mcp.tools.robot_model_tools import RobotModelTools, ROBOT_MODEL_FORMATS
from robotics_mcp.utils.state_manager import RobotStateManager


@pytest.fixture
def mock_mcp():
    """Create mock MCP server."""
    mcp = MagicMock()
    mcp.tool = MagicMock(return_value=lambda f: f)
    return mcp


@pytest.fixture
def state_manager():
    """Create robot state manager."""
    return RobotStateManager()


@pytest.fixture
def mock_mounted_servers():
    """Create mock mounted servers."""
    return {
        "blender": MagicMock(),
        "gimp": MagicMock(),
    }


@pytest.fixture
def robot_model_tools(mock_mcp, state_manager, mock_mounted_servers):
    """Create robot model tools instance."""
    tool = RobotModelTools(mock_mcp, state_manager, mock_mounted_servers)
    tool.register()
    return tool


@pytest.mark.asyncio
async def test_robot_model_create_scout(robot_model_tools, tmp_path):
    """Test creating Scout model."""
    tool_func = robot_model_tools.mcp.tool.call_args[0][0]
    output_path = str(tmp_path / "scout_model.fbx")
    
    with patch("robotics_mcp.tools.robot_model_tools.get_blender_executor") as mock_executor:
        mock_exec = MagicMock()
        mock_executor.return_value = mock_exec
        mock_exec.execute_script = AsyncMock()
        
        with patch("robotics_mcp.tools.robot_model_tools.Client") as mock_client:
            mock_client_instance = AsyncMock()
            mock_client.return_value.__aenter__.return_value = mock_client_instance
            mock_client_instance.call_tool = AsyncMock(return_value={"success": True})
            
            result = await tool_func(
                operation="create",
                robot_type="scout",
                output_path=output_path,
                format="fbx",
            )
            
            assert result["success"] is True
            assert "output_path" in result["data"]


@pytest.mark.asyncio
async def test_robot_model_create_invalid_type(robot_model_tools):
    """Test creating model with invalid robot type."""
    tool_func = robot_model_tools.mcp.tool.call_args[0][0]
    
    result = await tool_func(
        operation="create",
        robot_type="invalid_type",
        output_path="/tmp/test.fbx",
    )
    
    assert result["success"] is False
    assert "not supported" in result["error"].lower() or "invalid" in result["error"].lower()


@pytest.mark.asyncio
async def test_robot_model_import(robot_model_tools, tmp_path):
    """Test importing model."""
    model_path = tmp_path / "scout.fbx"
    model_path.touch()  # Create empty file
    
    tool_func = robot_model_tools.mcp.tool.call_args[0][0]
    
    with patch("robotics_mcp.tools.robot_model_tools.Client") as mock_client:
        mock_client_instance = AsyncMock()
        mock_client.return_value.__aenter__.return_value = mock_client_instance
        mock_client_instance.call_tool = AsyncMock(return_value={"success": True, "imported_path": str(model_path)})
        
        result = await tool_func(
            operation="import",
            robot_type="scout",
            model_path=str(model_path),
            platform="unity",
        )
        
        assert result["success"] is True


@pytest.mark.asyncio
async def test_robot_model_import_missing_file(robot_model_tools):
    """Test importing non-existent model file."""
    tool_func = robot_model_tools.mcp.tool.call_args[0][0]
    
    result = await tool_func(
        operation="import",
        robot_type="scout",
        model_path="/nonexistent/file.fbx",
    )
    
    assert result["success"] is False
    assert "not found" in result["error"].lower()


@pytest.mark.asyncio
async def test_robot_model_export(robot_model_tools, state_manager, tmp_path):
    """Test exporting model."""
    # Register a robot first
    state_manager.register_robot("vbot_scout_01", "scout", platform="unity")
    
    tool_func = robot_model_tools.mcp.tool.call_args[0][0]
    output_path = str(tmp_path / "exported.fbx")
    
    with patch("robotics_mcp.tools.robot_model_tools.Client") as mock_client:
        mock_client_instance = AsyncMock()
        mock_client.return_value.__aenter__.return_value = mock_client_instance
        mock_client_instance.call_tool = AsyncMock(return_value={"success": True, "exported_path": output_path})
        
        result = await tool_func(
            operation="export",
            robot_id="vbot_scout_01",
            output_path=output_path,
            format="fbx",
        )
        
        assert result["success"] is True


@pytest.mark.asyncio
async def test_robot_model_convert(robot_model_tools, tmp_path):
    """Test converting model format."""
    source_path = tmp_path / "scout.fbx"
    source_path.touch()
    target_path = tmp_path / "scout.glb"
    
    tool_func = robot_model_tools.mcp.tool.call_args[0][0]
    
    with patch("robotics_mcp.tools.robot_model_tools.Client") as mock_client:
        mock_client_instance = AsyncMock()
        mock_client.return_value.__aenter__.return_value = mock_client_instance
        mock_client_instance.call_tool = AsyncMock(return_value={"success": True, "converted_path": str(target_path)})
        
        result = await tool_func(
            operation="convert",
            source_path=str(source_path),
            source_format="fbx",
            target_format="glb",
            target_path=str(target_path),
        )
        
        assert result["success"] is True


@pytest.mark.asyncio
async def test_robot_model_invalid_operation(robot_model_tools):
    """Test invalid operation."""
    tool_func = robot_model_tools.mcp.tool.call_args[0][0]
    
    result = await tool_func(
        operation="invalid_op",
    )
    
    assert result["success"] is False
    assert "unknown operation" in result["error"].lower()


@pytest.mark.asyncio
async def test_robot_model_all_formats(robot_model_tools, tmp_path):
    """Test all supported formats."""
    tool_func = robot_model_tools.mcp.tool.call_args[0][0]
    
    with patch("robotics_mcp.tools.robot_model_tools.get_blender_executor") as mock_executor:
        mock_exec = MagicMock()
        mock_executor.return_value = mock_exec
        mock_exec.execute_script = AsyncMock()
        
        with patch("robotics_mcp.tools.robot_model_tools.Client") as mock_client:
            mock_client_instance = AsyncMock()
            mock_client.return_value.__aenter__.return_value = mock_client_instance
            mock_client_instance.call_tool = AsyncMock(return_value={"success": True})
            
            for fmt in ROBOT_MODEL_FORMATS:
                output_path = tmp_path / f"test.{fmt}"
                result = await tool_func(
                    operation="create",
                    robot_type="scout",
                    output_path=str(output_path),
                    format=fmt,
                )
                # Should handle format (may fail for some, but should not crash)
                assert result is not None

