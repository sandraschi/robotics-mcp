"""Unit tests for error handler utilities."""

import pytest

from robotics_mcp.utils.error_handler import (
    format_error_response,
    format_success_response,
    handle_tool_error,
)


def test_format_error_response_basic():
    """Test basic error response formatting."""
    result = format_error_response("Robot not found")
    
    assert result["status"] == "error"
    assert result["error_type"] == "error"
    assert result["message"] == "Robot not found"
    assert "robot_id" not in result
    assert "action" not in result


def test_format_error_response_with_context():
    """Test error response with robot_id and action."""
    result = format_error_response(
        "Failed to move robot",
        error_type="connection_error",
        robot_id="scout_01",
        action="move",
        details={"host": "192.168.1.100", "port": 9090}
    )
    
    assert result["status"] == "error"
    assert result["error_type"] == "connection_error"
    assert result["message"] == "Failed to move robot"
    assert result["robot_id"] == "scout_01"
    assert result["action"] == "move"
    assert result["details"]["host"] == "192.168.1.100"
    assert result["details"]["port"] == 9090


def test_format_success_response_basic():
    """Test basic success response formatting."""
    result = format_success_response("Robot moved successfully")
    
    assert result["status"] == "success"
    assert result["message"] == "Robot moved successfully"
    assert "robot_id" not in result
    assert "action" not in result


def test_format_success_response_with_data():
    """Test success response with data."""
    result = format_success_response(
        "Robot moved",
        data={"position": {"x": 1.0, "y": 2.0}},
        robot_id="scout_01",
        action="move"
    )
    
    assert result["status"] == "success"
    assert result["message"] == "Robot moved"
    assert result["robot_id"] == "scout_01"
    assert result["action"] == "move"
    assert result["position"]["x"] == 1.0
    assert result["position"]["y"] == 2.0


def test_handle_tool_error_valueerror():
    """Test handling ValueError."""
    error = ValueError("Invalid robot_id")
    result = handle_tool_error("test_operation", error, robot_id="scout_01", action="move")
    
    assert result["status"] == "error"
    assert result["error_type"] == "validation_error"
    assert "test_operation failed" in result["message"]
    assert result["robot_id"] == "scout_01"
    assert result["action"] == "move"
    assert result["details"]["error_type"] == "ValueError"


def test_handle_tool_error_keyerror():
    """Test handling KeyError."""
    error = KeyError("robot_id")
    result = handle_tool_error("test_operation", error)
    
    assert result["status"] == "error"
    assert result["error_type"] == "not_found"


def test_handle_tool_error_connectionerror():
    """Test handling ConnectionError."""
    error = ConnectionError("Connection refused")
    result = handle_tool_error("test_operation", error, robot_id="scout_01")
    
    assert result["status"] == "error"
    assert result["error_type"] == "connection_error"
    assert result["robot_id"] == "scout_01"


def test_handle_tool_error_timeouterror():
    """Test handling TimeoutError."""
    error = TimeoutError("Operation timed out")
    result = handle_tool_error("test_operation", error)
    
    assert result["status"] == "error"
    assert result["error_type"] == "timeout_error"


def test_handle_tool_error_generic():
    """Test handling generic exception."""
    error = RuntimeError("Unexpected error")
    result = handle_tool_error("test_operation", error, context={"key": "value"})
    
    assert result["status"] == "error"
    assert result["error_type"] == "error"
    assert result["details"]["error_type"] == "RuntimeError"


def test_handle_tool_error_with_context():
    """Test error handling with additional context."""
    error = ValueError("Invalid parameter")
    result = handle_tool_error(
        "test_operation",
        error,
        robot_id="scout_01",
        action="move",
        context={"linear": 0.5, "angular": 0.0}
    )
    
    assert result["status"] == "error"
    assert result["robot_id"] == "scout_01"
    assert result["action"] == "move"

