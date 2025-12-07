"""Error handling utilities for robotics-mcp server.

Provides consistent error handling patterns and error response formatting
following SOTA MCP server standards.
"""

from typing import Any, Dict, Optional

import structlog

logger = structlog.get_logger(__name__)


def format_error_response(
    message: str,
    error_type: str = "error",
    details: Optional[Dict[str, Any]] = None,
    robot_id: Optional[str] = None,
    action: Optional[str] = None,
) -> Dict[str, Any]:
    """Format a consistent error response.

    Args:
        message: Human-readable error message.
        error_type: Error type (e.g., "error", "validation_error", "not_found").
        details: Additional error details.
        robot_id: Robot identifier if applicable.
        action: Action that failed if applicable.

    Returns:
        Formatted error response dictionary.

    Examples:
        Basic error:
            format_error_response("Robot not found")
            # Returns: {"status": "error", "message": "Robot not found"}

        Detailed error:
            format_error_response(
                "Failed to connect to robot",
                error_type="connection_error",
                details={"host": "192.168.1.100", "port": 9090},
                robot_id="scout_01"
            )
    """
    response: Dict[str, Any] = {
        "status": "error",
        "error_type": error_type,
        "message": message,
    }

    if robot_id:
        response["robot_id"] = robot_id
    if action:
        response["action"] = action
    if details:
        response["details"] = details

    return response


def format_success_response(
    message: str,
    data: Optional[Dict[str, Any]] = None,
    robot_id: Optional[str] = None,
    action: Optional[str] = None,
) -> Dict[str, Any]:
    """Format a consistent success response.

    Args:
        message: Success message.
        data: Additional response data.
        robot_id: Robot identifier if applicable.
        action: Action that succeeded if applicable.

    Returns:
        Formatted success response dictionary.

    Examples:
        Basic success:
            format_success_response("Robot moved successfully")
            # Returns: {"status": "success", "message": "Robot moved successfully"}

        Detailed success:
            format_success_response(
                "Robot moved",
                data={"position": {"x": 1.0, "y": 2.0}},
                robot_id="scout_01",
                action="move"
            )
    """
    response: Dict[str, Any] = {
        "status": "success",
        "message": message,
    }

    if robot_id:
        response["robot_id"] = robot_id
    if action:
        response["action"] = action
    if data:
        response.update(data)

    return response


def handle_tool_error(
    operation: str,
    error: Exception,
    robot_id: Optional[str] = None,
    action: Optional[str] = None,
    context: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    """Handle tool errors with consistent logging and response formatting.

    Args:
        operation: Name of the operation that failed.
        error: Exception that was raised.
        robot_id: Robot identifier if applicable.
        action: Action that failed if applicable.
        context: Additional context for logging.

    Returns:
        Formatted error response dictionary.

    Examples:
        Handle error in robot control:
            try:
                result = await move_robot(robot_id, linear=0.2)
            except Exception as e:
                return handle_tool_error("move_robot", e, robot_id="scout_01", action="move")
    """
    error_msg = str(error)
    error_type = type(error).__name__

    # Log error with context
    log_context = {
        "operation": operation,
        "error_type": error_type,
        "error": error_msg,
    }
    if robot_id:
        log_context["robot_id"] = robot_id
    if action:
        log_context["action"] = action
    if context:
        log_context.update(context)

    logger.error("Tool operation failed", **log_context, exc_info=True)

    # Determine error type for response
    if isinstance(error, ValueError):
        response_error_type = "validation_error"
    elif isinstance(error, KeyError):
        response_error_type = "not_found"
    elif isinstance(error, ConnectionError):
        response_error_type = "connection_error"
    elif isinstance(error, TimeoutError):
        response_error_type = "timeout_error"
    else:
        response_error_type = "error"

    return format_error_response(
        message=f"{operation} failed: {error_msg}",
        error_type=response_error_type,
        details={"error_type": error_type},
        robot_id=robot_id,
        action=action,
    )

