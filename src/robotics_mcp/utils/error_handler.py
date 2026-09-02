"""Error handling utilities for robotics-mcp server.

Provides consistent error handling patterns and error response formatting
following SOTA MCP server standards.
"""

from typing import Any

import structlog

logger = structlog.get_logger(__name__)


def format_error_response(
    message: str,
    error_type: str = "error",
    details: dict[str, Any] | None = None,
    robot_id: str | None = None,
    action: str | None = None,
    **extra: Any,
) -> dict[str, Any]:
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
    response: dict[str, Any] = {
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

    response.update(extra)
    return response


def format_unavailable_error(
    backend: str,
    operation: str,
    *,
    robot_id: str | None = None,
    action: str | None = None,
    platform: str | None = None,
    details: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """Return a clear error when a required backend is not available."""
    extra: dict[str, Any] = {}
    if platform:
        extra["platform"] = platform
    if details:
        extra.update(details)
    return format_error_response(
        f"{backend} not available - cannot {operation}",
        error_type="not_available",
        robot_id=robot_id,
        action=action,
        details=extra or None,
    )


def format_not_implemented_error(
    feature: str,
    *,
    robot_id: str | None = None,
    action: str | None = None,
    details: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """Return a clear error for unimplemented functionality."""
    return format_error_response(
        f"{feature} is not implemented",
        error_type="not_implemented",
        robot_id=robot_id,
        action=action,
        details=details,
    )


def format_success_response(
    message: str,
    data: dict[str, Any] | None = None,
    robot_id: str | None = None,
    action: str | None = None,
    **extra: Any,
) -> dict[str, Any]:
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
    response: dict[str, Any] = {
        "status": "success",
        "message": message,
    }

    if robot_id:
        response["robot_id"] = robot_id
    if action:
        response["action"] = action
    if data:
        response.update(data)

    response.update(extra)
    return response


def handle_tool_error(
    operation: str,
    error: Exception,
    robot_id: str | None = None,
    action: str | None = None,
    context: dict[str, Any] | None = None,
) -> dict[str, Any]:
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
