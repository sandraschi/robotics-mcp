"""
FastMCP 2.14.1+ Conversational Response Builders for Robotics MCP.

Provides intelligent error recovery and conversational response patterns
for robotics operations, following the same patterns as tapo-camera-mcp.
"""

from typing import Any, Dict, List, Optional


def build_success_response(
    operation: str,
    summary: str,
    result: Optional[Dict[str, Any]] = None,
    recommendations: Optional[List[str]] = None,
    next_steps: Optional[List[str]] = None,
    **kwargs
) -> Dict[str, Any]:
    """Build structured success response for MCP clients.

    Args:
        operation: The operation that succeeded
        summary: Human-readable summary of the result
        result: The actual operation result data
        recommendations: Suggested next actions or optimizations
        next_steps: Logical next steps for the user
        **kwargs: Additional response fields

    Returns:
        Structured success response dictionary
    """
    response = {
        "success": True,
        "operation": operation,
        "summary": summary,
    }

    if result:
        response["result"] = result
    if recommendations:
        response["recommendations"] = recommendations
    if next_steps:
        response["next_steps"] = next_steps

    response.update(kwargs)
    return response


def build_error_response(
    error: str,
    error_code: str,
    message: str,
    recovery_options: Optional[List[str]] = None,
    suggestions: Optional[List[str]] = None,
    **kwargs
) -> Dict[str, Any]:
    """Build structured error response with recovery guidance for MCP clients.

    Args:
        error: Short error description
        error_code: Machine-readable error code
        message: Detailed error message
        recovery_options: Steps to recover from the error
        suggestions: Alternative approaches or preventive measures
        **kwargs: Additional response fields

    Returns:
        Structured error response dictionary
    """
    response = {
        "success": False,
        "error": error,
        "error_code": error_code,
        "message": message,
    }

    if recovery_options:
        response["recovery_options"] = recovery_options
    if suggestions:
        response["suggestions"] = suggestions

    response.update(kwargs)
    return response


def build_hardware_error_response(
    error: str,
    device_type: str,
    device_id: str,
    recovery_options: Optional[List[str]] = None,
    **kwargs
) -> Dict[str, Any]:
    """Build hardware-specific error response with device recovery guidance.

    Args:
        error: The hardware error description
        device_type: Type of hardware device (Robot, Camera, etc.)
        device_id: Specific device identifier
        recovery_options: Hardware-specific recovery steps
        **kwargs: Additional response fields

    Returns:
        Structured hardware error response
    """
    if not recovery_options:
        recovery_options = [
            f"Check if {device_type} '{device_id}' is powered on and connected to network",
            f"Verify {device_type} '{device_id}' is not blocked by firewall or network restrictions",
            f"Try power cycling {device_type} '{device_id}'",
            f"Check {device_type} '{device_id}' firmware is up to date",
        ]

    # Extract suggestions from kwargs if provided, otherwise use defaults
    suggestions = kwargs.pop('suggestions', [
        f"Try the operation again after applying recovery steps for {device_type} '{device_id}'",
        f"Check {device_type} status and connectivity before retrying",
        f"Verify {device_type} configuration and network settings"
    ])

    return build_error_response(
        error=f"{device_type} hardware error",
        error_code="HARDWARE_ERROR",
        message=error,
        device_type=device_type,
        device_id=device_id,
        recovery_options=recovery_options,
        suggestions=suggestions,
        **kwargs
    )


def build_network_error_response(
    error: str,
    service: Optional[str] = None,
    port: Optional[int] = None,
    recovery_options: Optional[List[str]] = None,
    **kwargs
) -> Dict[str, Any]:
    """Build network-specific error response with connectivity recovery guidance.

    Args:
        error: The network error description
        service: Service name if applicable
        port: Port number if applicable
        recovery_options: Network-specific recovery steps
        **kwargs: Additional response fields

    Returns:
        Structured network error response
    """
    if not recovery_options:
        recovery_options = []
        if service:
            recovery_options.append(f"Verify {service} service is running and accessible")
        if port:
            recovery_options.extend([
                f"Check if port {port} is available (might be used by previous server instance)",
                f"Kill any process using port {port}: 'netstat -ano | findstr :{port}' then 'taskkill /PID <pid>'"
            ])
        else:
            recovery_options.append("Check network connectivity and firewall settings")
        recovery_options.append("Verify device is on the same network subnet")
        recovery_options.append("Try restarting the MCP server")

    return build_error_response(
        error="Network connectivity error",
        error_code="NETWORK_ERROR",
        message=error,
        service=service,
        port=port,
        recovery_options=recovery_options,
        suggestions=[
            "Try the operation again after applying network recovery steps",
            "Check network connectivity and service status before retrying",
            "Verify firewall and network configuration settings"
        ],
        **kwargs
    )


def build_configuration_error_response(
    error: str,
    config_field: str,
    recovery_options: Optional[List[str]] = None,
    **kwargs
) -> Dict[str, Any]:
    """Build configuration-specific error response.

    Args:
        error: The configuration error description
        config_field: Which configuration field is problematic
        recovery_options: Configuration-specific recovery steps
        **kwargs: Additional response fields

    Returns:
        Structured configuration error response
    """
    if not recovery_options:
        recovery_options = [
            f"Check {config_field} configuration in config.yaml",
            f"Verify {config_field} format and values are correct",
            "Reload configuration: restart MCP server or call configuration refresh",
            f"Check logs for detailed {config_field} validation errors"
        ]

    return build_error_response(
        error="Configuration error",
        error_code="CONFIG_ERROR",
        message=error,
        config_field=config_field,
        recovery_options=recovery_options,
        suggestions=[
            f"Verify {config_field} configuration settings",
            "Check configuration file format and syntax",
            "Restart the MCP server after configuration changes"
        ],
        **kwargs
    )


def build_robotics_error_response(
    error: str,
    robot_type: str,
    robot_id: str,
    recovery_options: Optional[List[str]] = None,
    **kwargs
) -> Dict[str, Any]:
    """Build robotics-specific error response with robot recovery guidance.

    Args:
        error: The robotics error description
        robot_type: Type of robot (scout, go2, g1, etc.)
        robot_id: Specific robot identifier
        recovery_options: Robot-specific recovery steps
        **kwargs: Additional response fields

    Returns:
        Structured robotics error response
    """
    if not recovery_options:
        recovery_options = [
            f"Check if {robot_type} robot '{robot_id}' is powered on and connected to network",
            f"Verify {robot_type} robot '{robot_id}' is not blocked by firewall or network restrictions",
            f"Try power cycling {robot_type} robot '{robot_id}'",
            f"Check {robot_type} robot '{robot_id}' battery level - charge if low",
            f"Verify {robot_type} robot '{robot_id}' firmware is up to date",
            f"Ensure {robot_type} robot '{robot_id}' is not in use by another application"
        ]

        # Add ROS-specific recovery for physical robots
        if robot_type.lower() in ["scout", "go2", "g1"]:
            recovery_options.extend([
                "Check ROS master is running: 'roscore'",
                "Verify ROS_MASTER_URI environment variable",
                f"Ensure {robot_type} robot and MCP server are on same ROS network",
                f"Check ROS node registration and topics for {robot_type} robot"
            ])

    return build_hardware_error_response(
        error=f"Robotics operation failed: {error}",
        device_type="Robot",
        device_id=robot_id,
        recovery_options=recovery_options,
        robot_type=robot_type,
        suggestions=[
            f"Try the robotics operation again after applying recovery steps for {robot_type} '{robot_id}'",
            f"Check {robot_type} robot status with 'robotics_system' status action",
            f"Verify {robot_type} robot network connectivity and configuration"
        ],
        **kwargs
    )