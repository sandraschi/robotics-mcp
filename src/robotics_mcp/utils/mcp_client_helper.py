"""Helper utilities for calling tools on loaded MCP servers without exposing them."""

from typing import Any, Dict, Optional

from fastmcp import Client


async def call_mounted_server_tool(
    mounted_servers: Dict[str, Any],
    server_name: str,
    tool_name: str,
    params: Optional[Dict[str, Any]] = None,
) -> Any:
    """Call a tool on a loaded MCP server without exposing it.
    
    These servers are loaded for internal use only - their tools are NOT exposed
    to avoid tool explosion. Only robotics-mcp's own portmanteau tools are exposed.
    
    Args:
        mounted_servers: Dictionary of loaded server instances.
        server_name: Name of the server (e.g., "unity", "blender").
        tool_name: Name of the tool to call (without prefix).
        params: Tool parameters (dict).
        
    Returns:
        Tool execution result.
    """
    if server_name not in mounted_servers:
        raise ValueError(f"Server {server_name} not found in loaded servers")
    
    server = mounted_servers[server_name]
    params = params or {}
    
    # Get the FastMCP instance from the server
    mcp_instance = None
    if hasattr(server, "app"):
        # Unity3DMCP, etc. have .app attribute
        mcp_instance = server.app
    elif hasattr(server, "mcp"):
        # Some servers have .mcp attribute
        mcp_instance = server.mcp
    elif hasattr(server, "__class__"):
        # Check if server is already a FastMCP instance
        class_name = server.__class__.__name__
        if "FastMCP" in class_name or hasattr(server, "list_tools"):
            mcp_instance = server
        else:
            raise ValueError(f"Cannot determine FastMCP instance for server {server_name} (class: {class_name})")
    else:
        raise ValueError(f"Cannot determine FastMCP instance for server {server_name}")
    
    # Call tool via Client on the server's FastMCP instance
    async with Client(mcp_instance) as client:
        # FastMCP Client.call_tool expects (tool_name, arguments_dict)
        result = await client.call_tool(tool_name, params or {})
        return result
