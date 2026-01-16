#!/usr/bin/env python3
"""Test tool registration to understand FastMCP decorator behavior."""

import asyncio
from src.robotics_mcp.tools.robot_control import RobotControlTool
from src.robotics_mcp.utils.state_manager import RobotStateManager
from fastmcp import FastMCP

async def test_robot_control():
    print('=== TESTING ROBOT CONTROL TOOL REGISTRATION ===')

    # Create a minimal setup like the tests do
    mcp = FastMCP(name='test', version='0.1.0')
    state_manager = RobotStateManager()
    mounted_servers = {}

    robot_tool = RobotControlTool(mcp, state_manager, mounted_servers)
    robot_tool.register()

    # Check what the decorator does
    print(f'MCP tool attribute: {hasattr(mcp, "tool")}')
    if hasattr(mcp, 'tool'):
        print(f'MCP.tool type: {type(mcp.tool)}')
        print(f'MCP.tool registered_func: {hasattr(mcp.tool, "registered_func")}')
        if hasattr(mcp.tool, 'registered_func'):
            print(f'Registered func type: {type(mcp.tool.registered_func)}')

    # Check internal tools dict
    tools = getattr(mcp, '_tools', {})
    print(f'Internal _tools dict has {len(tools)} tools: {list(tools.keys())}')

if __name__ == "__main__":
    asyncio.run(test_robot_control())