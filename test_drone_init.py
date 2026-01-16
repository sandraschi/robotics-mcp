#!/usr/bin/env python3
"""Quick test of drone tool integration."""

import asyncio
from fastmcp import FastMCP
from src.robotics_mcp.tools.drone_control import DroneControlTool

async def test_drone_tools():
    """Test that drone tools can be instantiated."""
    print("=== TESTING DRONE TOOL INSTANTIATION ===")
    try:
        print("1. Creating mock MCP server...")
        mcp = FastMCP("test")

        print("2. Creating mock state manager...")
        class MockStateManager:
            def get_robot(self, robot_id):
                return type('MockRobot', (), {'id': robot_id})()

        state_manager = MockStateManager()

        print("3. Creating drone control tool...")
        drone_tool = DroneControlTool(mcp, state_manager, {})

        print("4. Testing tool registration...")
        drone_tool.register()

        print("5. Checking registered tools...")
        tools = getattr(mcp, "_tools", {})
        drone_tools = [name for name in tools.keys() if 'drone' in name.lower()]
        print(f"6. Found {len(drone_tools)} drone tools: {drone_tools}")

        if drone_tools:
            print("✅ Drone tools registered successfully!")
            for tool_name in drone_tools:
                print(f"   - {tool_name}")
        else:
            print("❌ No drone tools found")

    except Exception as e:
        print(f"❌ Exception caught: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("=== TEST COMPLETE ===")

if __name__ == "__main__":
    asyncio.run(test_drone_tools())