"""List all available tools in robotics-mcp server."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from robotics_mcp.server import RoboticsMCP, RoboticsConfig

config = RoboticsConfig()
server = RoboticsMCP(config)

# Get all tools
all_tools = server.mcp._tool_manager._tools if hasattr(server.mcp, '_tool_manager') else {}

print("=" * 60)
print("Available Tools in Robotics MCP")
print("=" * 60)
print(f"Total tools: {len(all_tools)}")
print()

# Filter blender tools
blender_tools = [t for t in all_tools.keys() if 'blender' in t.lower()]
print(f"Blender tools ({len(blender_tools)}):")
for tool in sorted(blender_tools):
    print(f"  - {tool}")

print()
print("All tools:")
for tool in sorted(all_tools.keys()):
    print(f"  - {tool}")

