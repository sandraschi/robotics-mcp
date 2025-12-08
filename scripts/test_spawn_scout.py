"""
Test script for spawning Scout vbot in Unity.

Usage:
    python scripts/test_spawn_scout.py

Prerequisites:
    - Unity Editor running with project open
    - VbotSpawner GameObject in scene
    - Scout Prefab assigned to VbotSpawner
    - robotics-mcp server running
"""
import asyncio
import sys
from pathlib import Path

# Add project root to path
script_dir = Path(__file__).resolve().parent
project_root = script_dir.parent
if str(project_root) not in sys.path:
    sys.path.insert(0, str(project_root))

from robotics_mcp.server import RoboticsMCP, RoboticsConfig
from fastmcp import Client
from fastmcp.messages import CallToolResult


async def test_spawn_scout():
    """Test spawning a Scout vbot in Unity."""
    print("=" * 60)
    print("Test: Spawn Scout Vbot in Unity")
    print("=" * 60)

    # Initialize server
    config = RoboticsConfig()
    server = RoboticsMCP(config)

    async with Client(server.mcp) as client:
        # Test 1: Spawn Scout at origin
        print("\n[Test 1] Spawning Scout at origin (0, 0, 0)...")
        try:
            result = await client.call_tool(
                "vbot_crud",
                {
                    "operation": "create",
                    "robot_type": "scout",
                    "robot_id": "scout_01",
                    "platform": "unity",
                    "position": {"x": 0.0, "y": 0.0, "z": 0.0},
                    "scale": 1.0,
                },
            )
            print_result("Spawn Scout", result)
        except Exception as e:
            print(f"❌ Error: {e}")

        # Test 2: List spawned robots
        print("\n[Test 2] Listing spawned robots...")
        try:
            result = await client.call_tool(
                "vbot_crud",
                {
                    "operation": "list",
                    "platform": "unity",
                },
            )
            print_result("List Robots", result)
        except Exception as e:
            print(f"❌ Error: {e}")

        # Test 3: Update Scout position
        print("\n[Test 3] Updating Scout position...")
        try:
            result = await client.call_tool(
                "vbot_crud",
                {
                    "operation": "update",
                    "robot_id": "scout_01",
                    "position": {"x": 2.0, "y": 0.0, "z": 2.0},
                    "scale": 1.0,
                },
            )
            print_result("Update Scout", result)
        except Exception as e:
            print(f"❌ Error: {e}")

        # Test 4: Read Scout info
        print("\n[Test 4] Reading Scout info...")
        try:
            result = await client.call_tool(
                "vbot_crud",
                {
                    "operation": "read",
                    "robot_id": "scout_01",
                },
            )
            print_result("Read Scout", result)
        except Exception as e:
            print(f"❌ Error: {e}")

        # Test 5: Spawn second Scout (larger scale)
        print("\n[Test 5] Spawning second Scout (10× scale)...")
        try:
            result = await client.call_tool(
                "vbot_crud",
                {
                    "operation": "create",
                    "robot_type": "scout",
                    "robot_id": "scout_02",
                    "platform": "unity",
                    "position": {"x": 5.0, "y": 0.0, "z": 5.0},
                    "scale": 10.0,  # 10× bigger for visibility
                },
            )
            print_result("Spawn Scout (Large)", result)
        except Exception as e:
            print(f"❌ Error: {e}")

    print("\n" + "=" * 60)
    print("Test Complete!")
    print("=" * 60)
    print("\nNext Steps:")
    print("  1. Check Unity Scene - Scouts should be visible")
    print("  2. Check Unity Console for any errors")
    print("  3. If Scouts are too small, use scale=10.0 or higher")
    print("  4. Test robot_animation, robot_camera, robot_navigation tools")


def print_result(test_name: str, result: CallToolResult):
    """Print test result."""
    if result.is_error:
        error_message = "Unknown error"
        if result.structured_content and "message" in result.structured_content:
            error_message = result.structured_content["message"]
        elif result.content:
            error_message = result.content[0].text
        print(f"❌ {test_name}: FAILED")
        print(f"   Error: {error_message}")
    else:
        print(f"✅ {test_name}: SUCCESS")
        if result.structured_content:
            if "message" in result.structured_content:
                print(f"   {result.structured_content['message']}")
            if "data" in result.structured_content:
                data = result.structured_content["data"]
                if isinstance(data, dict):
                    for key, value in data.items():
                        print(f"   {key}: {value}")
        elif result.content:
            print(f"   {result.content[0].text}")


if __name__ == "__main__":
    asyncio.run(test_spawn_scout())

