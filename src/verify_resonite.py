import asyncio
import logging
from unittest.mock import MagicMock

from robotics_mcp.tools.robot_virtual import RobotVirtualTool

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


async def test_resonite_integration():
    """Test the Resonite platform integration in RobotVirtualTool."""
    print("Starting Resonite Integration Test...")

    # Mock MCP
    mock_mcp = MagicMock()
    mock_state_manager = MagicMock()

    async def async_register(*args, **kwargs):
        return {"status": "success"}

    mock_state_manager.register_robot.side_effect = async_register

    # Initialize RobotVirtualTool
    # Passing arguments as expected by src/robotics_mcp/tools/robot_virtual.py
    # Using try-except on init just in case signatures mismatch due to PYTHONPATH issues
    try:
        tool = RobotVirtualTool(mcp=mock_mcp, state_manager=mock_state_manager, mounted_servers={})
        print("RobotVirtualTool initialized successfully with state_manager.")
    except TypeError as e:
        print(f"Init failed with state_manager: {e}")
        print("Attempting fallback init...")
        tool = RobotVirtualTool(mcp=mock_mcp)
        print("RobotVirtualTool initialized with mcp only.")

    # 1. Test Spawning a Robot in Resonite
    print("\n[Test 1] Spawning Robot in Resonite...")
    try:
        spawn_result = await tool.handle_operations(
            operation="spawn",
            robot_type="scout",
            platform="resonite",
            position={"x": 1.0, "y": 0.0, "z": 1.0},
            robot_id="test_scout_01",
        )
        print(f"Spawn Result: {spawn_result}")
    except Exception as e:
        print(f"Spawn Failed: {e}")
        import traceback

        traceback.print_exc()

    # 2. Test Updating a Robot in Resonite
    print("\n[Test 2] Updating Robot in Resonite...")
    try:
        update_result = await tool.handle_operations(
            operation="update",
            robot_id="test_scout_01",
            platform="resonite",
            position={"x": 2.0, "y": 0.0, "z": 2.0},
            scale=1.5,
        )
        print(f"Update Result: {update_result}")
    except Exception as e:
        print(f"Update Failed: {e}")
        import traceback

        traceback.print_exc()

    # 3. Test Invalid Platform
    print("\n[Test 3] Testing Invalid Platform...")
    try:
        await tool.handle_operations(
            operation="spawn",
            robot_type="scout",
            platform="invalid_platform",
            robot_id="test_scout_02",
        )
    except Exception as e:
        # Expected to print an error response, not necessarily raise exception if the tool handles it safely
        print(f"Caught exception (if any): {e}")

    # 4. Test List Robots in Resonite
    print("\n[Test 4] Listing Robots in Resonite...")
    try:
        list_result = await tool.handle_operations(operation="list", platform="resonite")
        print(f"List Result: {list_result}")
    except Exception as e:
        print(f"List Failed: {e}")
        import traceback

        traceback.print_exc()


if __name__ == "__main__":
    asyncio.run(test_resonite_integration())
