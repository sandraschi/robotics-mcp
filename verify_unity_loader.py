import asyncio
from unittest.mock import MagicMock, patch

from mcp.server.fastmcp import FastMCP

from robotics_mcp.tools.robot_virtual import RobotVirtualTool


# Mock Client context manager
class MockClientContext:
    def __init__(self, mcp_instance):
        self.mcp_instance = mcp_instance

    async def __aenter__(self):
        return self

    async def __aexit__(self, exc_type, exc_val, exc_tb):
        pass

    async def call_tool(self, name, arguments=None):
        print(f"Called tool: {name} with args: {arguments}")
        if name == "unity_import_marble_world":
            return {"content": [{"text": '{"status": "success", "imported": true}'}]}
        return {"content": [{"text": '{"status": "success"}'}]}


async def verify_loader():
    # Patch Client in mcp_client_helper
    with patch("robotics_mcp.utils.mcp_client_helper.Client", side_effect=MockClientContext):
        print("Initializing RobotVirtualTool...")

        # Mock FastMCP
        mcp_mock = MagicMock(spec=FastMCP)
        mcp_mock.tool = MagicMock(return_value=lambda x: x)

        # Mock State Manager
        state_manager_mock = MagicMock()

        # Mock Mounted Servers
        # Needs to have an attribute that call_mounted_server_tool checks
        unity_server_mock = MagicMock()
        unity_server_mock.mcp = MagicMock()  # Represents the FastMCP instance

        mounted_servers_mock = {"unity": unity_server_mock}

        tool = RobotVirtualTool(mcp_mock, state_manager_mock, mounted_servers_mock)

        print("\nTesting 'load_environment' for Unity...")
        # Use handle_operations instead of private method if possible, or use the public method exposed by decorator
        # But RobotVirtualTool uses helper method.
        # Let's call the internal handler directly for verification
        result = await tool.handle_operations(
            operation="load_environment",
            environment="stroheckgasse_apartment",
            platform="unity",
            environment_path="D:/Marble/stroheckgasse.glb",
        )

        print(f"Result: {result}")

        # format_success_response puts 'unity_result' at top level
        if result.get("unity_result", {}).get("status") == "success" or result.get("status") == "success":
            print("SUCCESS: Environment load command dispatched successfully.")
        else:
            print("FAILURE: Environment load command failed.")


if __name__ == "__main__":
    asyncio.run(verify_loader())
