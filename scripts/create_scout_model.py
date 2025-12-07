"""Script to create Scout 3D model using robotics-mcp tools.

Creates a Scout model with accurate dimensions from hardware specs:
- Length: 11.5 cm (0.115 m)
- Width: 10 cm (0.10 m)
- Height: 8 cm (0.08 m)
- 4 mecanum wheels
- Camera on front
"""

import asyncio
import sys
from pathlib import Path

# Add src to path
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from robotics_mcp.server import RoboticsMCP, RoboticsConfig
from robotics_mcp.tools.robot_model_tools import RobotModelTools
from robotics_mcp.utils.state_manager import RobotStateManager


async def create_scout_model():
    """Create Scout 3D model."""
    print("=" * 60)
    print("Creating Scout 3D Model")
    print("=" * 60)
    print("Dimensions: 11.5cm × 10cm × 8cm (0.115m × 0.10m × 0.08m)")
    print("Features: 4 mecanum wheels, front camera")
    print("-" * 60)

    # Initialize server
    config = RoboticsConfig()
    state_manager = RobotStateManager()
    server = RoboticsMCP(config)
    
    # Create output directory
    output_dir = Path("D:/Models")
    output_dir.mkdir(parents=True, exist_ok=True)
    output_path = output_dir / "scout_model.fbx"
    
    print(f"Output path: {output_path}")
    print("-" * 60)
    print("Initializing model creation...")
    
    try:
        from fastmcp import Client
        async with Client(server.mcp) as client:
            print("Calling robot_model tool with operation='create'...")
            
            result = await client.call_tool(
                "robot_model",  # Call the portmanteau tool
                {
                    "operation": "create",  # Specify the operation
                    "robot_type": "scout",
                    "output_path": str(output_path),
                    "format": "fbx",
                    "dimensions": {
                        "length": 0.115,  # 11.5 cm
                        "width": 0.10,    # 10 cm
                        "height": 0.08,   # 8 cm
                    },
                    "create_textures": True,
                    "texture_style": "realistic",
                },
            )
            
            print("\n" + "=" * 60)
            print("Result:")
            print("=" * 60)
            
            # Extract result data
            if hasattr(result, 'data'):
                result_data = result.data
            elif hasattr(result, 'structured_content'):
                result_data = result.structured_content
            else:
                result_data = result
            
            print(result_data)
            
            status = result_data.get("status") if isinstance(result_data, dict) else "unknown"
            
            if status == "success":
                print("\nSUCCESS: Scout model created!")
                print(f"Location: {output_path}")
            else:
                print("\nWARNING: Model creation completed with issues")
                message = result_data.get("message", "Unknown") if isinstance(result_data, dict) else str(result_data)
                print(f"Message: {message}")
                
    except Exception as e:
        print(f"\nERROR: Failed to create model: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    asyncio.run(create_scout_model())

