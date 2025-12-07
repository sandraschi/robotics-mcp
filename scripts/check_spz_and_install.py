"""Check .spz support and install Unity plugins.

This script checks for .spz conversion tools and installs Unity plugins
for Gaussian splat support (as an alternative to .spz).
"""

import asyncio
import sys
from pathlib import Path

# Add src to path
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from robotics_mcp.tools.spz_converter import SPZConverterTool
from fastmcp import FastMCP


async def main():
    """Check .spz support and provide installation options."""
    mcp = FastMCP("SPZ-Checker")
    tool = SPZConverterTool(mcp)
    tool.register()

    print("=" * 60)
    print("SPZ File Support Check")
    print("=" * 60)
    print()

    # Check support
    print("1. Checking .spz conversion tools...")
    async with mcp.client() as client:
        result = await client.call_tool("spz_converter", {"operation": "check_spz_support"})
        print(f"   Status: {result.get('status', 'unknown')}")
        if result.get("data"):
            tools = result["data"].get("tools_available", {})
            print(f"   Adobe spz-tools: {'✅' if tools.get('adobe_spz_tools') else '❌'}")
            print(f"   Python spz lib: {'✅' if tools.get('python_spz_lib') else '❌'}")
            print(f"   Manual conversion: {'✅' if tools.get('manual_conversion') else '❌'}")
            if result["data"].get("recommendations"):
                print("\n   Recommendations:")
                for rec in result["data"]["recommendations"]:
                    print(f"   - {rec}")

    print()
    print("2. Extracting .spz file info...")
    spz_path = r"C:\Users\sandr\Downloads\Modern Tropical Luxury Residence.spz"
    if Path(spz_path).exists():
        async with mcp.client() as client:
            result = await client.call_tool("spz_converter", {"operation": "extract_spz_info", "spz_path": spz_path})
            if result.get("status") == "success" and result.get("data"):
                data = result["data"]
                print(f"   File: {data.get('file_path')}")
                print(f"   Size: {data.get('file_size_mb')} MB")
                print(f"   Format: {data.get('format')}")
                print(f"   Note: {data.get('note')}")

    print()
    print("=" * 60)
    print("Installation Options")
    print("=" * 60)
    print()
    print("Since there's no Unity plugin for .spz files, here are your options:")
    print()
    print("Option 1: Install Gaussian Splatting Plugin (for .ply files)")
    print("  - Re-export from Marble as .ply")
    print("  - Install plugin via: unity_install_gaussian_splatting")
    print()
    print("Option 2: Use Mesh Format (Recommended for Robotics)")
    print("  - Re-export from Marble as .fbx or .glb")
    print("  - Full Unity support, no plugins needed")
    print("  - Better for NavMesh and navigation")
    print()
    print("Option 3: Build Custom Converter")
    print("  - Use Adobe's spz library: https://github.com/adobe/spz")
    print("  - Convert .spz to .ply manually")
    print()


if __name__ == "__main__":
    asyncio.run(main())

