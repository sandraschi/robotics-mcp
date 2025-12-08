"""Import Marble living room into Unity.

This script imports the Marble apartment environment files:
- GLB collider mesh (for physics/navigation)
- PLY splat files (for visuals)
"""

import asyncio
import sys
import json
from pathlib import Path

# Add src to path
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from fastmcp import Client
from robotics_mcp.server import RoboticsMCP, RoboticsConfig


async def import_living_room():
    """Import the Marble living room into Unity."""
    # Initialize server
    config = RoboticsConfig()
    server = RoboticsMCP(config)
    
    unity_project = r"C:\Users\sandr\My project"
    downloads = Path("C:/Users/sandr/Downloads")
    
    print("=" * 60)
    print("Importing Marble Living Room to Unity")
    print("=" * 60)
    print()
    print(f"Unity Project: {unity_project}")
    print()
    
    # Find files
    print("Scanning for Marble residence files...")
    
    # GLB collider file
    glb_file = downloads / "Modern Tropical Luxury Residence_collider.glb"
    
    # PLY files (with UUID names)
    ply_files = [
        downloads / "cafcb2c0-c073-435e-adc5-406e6588e0db_ceramic_500k.ply",
        downloads / "86b22779-6d6e-4382-a7d4-7d606b3d7611_ceramic.ply"
    ]
    
    files_to_import = []
    
    # Check GLB collider
    if glb_file.exists():
        size_mb = glb_file.stat().st_size / (1024 * 1024)
        print(f"[OK] Found collider: {glb_file.name} ({size_mb:.1f} MB)")
        files_to_import.append(("collider", str(glb_file)))
    else:
        print(f"[ERROR] Collider not found: {glb_file.name}")
    
    # Check PLY files
    found_ply = []
    for ply_file in ply_files:
        if ply_file.exists():
            size_mb = ply_file.stat().st_size / (1024 * 1024)
            print(f"[OK] Found splat: {ply_file.name} ({size_mb:.1f} MB)")
            found_ply.append(str(ply_file))
        else:
            print(f"[ERROR] Splat not found: {ply_file.name}")
    
    if found_ply:
        files_to_import.append(("splats", found_ply))
    
    if not files_to_import:
        print()
        print("[ERROR] No files found to import!")
        print("Expected files:")
        print(f"  - {glb_file.name}")
        for ply in ply_files:
            print(f"  - {ply.name}")
        return
    
    print()
    print(f"Found {len(files_to_import)} file group(s) to import")
    print()
    
    # Import via virtual_robotics tool
    async with Client(server.mcp) as client:
        results = []
        
        # Import collider GLB
        if any(item[0] == "collider" for item in files_to_import):
            collider_path = next(item[1] for item in files_to_import if item[0] == "collider")
            print("=" * 60)
            print("Importing Collider (GLB)")
            print("=" * 60)
            print(f"File: {Path(collider_path).name}")
            print()
            
            try:
                result = await client.call_tool(
                    "robot_virtual",
                    {
                        "operation": "load_environment",
                        "environment": "Modern Tropical Luxury Residence",
                        "environment_path": collider_path,
                        "platform": "unity",
                        "project_path": unity_project,
                        "include_colliders": True,
                    }
                )
                results.append(("Collider", result))
                print_result("Collider", result)
            except Exception as e:
                print(f"[ERROR] Error importing collider: {e}")
                import traceback
                traceback.print_exc()
        
        # Import PLY splats
        if any(item[0] == "splats" for item in files_to_import):
            splat_paths = next(item[1] for item in files_to_import if item[0] == "splats")
            print()
            print("=" * 60)
            print("Importing Visual Splats (PLY)")
            print("=" * 60)
            
            for i, splat_path in enumerate(splat_paths, 1):
                print(f"File {i}/{len(splat_paths)}: {Path(splat_path).name}")
                print()
                
                try:
                    result = await client.call_tool(
                        "robot_virtual",
                        {
                            "operation": "load_environment",
                            "environment": "Modern Tropical Luxury Residence",
                            "environment_path": splat_path,
                            "platform": "unity",
                            "project_path": unity_project,
                            "include_colliders": False,  # Splats are visual only
                        }
                    )
                    results.append((f"Splat {i}", result))
                    print_result(f"Splat {i}", result)
                    print()
                except Exception as e:
                    print(f"[ERROR] Error importing splat {i}: {e}")
                    import traceback
                    traceback.print_exc()
                    print()
        
        # Summary
        print()
        print("=" * 60)
        print("Import Summary")
        print("=" * 60)
        print()
        
        # Extract content from CallToolResult objects
        success_count = 0
        for name, result in results:
            content = extract_result_data(result)
            if isinstance(content, dict) and content.get("status") == "success":
                success_count += 1
        
        print(f"[OK] Successfully imported: {success_count}/{len(results)}")
        print()
        
        print("Next Steps:")
        print("  1. Open Unity Editor")
        print("  2. Wait for assets to import (may take a few minutes)")
        print("  3. Find assets in: Assets/WorldLabs/")
        print("  4. Collider GLB: Use for NavMesh and physics")
        print("  5. PLY splats: Use for visual rendering (requires Gaussian Splatting plugin)")
        print("  6. Add NavMesh: Window > AI > Navigation > Bake")
        print("  7. Spawn Scout vbot in the environment")


def extract_result_data(result):
    """Extract data from CallToolResult object."""
    # Try .data attribute first (most direct)
    if hasattr(result, 'data') and result.data:
        return result.data
    
    # Try .content attribute
    if hasattr(result, 'content') and result.content:
        if isinstance(result.content, list) and len(result.content) > 0:
            first_content = result.content[0]
            if hasattr(first_content, 'text'):
                try:
                    return json.loads(first_content.text)
                except:
                    return {"status": "success", "message": first_content.text}
        elif hasattr(result.content, 'text'):
            try:
                return json.loads(result.content.text)
            except:
                return {"status": "success", "message": result.content.text}
    
    # Fallback
    return {"status": "unknown", "raw": str(result)}


def print_result(name, result):
    """Print import result."""
    content = extract_result_data(result)
    
    if isinstance(content, dict):
        if content.get("status") == "success":
            print(f"[OK] {name}: SUCCESS")
            
            # Check for nested unity_result
            if content.get("data") and isinstance(content["data"], dict):
                data = content["data"]
                if data.get("unity_result"):
                    unity_result = data["unity_result"]
                    if isinstance(unity_result, dict):
                        if unity_result.get("unity_import_path"):
                            print(f"   Unity Path: {unity_result.get('unity_import_path')}")
                        if unity_result.get("mesh_path"):
                            print(f"   Mesh Path: {unity_result.get('mesh_path')}")
                        if unity_result.get("splat_path"):
                            print(f"   Splat Path: {unity_result.get('splat_path')}")
                        if unity_result.get("gaussian_splatting_installed"):
                            print(f"   Gaussian Splatting: Installed [OK]")
            elif content.get("unity_import_path"):
                print(f"   Unity Path: {content.get('unity_import_path')}")
            elif content.get("message"):
                print(f"   {content.get('message')}")
        else:
            print(f"[ERROR] {name}: ERROR")
            print(f"   {content.get('message', 'Unknown error')}")
    else:
        print(f"[WARN] {name}: Unexpected result format")
        print(f"   {type(content).__name__}: {content}")


if __name__ == "__main__":
    asyncio.run(import_living_room())
