"""Import Scout FBX model into Unity3D project.

This script copies the Scout FBX file to Unity's Assets folder,
where Unity will automatically import it.

Usage:
    python scripts/import_scout_to_unity.py [unity_project_path]

Example:
    python scripts/import_scout_to_unity.py "D:/Projects/MyUnityProject"
"""

import shutil
import sys
from pathlib import Path

# Paths
SCOUT_FBX = Path("D:/Models/scout_model.fbx")

# Get Unity project path from command line or use default
if len(sys.argv) > 1:
    UNITY_PROJECT = Path(sys.argv[1])
else:
    UNITY_PROJECT = Path("C:/Users/sandr/My project")  # Default Unity project path

UNITY_ASSETS = UNITY_PROJECT / "Assets" / "Models"

def import_scout_to_unity():
    """Copy Scout FBX to Unity Assets folder."""
    print("=" * 60)
    print("Importing Scout Model to Unity3D")
    print("=" * 60)
    
    # Check if Scout FBX exists
    if not SCOUT_FBX.exists():
        print(f"ERROR: Scout FBX not found at {SCOUT_FBX}")
        print("Please run create_scout_model.py first to generate the model.")
        return False
    
    # Create Models folder in Unity Assets
    UNITY_ASSETS.mkdir(parents=True, exist_ok=True)
    print(f"Created/verified Unity Assets folder: {UNITY_ASSETS}")
    
    # Copy FBX to Unity Assets
    dest_path = UNITY_ASSETS / SCOUT_FBX.name
    shutil.copy2(SCOUT_FBX, dest_path)
    print(f"Copied Scout FBX to: {dest_path}")
    print(f"File size: {dest_path.stat().st_size} bytes")
    
    print("-" * 60)
    print("SUCCESS: Scout model copied to Unity Assets folder")
    print("-" * 60)
    print("\nNext steps:")
    print("1. Open Unity Editor (if not already open)")
    print("2. Unity will automatically detect and import the FBX")
    print("3. Check Unity Console for import status")
    print("4. Find the model in Project window: Assets/Models/scout_model.fbx")
    print("5. Drag to Scene to instantiate, or create a Prefab")
    print("\nTo spawn via robotics-mcp:")
    print("  - Ensure VbotSpawner.cs is in your scene")
    print("  - Assign Scout prefab to VbotSpawner's robotPrefabs list")
    print("  - Use vbot_crud tool to spawn: operation='create', robot_type='scout'")
    
    return True

if __name__ == "__main__":
    import_scout_to_unity()

