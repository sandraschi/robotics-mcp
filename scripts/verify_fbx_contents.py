"""Verify FBX file contents by importing it back into Blender."""

import bpy
from pathlib import Path

# Clear scene
bpy.ops.object.select_all(action='SELECT')
bpy.ops.object.delete(use_global=False)

# Import FBX
fbx_path = Path("D:/Models/scout_model.fbx")
if not fbx_path.exists():
    print(f"ERROR: FBX file not found at {fbx_path}")
    exit(1)

print(f"Importing FBX: {fbx_path}")
bpy.ops.import_scene.fbx(filepath=str(fbx_path))

print(f"\nImported {len(bpy.data.objects)} objects:")
for obj in bpy.data.objects:
    print(f"  - {obj.name}: {obj.type} at {obj.location}")

# Check if we have all Scout components
expected_objects = ["scout_body", "scout_wheel_1", "scout_wheel_2", "scout_wheel_3", "scout_wheel_4", "scout_camera", "scout_mounting_plate"]
found_objects = [obj.name for obj in bpy.data.objects]

print(f"\nExpected objects: {len(expected_objects)}")
print(f"Found objects: {len(found_objects)}")

missing = [name for name in expected_objects if name not in found_objects]
if missing:
    print(f"\nWARNING: Missing objects: {missing}")
else:
    print("\nSUCCESS: All Scout components found in FBX!")

