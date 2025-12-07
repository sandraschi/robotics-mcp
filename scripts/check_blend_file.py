"""Check what objects are in the Scout blend file."""
import sys
from pathlib import Path

# This script needs to be run with Blender's Python
# blender.exe --background --python scripts/check_blend_file.py

blend_file = Path("D:/Models/scout_model.blend")

if not blend_file.exists():
    print(f"ERROR: Blend file not found: {blend_file}")
    sys.exit(1)

import bpy

# Clear existing scene
bpy.ops.wm.read_homefile(app_template="")

# Open the blend file
bpy.ops.wm.open_mainfile(filepath=str(blend_file))

print(f"\n{'='*60}")
print(f"Objects in {blend_file.name}:")
print(f"{'='*60}")
print(f"Total objects: {len(bpy.data.objects)}")
print(f"\nObject details:")
for obj in bpy.data.objects:
    rotation_str = f"Rotation: {obj.rotation_euler}" if hasattr(obj, 'rotation_euler') else "No rotation"
    print(f"  - {obj.name:30s} | Type: {obj.type:10s} | Location: {obj.location} | {rotation_str}")

print(f"\n{'='*60}")
print("To view in Blender:")
print("1. Open the blend file")
print("2. Press '.' (period) to frame all objects")
print("3. Or press 'A' to select all, then 'Numpad .' to frame selected")
print(f"{'='*60}\n")

