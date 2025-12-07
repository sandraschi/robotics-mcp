"""Test script to verify wheel rotation works in Blender."""
import bpy

# Clear scene
bpy.ops.object.select_all(action='SELECT')
bpy.ops.object.delete(use_global=False)

# Create a test body
bpy.ops.mesh.primitive_cube_add(size=1, location=(0, 0, 0.04), scale=(0.115, 0.10, 0.08))
body = bpy.context.active_object
body.name = "test_body"

# Create a wheel - horizontal (default)
bpy.ops.mesh.primitive_cylinder_add(radius=0.025, depth=0.015, location=(-0.08, 0.05, 0.025))
wheel1 = bpy.context.active_object
wheel1.name = "wheel_horizontal"

# Create a wheel - vertical (rotated 90° on X-axis)
bpy.ops.mesh.primitive_cylinder_add(
    radius=0.025, 
    depth=0.015, 
    location=(0.08, 0.05, 0.025),
    rotation=(1.5708, 0, 0)  # 90 degrees on X-axis
)
wheel2 = bpy.context.active_object
wheel2.name = "wheel_vertical"

print("Created test objects:")
print(f"  - {body.name} at {body.location}")
print(f"  - {wheel1.name} at {wheel1.location} (horizontal)")
print(f"  - {wheel2.name} at {wheel2.location} (vertical, rotated)")

# Save test file
bpy.ops.wm.save_as_mainfile(filepath=r"D:\Models\test_wheels.blend")
print("Saved test file: D:\\Models\\test_wheels.blend")

