"""Script to create a proper mecanum wheel model for Scout robot.

Mecanum wheels have rollers at 45-degree angles that enable omnidirectional movement.
This creates a realistic wheel with proper geometry.
"""

import bpy
import math
from mathutils import Vector, Euler
from typing import Tuple, List


def create_mecanum_wheel(
    wheel_radius: float = 0.025,
    wheel_thickness: float = 0.015,
    roller_count: int = 12,
    roller_angle: float = 45.0,  # degrees - typical for mecanum wheels
    location: Tuple[float, float, float] = (0, 0, 0),
    rotation: Tuple[float, float, float] = (0, 0, 0),
    name: str = "mecanum_wheel"
) -> bpy.types.Object:
    """Create a proper mecanum wheel with angled rollers.

    Args:
        wheel_radius: Radius of the main wheel
        wheel_thickness: Thickness of the wheel
        roller_count: Number of rollers around circumference
        roller_angle: Angle of rollers in degrees (45° for standard mecanum)
        location: Position to place the wheel
        rotation: Rotation to apply to the wheel
        name: Name for the wheel object

    Returns:
        The created wheel object
    """

    # Create main wheel hub (cylinder)
    bpy.ops.mesh.primitive_cylinder_add(
        radius=wheel_radius * 0.3,  # Hub is smaller than full radius
        depth=wheel_thickness,
        location=location,
        rotation=rotation
    )
    wheel_hub = bpy.context.active_object
    wheel_hub.name = f"{name}_hub"

    # Create main wheel rim (torus/donut shape for rollers to attach to)
    bpy.ops.mesh.primitive_torus_add(
        major_radius=wheel_radius * 0.8,  # Slightly smaller than full radius
        minor_radius=wheel_radius * 0.15,  # Thickness of the rim
        major_segments=roller_count * 2,  # Match roller count for clean attachment
        minor_segments=8,
        location=location,
        rotation=rotation
    )
    wheel_rim = bpy.context.active_object
    wheel_rim.name = f"{name}_rim"

    # Create rollers
    rollers = []
    angle_step = 2 * math.pi / roller_count

    for i in range(roller_count):
        angle = i * angle_step

        # Position roller on circumference
        roller_x = location[0] + math.cos(angle) * (wheel_radius * 0.8)
        roller_y = location[1] + math.sin(angle) * (wheel_radius * 0.8)
        roller_z = location[2]

        # Create roller cylinder
        bpy.ops.mesh.primitive_cylinder_add(
            radius=wheel_radius * 0.12,  # Roller radius
            depth=wheel_thickness * 0.9,  # Slightly shorter than wheel
            location=(roller_x, roller_y, roller_z),
            rotation=(rotation[0], rotation[1], rotation[2] + math.radians(roller_angle))
        )

        roller = bpy.context.active_object
        roller.name = f"{name}_roller_{i+1}"
        rollers.append(roller)

        # Rotate roller around the wheel's circumference
        roller.rotation_euler.z += angle

    # Parent all rollers to the rim for easy manipulation
    for roller in rollers:
        roller.parent = wheel_rim

    # Create a main wheel object that groups everything
    bpy.ops.object.empty_add(location=location, rotation=rotation)
    main_wheel = bpy.context.active_object
    main_wheel.name = name

    # Parent hub and rim to main wheel
    wheel_hub.parent = main_wheel
    wheel_rim.parent = main_wheel

    return main_wheel


def create_scout_with_mecanum_wheels():
    """Create a complete Scout model with proper mecanum wheels."""

    # Clear existing objects
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete(use_global=False)

    # Scout dimensions (in meters)
    scout_length = 0.115
    scout_width = 0.10
    scout_height = 0.08

    # Wheel specifications
    wheel_radius = 0.025
    wheel_thickness = 0.015

    # Create main body
    bpy.ops.mesh.primitive_cube_add(
        size=1,
        location=(0, 0, scout_height / 2),
        scale=(scout_length, scout_width, scout_height)
    )
    body = bpy.context.active_object
    body.name = "scout_body"

    # Wheel positions (at corners of body)
    wheel_positions = [
        (-scout_length/2 - wheel_radius, scout_width/2, wheel_radius),  # Front-left
        (scout_length/2 + wheel_radius, scout_width/2, wheel_radius),   # Front-right
        (-scout_length/2 - wheel_radius, -scout_width/2, wheel_radius), # Back-left
        (scout_length/2 + wheel_radius, -scout_width/2, wheel_radius),  # Back-right
    ]

    wheel_names = ["front_left", "front_right", "back_left", "back_right"]

    # Create wheels
    wheels = []
    for i, (pos, name) in enumerate(zip(wheel_positions, wheel_names)):
        # Mecanum wheels need to be rotated 90° around X-axis to stand vertically
        wheel = create_mecanum_wheel(
            wheel_radius=wheel_radius,
            wheel_thickness=wheel_thickness,
            location=pos,
            rotation=(math.radians(90), 0, 0),  # Rotate to vertical orientation
            name=f"scout_wheel_{name}"
        )
        wheels.append(wheel)

        # For mecanum wheels, alternate the roller angle direction for proper movement
        # Front-left and back-right should have rollers angled one way
        # Front-right and back-left should have rollers angled the opposite way
        if name in ["front_left", "back_right"]:
            roller_angle = 45.0  # Positive angle
        else:
            roller_angle = -45.0  # Negative angle

        # Update roller rotations (this is simplified - in a full implementation,
        # you'd need to update each roller's rotation individually)
        for child in wheel.children_recursive:
            if "roller" in child.name:
                child.rotation_euler.y += math.radians(roller_angle)

    # Add camera
    camera_size = 0.012
    bpy.ops.mesh.primitive_cylinder_add(
        radius=camera_size,
        depth=camera_size * 0.8,
        location=(scout_length/2 + 0.008, 0, scout_height / 2),
        rotation=(0, 0, 0)
    )
    camera = bpy.context.active_object
    camera.name = "scout_camera"

    # Add mounting plate on top
    bpy.ops.mesh.primitive_cube_add(
        size=1,
        location=(0, 0, scout_height + 0.005),
        scale=(scout_length * 0.8, scout_width * 0.8, 0.01)
    )
    plate = bpy.context.active_object
    plate.name = "scout_mounting_plate"

    # Add materials for visibility
    # Body material (blue)
    mat_body = bpy.data.materials.new(name="ScoutBody")
    mat_body.use_nodes = True
    mat_body.node_tree.nodes["Principled BSDF"].inputs[0].default_value = (0.2, 0.2, 0.8, 1.0)
    body.data.materials.append(mat_body)

    # Wheel material (dark gray)
    mat_wheel = bpy.data.materials.new(name="ScoutWheel")
    mat_wheel.use_nodes = True
    mat_wheel.node_tree.nodes["Principled BSDF"].inputs[0].default_value = (0.1, 0.1, 0.1, 1.0)

    # Apply wheel material to all wheel parts
    for wheel in wheels:
        for child in wheel.children_recursive:
            if child.type == 'MESH':
                child.data.materials.append(mat_wheel)

    # Camera material (yellow)
    mat_camera = bpy.data.materials.new(name="ScoutCamera")
    mat_camera.use_nodes = True
    mat_camera.node_tree.nodes["Principled BSDF"].inputs[0].default_value = (0.8, 0.8, 0.2, 1.0)
    camera.data.materials.append(mat_camera)

    # Mounting plate material (gray)
    mat_plate = bpy.data.materials.new(name="ScoutPlate")
    mat_plate.use_nodes = True
    mat_plate.node_tree.nodes["Principled BSDF"].inputs[0].default_value = (0.5, 0.5, 0.5, 1.0)
    plate.data.materials.append(mat_plate)

    # Frame view
    bpy.ops.object.select_all(action='SELECT')
    try:
        bpy.ops.view3d.view_all()
    except:
        pass  # Skip if in background mode

    print(f"Created Scout model with {len(wheels)} proper mecanum wheels")
    return body


if __name__ == "__main__":
    create_scout_with_mecanum_wheels()
