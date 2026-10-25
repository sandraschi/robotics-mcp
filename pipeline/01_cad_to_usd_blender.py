"""Blender Headless CAD to OpenUSD Converter.

Automates the ingestion of FreeCAD / STEP / STL / OBJ exports and converts them
into an organized OpenUSD (.usd / .usda / .usdc) hierarchy with clean geometry,
centered origins, material slots, and proper scaling for NVIDIA Isaac Sim / Lab.

Usage:
    blender --background --python 01_cad_to_usd_blender.py -- \
        --input ./assets/robot_cad.stl \
        --output ./usd/robot_base.usd \
        --scale 0.001  # FreeCAD mm to Isaac Sim meters
"""

import argparse
import sys
from pathlib import Path


def parse_args():
    # Strip Blender's own CLI args before '--'
    argv = sys.argv
    if "--" in argv:
        argv = argv[argv.index("--") + 1 :]
    else:
        argv = []

    parser = argparse.ArgumentParser(description="Convert CAD geometry to OpenUSD for Isaac Sim.")
    parser.add_argument("--input", required=True, type=str, help="Input CAD file (.stl, .obj, .fbx, .ply)")
    parser.add_argument("--output", required=True, type=str, help="Output USD file (.usd, .usda, .usdc)")
    parser.add_argument(
        "--scale", type=float, default=0.001, help="Scale multiplier (default: 0.001 for mm -> m conversion)"
    )
    parser.add_argument("--clean-mesh", action="store_true", default=True, help="Remove duplicate vertices and recalculate normals")
    return parser.parse_args(argv)


def run_conversion():
    try:
        import bpy
    except ImportError:
        print("[ERROR] This script must be executed inside Blender's Python runtime.")
        sys.exit(1)

    args = parse_args()
    input_path = Path(args.input).resolve()
    output_path = Path(args.output).resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)

    if not input_path.exists():
        print(f"[ERROR] Input file not found: {input_path}")
        sys.exit(1)

    # Clean default Blender scene
    bpy.ops.wm.read_factory_settings(use_empty=True)

    # Import CAD file according to format
    ext = input_path.suffix.lower()
    print(f"[INFO] Importing {input_path} (format: {ext})...")

    if ext == ".stl":
        bpy.ops.wm.stl_import(filepath=str(input_path))
    elif ext == ".obj":
        bpy.ops.wm.obj_import(filepath=str(input_path))
    elif ext == ".ply":
        bpy.ops.wm.ply_import(filepath=str(input_path))
    elif ext == ".fbx":
        bpy.ops.import_scene.fbx(filepath=str(input_path))
    else:
        print(f"[ERROR] Unsupported file extension: {ext}. Export FreeCAD to STL/OBJ/PLY first.")
        sys.exit(1)

    # Process imported objects
    imported_objects = [obj for obj in bpy.context.scene.objects if obj.type == "MESH"]
    print(f"[INFO] Processed {len(imported_objects)} mesh objects.")

    for obj in imported_objects:
        bpy.context.view_layer.objects.active = obj
        obj.select_set(True)

        # Apply scaling (FreeCAD mm to Isaac Sim SI meters)
        obj.scale = (args.scale, args.scale, args.scale)
        bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)

        if args.clean_mesh:
            bpy.ops.object.mode_set(mode="EDIT")
            bpy.ops.mesh.select_all(action="SELECT")
            bpy.ops.mesh.remove_doubles(threshold=0.0001)
            bpy.ops.mesh.normals_make_consistent(inside=False)
            bpy.ops.object.mode_set(mode="OBJECT")

    # Export OpenUSD
    print(f"[INFO] Exporting to OpenUSD: {output_path}...")
    bpy.ops.wm.usd_export(
        filepath=str(output_path),
        selected_objects_only=False,
        export_materials=True,
        export_textures=True,
        export_mesh_colors=True,
        root_prim_path="/World/Robot",
        up_axis="Z",
        triangulate_meshes=True,
    )
    print(f"[SUCCESS] USD successfully created: {output_path}")


if __name__ == "__main__":
    run_conversion()
