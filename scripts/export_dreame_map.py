#!/usr/bin/env python3
"""
Export Dreame D20 Pro LIDAR Map Data
Convert vacuum maps to formats usable by other robots
"""

import asyncio
import json
import sys
import os
from typing import Dict, Any, List
from pathlib import Path

# Add parent directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from robotics_mcp.tools.dreame_client import get_dreame_client

def convert_to_occupancy_grid(map_data: Dict[str, Any], resolution: float = 0.05) -> Dict[str, Any]:
    """Convert Dreame map to ROS occupancy grid format."""
    # This is a simplified conversion - real implementation would need proper grid generation
    width = 200  # 10m at 5cm resolution
    height = 200  # 10m at 5cm resolution

    # Create simple occupancy grid (0=free, 100=occupied, -1=unknown)
    data = [-1] * (width * height)  # Start with all unknown

    # Mark rooms as free space
    if map_data.get('rooms'):
        for room in map_data['rooms']:
            if room.get('coordinates'):
                # Simplified: mark room areas as free
                # Real implementation would need proper polygon filling
                pass

    # Mark obstacles as occupied
    if map_data.get('obstacles'):
        for obstacle in map_data['obstacles']:
            if obstacle.get('x') is not None and obstacle.get('y') is not None:
                # Convert coordinates to grid indices
                grid_x = int(obstacle['x'] * 20)  # 5cm resolution
                grid_y = int(obstacle['y'] * 20)
                if 0 <= grid_x < width and 0 <= grid_y < height:
                    index = grid_y * width + grid_x
                    if index < len(data):
                        data[index] = 100  # Occupied

    return {
        "header": {
            "frame_id": "map"
        },
        "info": {
            "resolution": resolution,
            "width": width,
            "height": height,
            "origin": {
                "position": {"x": -5.0, "y": -5.0, "z": 0.0},
                "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
            }
        },
        "data": data
    }

def convert_to_yaml_navigation(map_data: Dict[str, Any]) -> str:
    """Convert to YAML format for ROS navigation."""
    yaml_content = f"""# Dreame D20 Pro Map Export
# Generated for ROS Navigation Stack

image: dreame_map.pgm
resolution: 0.050000
origin: [-5.000000, -5.000000, 0.000000]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196

# Map metadata
rooms: {len(map_data.get('rooms', []))}
obstacles: {len(map_data.get('obstacles', []))}
charging_station: {map_data.get('charging_station') is not None}
"""

    return yaml_content

def export_to_unity_navmesh(map_data: Dict[str, Any]) -> Dict[str, Any]:
    """Convert to format usable by Unity NavMesh."""
    vertices = []
    triangles = []

    # Convert rooms to Unity mesh format
    vertex_index = 0
    if map_data.get('rooms'):
        for room in map_data['rooms']:
            if room.get('coordinates') and len(room['coordinates']) >= 6:
                # Convert coordinates to Unity space (flip Y, scale appropriately)
                room_vertices = []
                for i in range(0, len(room['coordinates']), 2):
                    x = room['coordinates'][i] * 0.001  # mm to meters
                    z = room['coordinates'][i + 1] * 0.001  # Y becomes Z in Unity
                    vertices.append({"x": x, "y": 0.0, "z": z})
                    room_vertices.append(vertex_index)
                    vertex_index += 1

                # Create triangles (simple triangulation)
                if len(room_vertices) >= 3:
                    for i in range(1, len(room_vertices) - 1):
                        triangles.extend([room_vertices[0], room_vertices[i], room_vertices[i + 1]])

    return {
        "vertices": vertices,
        "triangles": triangles,
        "metadata": {
            "source": "Dreame D20 Pro",
            "rooms": len(map_data.get('rooms', [])),
            "obstacles": len(map_data.get('obstacles', []))
        }
    }

async def main():
    """Main export function."""
    if len(sys.argv) < 2:
        print("Usage: python export_dreame_map.py <robot_id> [output_dir]")
        print("Example: python export_dreame_map.py dreame_01 ./maps")
        return

    robot_id = sys.argv[1]
    output_dir = sys.argv[2] if len(sys.argv) > 2 else "./dreame_maps"

    # Create output directory
    Path(output_dir).mkdir(exist_ok=True)

    print(f"Exporting Dreame map for robot: {robot_id}")
    print(f"Output directory: {output_dir}")

    try:
        # Get Dreame client and fetch map
        client = get_dreame_client(robot_id)
        map_data = await client.get_map()

        if not map_data:
            print("❌ Failed to retrieve map data")
            return

        print("✅ Map data retrieved successfully")
        print(f"   Rooms: {len(map_data.get('rooms', []))}")
        print(f"   Obstacles: {len(map_data.get('obstacles', []))}")
        print(f"   Charging station: {map_data.get('charging_station') is not None}")

        # Export to different formats
        base_filename = f"{output_dir}/dreame_{robot_id}_{map_data.get('map_id', 'unknown')}"

        # 1. JSON format (raw data)
        with open(f"{base_filename}.json", 'w') as f:
            json.dump(map_data, f, indent=2)
        print(f"📄 Exported raw JSON: {base_filename}.json")

        # 2. ROS occupancy grid
        occupancy_grid = convert_to_occupancy_grid(map_data)
        with open(f"{base_filename}_occupancy.json", 'w') as f:
            json.dump(occupancy_grid, f, indent=2)
        print(f"🗺️  Exported ROS occupancy grid: {base_filename}_occupancy.json")

        # 3. ROS YAML map file
        yaml_content = convert_to_yaml_navigation(map_data)
        with open(f"{base_filename}.yaml", 'w') as f:
            f.write(yaml_content)
        print(f"📋 Exported ROS YAML: {base_filename}.yaml")

        # 4. Unity NavMesh format
        unity_navmesh = export_to_unity_navmesh(map_data)
        with open(f"{base_filename}_unity.json", 'w') as f:
            json.dump(unity_navmesh, f, indent=2)
        print(f"🎮 Exported Unity NavMesh: {base_filename}_unity.json")

        print("\n🎉 Map export complete!")
        print(f"All files saved to: {output_dir}")
        print("\nUsage:")
        print("- JSON files: Direct use in applications")
        print("- ROS YAML: Use with ROS navigation stack")
        print("- Unity JSON: Import into Unity for NavMesh generation")
        print("- Occupancy grid: Use with path planning algorithms")

    except Exception as e:
        print(f"❌ Export failed: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    asyncio.run(main())