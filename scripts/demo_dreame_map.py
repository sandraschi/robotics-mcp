#!/usr/bin/env python3
"""
Demonstrate Dreame D20 Pro LIDAR Map Retrieval and Visualization
"""

import asyncio
import json
import sys
import os
from pathlib import Path

# Add parent directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from robotics_mcp.tools.dreame_client import get_dreame_client

def print_map_summary(map_data: dict):
    """Print a human-readable summary of the map data."""
    print("\n" + "="*60)
    print("DREAME D20 PRO LIDAR MAP SUMMARY")
    print("="*60)

    print(f"Map ID: {map_data.get('map_id', 'Unknown')}")
    print(f"Total Rooms: {len(map_data.get('rooms', []))}")
    print(f"Total Obstacles: {len(map_data.get('obstacles', []))}")
    print(f"Charging Station: {'Yes' if map_data.get('charging_station') else 'No'}")

    # Room details
    if map_data.get('rooms'):
        print(f"\n🏠 ROOM DETAILS:")
        for i, room in enumerate(map_data['rooms'], 1):
            coords = room.get('coordinates', [])
            if len(coords) >= 4:
                width = abs(coords[2] - coords[0]) if len(coords) > 2 else 0
                height = abs(coords[3] - coords[1]) if len(coords) > 3 else 0
                area_m2 = (width * height) / 1000000  # Convert mm² to m²
                print(f"  Room {i}: {room.get('name', f'Room {i}')} - {area_m2:.1f} m²")
            else:
                print(f"  Room {i}: {room.get('name', f'Room {i}')} - Coordinates unavailable")

    # Obstacle details
    if map_data.get('obstacles'):
        print(f"\n🚧 OBSTACLE DETAILS:")
        for i, obstacle in enumerate(map_data['obstacles'][:10], 1):  # Show first 10
            x = obstacle.get('x', 'Unknown')
            y = obstacle.get('y', 'Unknown')
            size = obstacle.get('size', 'Unknown')
            print(f"  Obstacle {i}: Position ({x}, {y}) mm, Size: {size} mm")

        if len(map_data['obstacles']) > 10:
            print(f"  ... and {len(map_data['obstacles']) - 10} more obstacles")

    # Charging station
    if map_data.get('charging_station'):
        station = map_data['charging_station']
        x = station.get('x', 'Unknown')
        y = station.get('y', 'Unknown')
        print(f"\n🔋 CHARGING STATION:")
        print(f"  Position: ({x}, {y}) mm")

    print(f"\n💾 RAW MAP DATA STRUCTURE:")
    print(f"  Keys: {list(map_data.keys())}")
    if map_data.get('rooms'):
        print(f"  Room data keys: {list(map_data['rooms'][0].keys()) if map_data['rooms'] else 'None'}")
    if map_data.get('obstacles'):
        print(f"  Obstacle data keys: {list(map_data['obstacles'][0].keys()) if map_data['obstacles'] else 'None'}")

async def main():
    """Main demonstration function."""
    print("🤖 Dreame D20 Pro LIDAR Map Demo")
    print("="*50)

    if len(sys.argv) < 2:
        print("Usage: python demo_dreame_map.py <robot_id> [save_json]")
        print("Example: python demo_dreame_map.py dreame_01 true")
        return

    robot_id = sys.argv[1]
    save_json = len(sys.argv) > 2 and sys.argv[2].lower() == 'true'

    print(f"Robot ID: {robot_id}")
    print(f"Save JSON: {save_json}")
    print()

    try:
        # Get Dreame client
        print("🔌 Connecting to Dreame robot...")
        client = get_dreame_client(robot_id)

        # Retrieve map data
        print("🗺️  Retrieving LIDAR map...")
        map_data = await client.get_map()

        if not map_data:
            print("❌ Failed to retrieve map data")
            print("Possible issues:")
            print("- Robot not connected or configured")
            print("- Robot hasn't created a map yet")
            print("- Authentication/token issues")
            return

        # Display summary
        print_map_summary(map_data)

        # Save to JSON if requested
        if save_json:
            output_file = f"dreame_map_{robot_id}.json"
            with open(output_file, 'w') as f:
                json.dump(map_data, f, indent=2)
            print(f"\n💾 Map data saved to: {output_file}")

        # Usage examples
        print(f"\n🚀 USAGE EXAMPLES:")
        print("# Use map data with Yahboom robot navigation")
        print("await robot_control(robot_id='yahboom_01', action='navigate_to', x=2000, y=1500)")
        print()
        print("# Export map for ROS navigation")
        print("python scripts/export_dreame_map.py dreame_01 ./ros_maps")
        print()
        print("# Import into Unity for virtual testing")
        print("await vbot_crud(operation='create', robot_type='scout', platform='unity', environment_map='dreame_map.json')")

        print(f"\n✅ Demo completed successfully!")

    except Exception as e:
        print(f"❌ Demo failed: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    asyncio.run(main())