#!/usr/bin/env python3
"""
Simple Dreame D20 Pro Network Discovery Script
No Android device required - finds your robot's IP address
"""

import socket
import time
import sys
import os

def discover_dreame_robots(timeout: int = 10) -> list:
    """Discover Dreame robots on the local network using UDP broadcast."""
    print("Scanning for Dreame D20 Pro robots...")
    print(f"   Timeout: {timeout} seconds")
    print("   Make sure your robot is powered on and connected to WiFi")
    print()

    # Xiaomi MiIO discovery packet (works for Dreame robots)
    discovery_packet = b'\x21\x31\x00\x20\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff'

    # Create UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.settimeout(timeout)

    robots = []

    try:
        print("Sending discovery broadcast...")
        # Send discovery broadcast multiple times
        for i in range(3):
            sock.sendto(discovery_packet, ('255.255.255.255', 54321))
            time.sleep(0.5)

        print("Listening for responses...")
        start_time = time.time()

        while time.time() - start_time < timeout:
            try:
                data, addr = sock.recvfrom(1024)
                ip = addr[0]

                # Basic validation - Dreame robots respond with specific packet structure
                if len(data) >= 32:
                    device_info = {
                        'ip': ip,
                        'port': addr[1],
                        'data_length': len(data),
                        'raw_data': data.hex()[:64] + '...' if len(data) > 32 else data.hex(),
                        'timestamp': time.time()
                    }
                    robots.append(device_info)
                    print(f"FOUND: Dreame robot at {ip}:{addr[1]}")

            except socket.timeout:
                continue

    except Exception as e:
        print(f"❌ Discovery error: {e}")
    finally:
        sock.close()

    return robots

def main():
    """Main discovery function."""
    print("Dreame D20 Pro Network Discovery Tool")
    print("=" * 50)

    robots = discover_dreame_robots()

    if not robots:
        print("\nNo Dreame robots found")
        print("\nTroubleshooting:")
        print("1. Ensure your Dreame D20 Pro is powered on")
        print("2. Make sure it's connected to your WiFi network")
        print("3. Wait 2-3 minutes after powering on for network connection")
        print("4. Try running this script closer to your router")
        print("5. Check if your firewall blocks UDP broadcasts")
        print("\nYou can also:")
        print("- Check your router's admin page for connected devices")
        print("- Look for device names like 'dreame-vacuum' or similar")
        print("- Try the full token extraction script: python get_dreame_token.py")
        return

    print(f"\nFound {len(robots)} Dreame robot(s):")
    print()

    for i, robot in enumerate(robots, 1):
        print(f"Robot #{i}:")
        print(f"   IP Address: {robot['ip']}")
        print(f"   Port: {robot['port']}")
        print(f"   Response size: {robot['data_length']} bytes")
        print()

        print("Add this to your Robotics MCP config:")
        print(f"""
robotics:
  dreame_d20_pro:
    enabled: true
    robot_id: "dreame_01"
    robot_type: "dreame"
    ip_address: "{robot['ip']}"
    token: "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX"  # You'll need to get the token
    mock_mode: false
""")

        print("To get the token, run: python scripts/get_dreame_token.py")
        print()

if __name__ == "__main__":
    main()