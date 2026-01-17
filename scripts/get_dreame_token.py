#!/usr/bin/env python3
"""
Dreame D20 Pro Token Extraction Script
No Android device required - uses network analysis and Xiaomi cloud APIs
"""

import asyncio
import json
import socket
import struct
import time
from typing import Optional, Dict, Any
import requests
import sys
import os

# Add parent directory to path for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

def discover_dreame_robots() -> list:
    """Discover Dreame robots on the local network using UDP broadcast."""
    print("🔍 Discovering Dreame robots on local network...")

    # Xiaomi MiIO discovery packet
    discovery_packet = b'\x21\x31\x00\x20\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff'

    # Create UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.settimeout(5)

    robots = []

    try:
        # Send discovery broadcast
        sock.sendto(discovery_packet, ('255.255.255.255', 54321))

        while True:
            try:
                data, addr = sock.recvfrom(1024)
                ip = addr[0]

                # Parse response (basic check for Dreame devices)
                if len(data) > 32:
                    # Extract device info from response
                    device_info = {
                        'ip': ip,
                        'raw_data': data.hex(),
                        'timestamp': time.time()
                    }
                    robots.append(device_info)
                    print(f"📡 Found device at {ip}")

            except socket.timeout:
                break

    except Exception as e:
        print(f"❌ Discovery error: {e}")
    finally:
        sock.close()

    return robots

def extract_token_from_cloud(username: str, password: str, country: str = "us") -> Optional[Dict[str, Any]]:
    """Extract Dreame token from Xiaomi cloud API."""
    print(f"☁️  Attempting cloud extraction for {username}...")

    # Xiaomi cloud login endpoint
    login_url = f"https://{country}.api.io.mi.com/app/user/login"

    headers = {
        'User-Agent': 'Dreamehome/1.0.0',
        'Content-Type': 'application/x-www-form-urlencoded'
    }

    data = {
        'username': username,
        'password': password,
        'sid': 'dreamehome'
    }

    try:
        response = requests.post(login_url, headers=headers, data=data, timeout=10)

        if response.status_code == 200:
            result = response.json()
            if result.get('code') == 0:
                user_id = result['result']['user_id']
                token = result['result']['token']

                print(f"✅ Cloud login successful!")
                print(f"   User ID: {user_id}")
                print(f"   Token: {token}")

                return {
                    'user_id': user_id,
                    'token': token,
                    'country': country
                }

    except Exception as e:
        print(f"❌ Cloud extraction failed: {e}")

    return None

def try_common_tokens(robot_ip: str) -> Optional[str]:
    """Try common default tokens for Dreame robots."""
    print(f"🔐 Trying common tokens for {robot_ip}...")

    common_tokens = [
        "00000000000000000000000000000000",  # All zeros
        "FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF",  # All F
        "12345678901234567890123456789012",  # Sequential
    ]

    from miio import DreameVacuumMiot

    for token in common_tokens:
        try:
            print(f"   Trying token: {token[:8]}...")
            device = DreameVacuumMiot(robot_ip, token)

            # Try to get status (this will fail with wrong token)
            status = device.status()
            if status:
                print(f"✅ Token works: {token}")
                return token

        except Exception:
            continue  # Token doesn't work

    print("❌ No common tokens worked")
    return None

async def main():
    """Main token extraction workflow."""
    print("🤖 Dreame D20 Pro Token Extraction Tool")
    print("=" * 50)

    # Step 1: Network discovery
    robots = discover_dreame_robots()

    if not robots:
        print("❌ No Dreame robots found on network")
        print("💡 Make sure your robot is powered on and connected to the same network")
        return

    print(f"📋 Found {len(robots)} potential devices")

    # Step 2: Try cloud extraction if user provides credentials
    print("\n" + "=" * 50)
    print("🌐 Option 1: Cloud Token Extraction")
    print("   (Requires Dreamehome account credentials)")

    use_cloud = input("Do you have Dreamehome account credentials? (y/n): ").lower().strip()

    if use_cloud == 'y':
        username = input("Enter Dreamehome email/phone: ").strip()
        password = input("Enter Dreamehome password: ").strip()
        country = input("Enter country code (us/cn/eu etc, default 'us'): ").strip() or "us"

        cloud_data = extract_token_from_cloud(username, password, country)

        if cloud_data:
            print("✅ Cloud extraction successful!")
            print(f"   Token: {cloud_data['token']}")
            return

    # Step 3: Try common tokens
    print("\n" + "=" * 50)
    print("🔑 Option 2: Common Token Testing")

    for robot in robots:
        print(f"\nTesting robot at {robot['ip']}:")
        token = try_common_tokens(robot['ip'])

        if token:
            print(f"🎉 SUCCESS! Robot at {robot['ip']} uses token: {token}")
            print("\n📝 Add this to your Robotics MCP config:")
            print(f"""
robotics:
  dreame_d20_pro:
    enabled: true
    robot_id: "dreame_01"
    robot_type: "dreame"
    ip_address: "{robot['ip']}"
    token: "{token}"
    mock_mode: false
""")
            return

    # Step 4: Manual entry
    print("\n" + "=" * 50)
    print("📝 Option 3: Manual Configuration")
    print("   If you can find the token elsewhere, enter it manually:")

    manual_ip = input("Enter robot IP address: ").strip()
    manual_token = input("Enter 32-character token (or press Enter to skip): ").strip()

    if manual_token and len(manual_token) == 32:
        print("✅ Manual configuration complete!")
        print(f"   IP: {manual_ip}")
        print(f"   Token: {manual_token}")
        print("\n📝 Add this to your Robotics MCP config:")
        print(f"""
robotics:
  dreame_d20_pro:
    enabled: true
    robot_id: "dreame_01"
    robot_type: "dreame"
    ip_address: "{manual_ip}"
    token: "{manual_token}"
    mock_mode: false
""")
    else:
        print("❌ No valid configuration found")
        print("\n🔍 Troubleshooting tips:")
        print("1. Make sure your Dreame robot is powered on")
        print("2. Ensure it's connected to the same WiFi network")
        print("3. Try power cycling the robot")
        print("4. Check your router's connected devices list for the robot's IP")
        print("5. Consider using a network sniffer like Wireshark during robot setup")

if __name__ == "__main__":
    asyncio.run(main())