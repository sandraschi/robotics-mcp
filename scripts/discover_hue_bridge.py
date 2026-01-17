#!/usr/bin/env python3
"""
Philips Hue Bridge Discovery Script
Finds Hue Bridge Pro devices on your local network
"""

import socket
import json
import time
import requests
import sys
from typing import List, Dict, Any

def discover_hue_bridges(timeout: int = 10) -> List[Dict[str, Any]]:
    """Discover Philips Hue bridges using SSDP (Simple Service Discovery Protocol)."""
    print("Discovering Philips Hue bridges...")
    print(f"Timeout: {timeout} seconds")
    print("Make sure your Hue Bridge Pro is powered on and connected to the network")
    print()

    # SSDP M-SEARCH message for Hue bridges
    ssdp_request = (
        'M-SEARCH * HTTP/1.1\r\n'
        'HOST: 239.255.255.250:1900\r\n'
        'MAN: "ssdp:discover"\r\n'
        'MX: 10\r\n'
        'ST: ssdp:all\r\n'
        'USER-AGENT: Python/3.8 UPnP/1.1\r\n'
        '\r\n'
    )

    # Create UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 2)
    sock.settimeout(timeout)

    bridges = []

    try:
        # Send SSDP discovery message
        sock.sendto(ssdp_request.encode(), ('239.255.255.250', 1900))

        start_time = time.time()

        while time.time() - start_time < timeout:
            try:
                data, addr = sock.recvfrom(4096)
                response = data.decode('utf-8', errors='ignore')

                # Check if this is a Hue bridge response
                if 'hue' in response.lower() or 'philips' in response.lower():
                    bridge_info = parse_ssdp_response(response, addr[0])
                    if bridge_info and bridge_info not in bridges:
                        bridges.append(bridge_info)
                        print(f"FOUND: Hue Bridge at {addr[0]}")
                        print(f"  Model: {bridge_info.get('model', 'Unknown')}")
                        print(f"  Serial: {bridge_info.get('serial', 'Unknown')}")
                        print()

            except socket.timeout:
                continue

    except Exception as e:
        print(f"Discovery error: {e}")
    finally:
        sock.close()

    return bridges

def parse_ssdp_response(response: str, ip: str) -> Dict[str, Any]:
    """Parse SSDP response to extract bridge information."""
    lines = response.split('\n')
    bridge_info = {'ip': ip}

    for line in lines:
        line = line.strip()
        if ':' in line:
            key, value = line.split(':', 1)
            key = key.strip().lower()
            value = value.strip()

            if 'server' in key and 'hue' in value.lower():
                bridge_info['model'] = 'Hue Bridge'
                if 'pro' in value.lower():
                    bridge_info['model'] = 'Hue Bridge Pro'
            elif 'usn' in key:
                # Extract serial number from USN
                if 'uuid:' in value:
                    uuid_part = value.split('uuid:')[1].split('-')[0]
                    bridge_info['serial'] = uuid_part

    return bridge_info

def test_bridge_connection(ip: str) -> Dict[str, Any]:
    """Test connection to a discovered bridge and get detailed information."""
    print(f"Testing connection to {ip}...")

    try:
        # Try to get bridge configuration
        url = f"http://{ip}/api/config"
        response = requests.get(url, timeout=5)

        if response.status_code == 200:
            config = response.json()

            bridge_details = {
                'ip': ip,
                'name': config.get('name', 'Unknown'),
                'model': config.get('modelid', 'Unknown'),
                'sw_version': config.get('swversion', 'Unknown'),
                'api_version': config.get('apiversion', 'Unknown'),
                'bridge_id': config.get('bridgeid', 'Unknown'),
                'connected': True,
                'homeaware_capable': 'pro' in config.get('modelid', '').lower()
            }

            print("SUCCESS: Bridge details retrieved")
            print(f"  Name: {bridge_details['name']}")
            print(f"  Model: {bridge_details['model']}")
            print(f"  Software: {bridge_details['sw_version']}")
            print(f"  HomeAware: {'Yes' if bridge_details['homeaware_capable'] else 'No'}")
            print()

            return bridge_details

    except Exception as e:
        print(f"Connection test failed: {e}")
        return {'ip': ip, 'connected': False, 'error': str(e)}

def main():
    """Main discovery function."""
    print("Philips Hue Bridge Discovery Tool")
    print("=" * 40)
    print()

    # Discover bridges
    bridges = discover_hue_bridges()

    if not bridges:
        print("No Hue bridges found")
        print("\nTroubleshooting:")
        print("1. Ensure your Hue Bridge Pro is powered on")
        print("2. Make sure it's connected to your router via Ethernet")
        print("3. Wait 1-2 minutes after powering on")
        print("4. Try running this script closer to your router")
        print("5. Check if your firewall blocks SSDP (UDP port 1900)")
        print("\nAlternative methods:")
        print("- Check your router's admin page for connected devices")
        print("- Look for device names containing 'hue' or 'philips'")
        return

    print(f"Found {len(bridges)} potential bridge(s)")
    print()

    # Test connections and get details
    detailed_bridges = []
    for bridge in bridges:
        details = test_bridge_connection(bridge['ip'])
        detailed_bridges.append(details)

    # Show results
    print("DISCOVERY RESULTS")
    print("=" * 40)

    for i, bridge in enumerate(detailed_bridges, 1):
        print(f"\nBridge #{i}:")
        print(f"  IP Address: {bridge['ip']}")

        if bridge.get('connected'):
            print(f"  Name: {bridge.get('name', 'Unknown')}")
            print(f"  Model: {bridge.get('model', 'Unknown')}")
            print(f"  Software Version: {bridge.get('sw_version', 'Unknown')}")
            print(f"  HomeAware Capable: {'Yes' if bridge.get('homeaware_capable') else 'No'}")
            print(f"  Bridge ID: {bridge.get('bridge_id', 'Unknown')}")

            print("\n  CONFIGURATION FOR ROBOTICS MCP:")
            print(f"  Add this to your config.yaml:")
            print(f"""
  hue_bridge_pro:
    enabled: true
    robot_id: "hue_01"
    robot_type: "hue"
    ip_address: "{bridge['ip']}"
    # API key will be generated on first connection
""")
        else:
            print("  Status: Connection failed"
            print(f"  Error: {bridge.get('error', 'Unknown')}")

    print(f"\n{'='*40}")
    print("NEXT STEPS:")
    print("1. Add the configuration above to your Robotics MCP config.yaml")
    print("2. Run the MCP server")
    print("3. Test with: python -c \"from robotics_mcp.tools.robot_control import robot_control; import asyncio; asyncio.run(robot_control(robot_id='hue_01', action='hue_get_sensor_status'))\"")
    print("4. Enable HomeAware in the Philips Hue app and wait 24-48 hours for calibration")

if __name__ == "__main__":
    main()