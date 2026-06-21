#!/usr/bin/env python3
"""
Discover all Xiaomi devices on the local network
"""

import sys

sys.path.insert(0, "src")

from miio import Discovery

print("Scanning for Xiaomi devices on local network...")
print("This will find ALL Xiaomi devices, not just Dreame...")
print()

try:
    devices = Discovery.discover_mdns(timeout=10)

    print(f"Found {len(devices)} Xiaomi devices:")

    for device in devices:
        print(f"\nDevice: {device}")
        print(f"  IP: {device.ip}")
        print(f"  Token: {device.token}")
        print(f"  Model: {getattr(device, 'model', 'unknown')}")

        # Check if it's a Dreame device
        if hasattr(device, "model") and "dreame" in device.model.lower():
            print("  *** FOUND DREAME DEVICE! ***")

    if len(devices) == 0:
        print("No Xiaomi devices found.")
        print("This could mean:")
        print("- Devices are not on the same network")
        print("- Devices are in cloud-only mode")
        print("- Firewall blocking discovery")
        print("- Devices require different discovery method")

except Exception as e:
    print(f"Discovery error: {e}")
    print("This might mean no devices found, or network issues.")
