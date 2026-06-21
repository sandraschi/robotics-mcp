#!/usr/bin/env python3
import os
import sys

# Add src to path if needed
sys.path.insert(0, os.path.join(os.getcwd(), "src"))

try:
    from miio import DreameVacuumMiot
except ImportError:
    print("Error: python-miio not installed in this environment.")
    sys.exit(1)

robot_ip = "192.168.0.179"
common_tokens = [
    "00000000000000000000000000000000",
    "FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF",
    "12345678901234567890123456789012",
]

print(f"Testing direct connection to Dreame at {robot_ip}...")

for token in common_tokens:
    print(f"Testing token: {token[:8]}...")
    try:
        device = DreameVacuumMiot(robot_ip, token)
        # Try to get status to verify token
        status = device.status()
        if status:
            print("\n[SUCCESS] Connected to Dreame D20 Pro!")
            print(f"Token: {token}")
            print(f"Battery: {status.battery}%")
            print(f"State: {status.state}")
            sys.exit(0)
    except Exception as e:
        print(f"  Failed: {str(e)[:60]}")

print("\n[FAILURE] Could not connect using common tokens.")
print("The token is likely randomized during initial setup.")
print("Recommendation: Use a Xiaomi Cloud token extractor or check router for local keys.")
