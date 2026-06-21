#!/usr/bin/env python3
"""
Simple Dreame token test for IP 192.168.0.178
"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "src"))

from miio import DreameVacuum

# Test Dreame robot at 192.168.0.178
robot_ip = "192.168.0.178"
common_tokens = [
    "00000000000000000000000000000000",
    "FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF",
    "12345678901234567890123456789012",
]

print(f"Testing Dreame robot at {robot_ip}...")
for token in common_tokens:
    try:
        print(f"Trying token: {token[:8]}...")
        device = DreameVacuum(robot_ip, token)
        status = device.status()
        if status:
            print(f"SUCCESS! Token {token} works!")
            print("Update config.yaml with:")
            print(f'  ip_address: "{robot_ip}"')
            print(f'  token: "{token}"')
            print("  mock_mode: false")
            sys.exit(0)
    except Exception as e:
        print(f"Token {token[:8]} failed: {str(e)[:50]}...")
else:
    print("No common tokens worked. Try manual methods.")
    print("You can try:")
    print("1. Wireshark capture during app setup")
    print("2. Android backup extraction")
    print("3. Xiaomi cloud API if you have Dreamehome credentials")
