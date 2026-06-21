#!/usr/bin/env python3
import sys

# Ensure we use the correct miio integration path
try:
    from miio.integrations.vacuum.dreame import DreameVacuum
except ImportError:
    print("Error: miio integration not found at expected path.")
    sys.exit(1)

robot_ip = "192.168.0.179"
# Common/Default tokens to try
tokens_to_try = [
    "00000000000000000000000000000000",
    "FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF",
    "12345678901234567890123456789012",
]

print(f"--- Dreame D20 Pro Probe (IP: {robot_ip}) ---")

for token in tokens_to_try:
    print(f"Testing token: {token[:8]}...")
    try:
        # dreame_vacuum_r2566a usually implies MIOT protocol
        device = DreameVacuum(robot_ip, token)
        status = device.status()
        if status:
            print("\n[SUCCESS] Connected to Dreame!")
            print(f"Token: {token}")
            print(f"Status: {status}")
            sys.exit(0)
    except Exception as e:
        # Check if it's a timeout or unauthorized
        err_msg = str(e)
        if "Unable to discover the device" in err_msg:
            print("  Failed: Device unreachable or discovery failed.")
            break  # No point testing other tokens if unreachable
        else:
            print(f"  Failed: {err_msg[:100]}")

print("\n[FAILURE] Local connection failed. Handshake requires valid token.")
print("Proceeding to Phase 16: Token Extraction required.")
sys.exit(1)
