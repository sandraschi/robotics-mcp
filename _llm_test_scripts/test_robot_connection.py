#!/usr/bin/env python3
"""
Test connection to Dreame robot at 192.168.0.178
"""

import socket

robot_ip = "192.168.0.178"

# Test different ports that Xiaomi/Dreame devices might use
ports_to_test = [54321, 80, 443, 8888, 9898, 6053]

print(f"Testing TCP connections to {robot_ip}...")

for port in ports_to_test:
    try:
        print(f"Testing port {port}...")
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(3)
        result = sock.connect_ex((robot_ip, port))
        sock.close()

        if result == 0:
            print(f"SUCCESS: Port {port} is open!")
            break
        else:
            print(f"Port {port} closed")

    except Exception as e:
        print(f"Port {port} error: {e}")

print("\nTesting UDP discovery on multiple ports...")
for port in [54321, 2425, 5353, 7000]:
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.settimeout(2)
        discovery_packet = b"\x21\x31\x00\x20\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff"

        sock.sendto(discovery_packet, (robot_ip, port))
        data, addr = sock.recvfrom(1024)

        print(f"UDP SUCCESS on port {port}: {len(data)} bytes from {addr}")
        print(f"Response: {data.hex()[:50]}...")
        sock.close()
        break

    except TimeoutError:
        print(f"UDP timeout on port {port}")
    except Exception as e:
        print(f"UDP error on port {port}: {e}")

# Test direct miio connection with a common token
print("\nTesting miio connection...")
try:
    from miio import DreameVacuum

    device = DreameVacuum(robot_ip, "00000000000000000000000000000000")
    print("Attempting to get status...")
    status = device.status()
    print(f"SUCCESS! Status: {status}")
except Exception as e:
    print(f"miio connection failed: {e}")

print("\nDone testing.")
