import logging

from miio import Discovery

logging.basicConfig(level=logging.INFO)

print("Discovering devices...")
devices = Discovery.discover_mdns()
print(f"Found {len(devices)} devices via mDNS")
for dev in devices:
    print(dev)

print("\nDiscovering via handshake...")
devices_hs = Discovery.discover_handshake()
print(f"Found {len(devices_hs)} devices via Handshake")
for dev in devices_hs:
    print(dev)
