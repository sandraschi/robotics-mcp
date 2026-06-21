import argparse
import getpass
import json
import os
import sys

# Add Tasshack repo to sys.path
TASSHACK_REPO = r"D:\Dev\repos\tasshack_dreame_vacuum_ref"
if TASSHACK_REPO not in sys.path:
    sys.path.append(TASSHACK_REPO)

try:
    from custom_components.dreame_vacuum.dreame.protocol import DreameVacuumDreameHomeCloudProtocol
except ImportError as e:
    print(f"Error importing Dreame protocol: {e}")
    print(f"Verified path: {TASSHACK_REPO}")
    print(f"Contents of path: {os.listdir(TASSHACK_REPO)}")
    sys.exit(1)


def main():
    parser = argparse.ArgumentParser(description="Extract Dreame Device Token")
    parser.add_argument("--username", help="Dreamehome username/email")
    parser.add_argument("--password", help="Dreamehome password")
    parser.add_argument("--country", default="eu", help="Country code (default: eu)")
    args = parser.parse_args()

    username = args.username
    password = args.password

    if not username:
        username = input("Enter Dreamehome Username: ").strip()
    if not password:
        password = getpass.getpass("Enter Dreamehome Password: ").strip()

    print(f"Logging in as {username} (Region: {args.country})...")

    protocol = DreameVacuumDreameHomeCloudProtocol(
        username=username, password=password, country=args.country, account_type="dreame"
    )

    if protocol.login():
        print("\n✅ Login Successful!")
        print(f"Cloud Auth Key: {protocol.auth_key}")

        print("\nFetching devices...")
        devices = protocol.get_devices()

        if devices and "page" in devices and "records" in devices["page"]:
            records = devices["page"]["records"]
            print(f"Found {len(records)} devices.")

            for device in records:
                print("\n" + "=" * 40)
                name = device.get("customName") or device.get("deviceInfo", {}).get("displayName", "Unknown")
                print(f"Name:  {name}")
                print(f"Model: {device.get('model')}")
                print(f"DID:   {device.get('did')}")
                print(f"MAC:   {device.get('mac')}")

                # Check for token in various known locations
                token = device.get("token")
                if not token and "token" in device.get("deviceInfo", {}):
                    token = device["deviceInfo"]["token"]

                if token:
                    print(f"TOKEN: {token}")
                else:
                    print("TOKEN: [Not found in standard fields]")
                    # Print keys to help debug if token is missing
                    # print("Keys:", device.keys())
                print("=" * 40)
        else:
            print("No device records found in response.")
            print(json.dumps(devices, indent=2))

    else:
        print("❌ Login Failed.")


if __name__ == "__main__":
    main()
