#!/usr/bin/env python3
"""
SOTA Dreamehome Token Extractor (2026)
Targets api.dreame.tech for Dreamehome-native accounts.
"""

import sys

import requests


def extract_dreamehome_token(email, password):
    print(f"🚀 Initializing SOTA extraction for {email}...")

    # Dreamehome API endpoints
    # Dreamehome API endpoints
    auth_url = "https://io.dreame.tech/api/v2/auth/login"
    devices_url = "https://io.dreame.tech/api/v2/device/list"

    headers = {
        "User-Agent": "Dreamehome/1.0.0 (iOS 17.2; iPhone 15 Pro)",
        "Content-Type": "application/json",
        "Accept": "application/json",
    }

    # Password hashing (Dreamehome often uses MD5 or similar for the initial handshake)
    # This is a simplified version; real app uses complex signing, but some
    # API endpoints allow simple auth for legacy integration.

    # Attempting OAuth-style or direct login
    login_payload = {
        "username": email,
        "password": password,
        "app_id": "dreamehome_official",
        "region": "europe",
    }

    try:
        print("Obtaining session token from Dreame Cloud...")
        response = requests.post(auth_url, json=login_payload, headers=headers, timeout=15)

        if response.status_code == 200:
            data = response.json()
            if data.get("code") == 0:
                print("✅ Login Successful!")
                access_token = data["data"]["access_token"]

                # Use access token to list devices
                print("Fetching device list...")
                headers["Authorization"] = f"Bearer {access_token}"
                devices_resp = requests.get(devices_url, headers=headers, timeout=15)

                if devices_resp.status_code == 200:
                    devices_data = devices_resp.json()
                    if devices_data.get("code") == 0 and devices_data.get("data"):
                        print(f"Found {len(devices_data['data'])} devices.")
                        for device in devices_data["data"]:
                            name = device.get("name", "Unknown")
                            model = device.get("model", "Unknown")
                            token = device.get("token", "N/A")
                            did = device.get("did", "N/A")
                            ip = device.get("ip", "N/A")

                            print("-" * 30)
                            print(f"Device: {name}")
                            print(f"Model: {model}")
                            print(f"DID: {did}")
                            print(f"IP: {ip}")
                            print(f"TOKEN: {token}")
                            print("-" * 30)

                            return token
                    else:
                        print("No devices found in this account.")
                else:
                    print(f"Failed to fetch devices: {devices_resp.status_code}")
            else:
                print(f"Login failed: {data.get('msg', 'Unknown Error')}")
        else:
            print(f"Auth request failed with status: {response.status_code}")
            if response.status_code == 401:
                print("Incorrect credentials or regional block.")

    except Exception as e:
        print(f"Unexpected error: {e}")

    return None


if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python get_dreamehome_sota_token.py email password")
        sys.exit(1)

    email = sys.argv[1]
    password = sys.argv[2]

    token = extract_dreamehome_token(email, password)
    if token:
        print(f"\n[SUCESS] Extracted Token: {token}")
    else:
        print("\n[FAILURE] extraction failed. Check credentials and region.")
