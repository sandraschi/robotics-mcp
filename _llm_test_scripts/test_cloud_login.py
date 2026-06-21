#!/usr/bin/env python3
"""
Test Dreame cloud login
"""

import requests


def test_dreame_cloud_login(username=None, password=None):
    """Test Dreame cloud login with provided credentials"""

    if not username or not password:
        print("No credentials provided. To test cloud login:")
        print("1. Get your Dreamehome account email and password")
        print("2. Run: python test_cloud_login.py your-email@example.com your-password")
        return None

    # Try different regions
    regions = ["de", "us", "sg", "ru"]
    sids = ["dreamehome", "xiaomiio"]

    for country in regions:
        for sid in sids:
            print(f"\n--- Testing region: {country.upper()} | SID: {sid} ---")
            login_url = f"https://{country}.api.io.mi.com/app/user/login"
            headers = {
                "User-Agent": "Dreamehome/1.0.0",
                "Content-Type": "application/x-www-form-urlencoded",
            }
            data = {"username": username, "password": password, "sid": sid}

            try:
                print(f"Attempting cloud login for {username}...")
                response = requests.post(login_url, headers=headers, data=data, timeout=10)
                print(f"Response status: {response.status_code}")

                if response.status_code == 200:
                    result = response.json()
                    if result.get("code") == 0:
                        print(f"SUCCESS! Cloud login worked in {country.upper()} with SID {sid}")
                        token = result["result"]["token"]
                        user_id = result["result"]["user_id"]
                        print(f"User ID: {user_id}")
                        print(f"Token: {token}")
                        return result["result"]
                    else:
                        print(f"Login failed with code {result.get('code')}: {result.get('message', 'No message')}")
                elif response.status_code == 401:
                    print("Unauthorized. Possible password error, wrong region, or Captcha required.")

            except Exception as e:
                print(f"Cloud login error for {country}/{sid}: {e}")

    return None


if __name__ == "__main__":
    import sys

    if len(sys.argv) >= 3:
        username = sys.argv[1]
        password = sys.argv[2]
        test_dreame_cloud_login(username, password)
    elif len(sys.argv) >= 2:
        username = sys.argv[1]
        print(f"Testing cloud login for: {username}")
        print("Please provide password as second argument:")
        print("Usage: python test_cloud_login.py email@example.com yourpassword")
    else:
        test_dreame_cloud_login()
