import hashlib
import sys

import requests


def get_md5(s):
    return hashlib.md5(s.encode("utf-8")).hexdigest()


def dreame_login(username, password):
    url = "https://io.dreame.tech/api/v2/auth/login"  # Common endpoint for global

    # Dreame Cloud hashing (simplified based on Tasshack logic)
    # They often use a specific app_id and custom hashing
    print(f"Attempting Dreame Cloud login for {username}...")

    headers = {
        "User-Agent": "Dreamehome/1.5.87 (iPhone; iOS 16.0; Scale/3.00)",
        "Content-Type": "application/json",
        "App-Id": "dreamehome_ios",
    }

    payload = {
        "username": username,
        "password": get_md5(password),  # Dreame usually MD5s the password
        "app_id": "dreamehome_ios",
        "region": "de",  # User is in Vienna
    }

    try:
        response = requests.post(url, json=payload, headers=headers, timeout=10)
        print(f"Status Code: {response.status_code}")
        if response.status_code == 200:
            print("Login Successful!")
            return response.json()
        else:
            print(f"Error Response: {response.text}")
            return None
    except Exception as e:
        print(f"Request failed: {e}")
        return None


if __name__ == "__main__":
    import sys

    if len(sys.argv) < 3:
        print("Usage: python dreame_auth.py <email> <password>")
    else:
        dreame_login(sys.argv[1], sys.argv[2])
