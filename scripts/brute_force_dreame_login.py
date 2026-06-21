import hashlib
import json
import socket

import requests

CREDENTIALS = {
    "username": "sandraschipal@hotmail.com",
    "password": "Sec1000dr#",
    "region": "de",
    "country_code": "DE",
}

ENDPOINTS = [
    # Tasshack mentions
    "https://cn.iot.dreame.tech:13267/dreame-auth/oauth/token",
    "https://cn.iot.dreame.tech/dreame-auth/oauth/token",
    # Regional variants
    "https://de.iot.dreame.tech/dreame-auth/oauth/token",
    "https://eu.iot.dreame.tech/dreame-auth/oauth/token",
    "https://iot.dreame.tech/dreame-auth/oauth/token",
    # Old API variants
    "https://api.dreame.tech/login",
    "https://io.dreame.tech/api/v2/auth/login",
    "https://dreame-de.iot.dreame.tech/api/v2/auth/login",
    # Misc guesses
    "https://auth.dreame.tech/login",
    "https://account.dreame.tech/login",
    "https://app-service.dreame.tech/v2/auth/login",
]


def get_md5(s):
    return hashlib.md5(s.encode("utf-8")).hexdigest()


def test_endpoint(url):
    print(f"Testing {url}...")

    # Check DNS first
    domain = url.split("/")[2].split(":")[0]
    try:
        socket.gethostbyname(domain)
    except socket.gaierror:
        print(f"  ❌ DNS Failed for {domain}")
        return False

    password_md5 = get_md5(CREDENTIALS["password"])

    # Payload 1: Legacy Login
    payload_legacy = {
        "username": CREDENTIALS["username"],
        "password": password_md5,
        "app_id": "dreamehome_ios",
        "region": CREDENTIALS["region"],
    }

    # Payload 2: OAuth style (Tasshack)
    payload_oauth = {
        "grant_type": "password",
        "client_id": "dreame-app",
        "username": CREDENTIALS["username"],
        "password": password_md5,  # Sometimes raw password?
        "scope": "all",
    }

    headers = {"User-Agent": "Dreamehome/1.0.0 (iOS)", "Content-Type": "application/json"}

    try:
        # Try Legacy
        resp = requests.post(url, json=payload_legacy, headers=headers, timeout=5)
        if resp.status_code == 200 and "code" in resp.json() and resp.json()["code"] == 0:
            print(f"  ✅ SUCCESS (Legacy)! Response: {resp.text[:100]}...")
            return resp.json()
        elif resp.status_code != 404:
            print(f"  ⚠️  Legacy: {resp.status_code} - {resp.text[:50]}")

        # Try OAuth
        resp_oauth = requests.post(url, data=payload_oauth, timeout=5)  # OAuth usually form-data
        if resp_oauth.status_code == 200:
            print(f"  ✅ SUCCESS (OAuth)! Response: {resp_oauth.text[:100]}...")
            return resp_oauth.json()

    except Exception as e:
        print(f"  ❌ Error: {str(e)[:50]}")

    return False


def main():
    print("Starting Dreame API Fuzzer...")
    for url in ENDPOINTS:
        res = test_endpoint(url)
        if res:
            print("\n!!! FOUND WORKING ENDPOINT !!!")
            print(f"URL: {url}")
            print(json.dumps(res, indent=2))
            break


if __name__ == "__main__":
    main()
