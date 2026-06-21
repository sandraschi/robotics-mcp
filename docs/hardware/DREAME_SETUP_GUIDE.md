# Dreame D20 Pro Setup Guide

## Overview
This guide explains how to set up your Dreame D20 Pro robot vacuum for control via the Robotics MCP server.

**Kudos to the Dreame engineering team** for their impressive LIDAR auto-mapping technology that creates detailed, real-time environmental maps. We're standing on the shoulders of giants by integrating this sophisticated mapping capability into our multi-robot coordination system.

## Prerequisites
- Dreame D20 Pro robot vacuum
- Dreamehome app installed on Android device
- Python environment with Robotics MCP installed
- `python-miio` library installed

## Step 1: Install Required Software

```bash
# Install python-miio library
pip install python-miio
```

## Step 2: Get Robot Credentials

### Method 1: Network Discovery & Token Testing (No Android Required - RECOMMENDED)
```bash
# First, find your robot's IP address
cd scripts
python discover_dreame.py

# Then get the token
python get_dreame_token.py

# These scripts will:
# 1. Discover your Dreame robot on the network
# 2. Try common/default tokens automatically
# 3. Provide complete configuration for your MCP server
```

### Method 2: Manual Network Discovery
```bash
# Install miiocli
pip install "python-miio[cli]"

# Discover all Xiaomi devices on your network
miiocli discover

# Look for your Dreame D20 Pro in the output
# Note the IP address
```

### Method 2.5: Common Token Testing (If Discovery Works)
If you found the IP address but need the token, try these common defaults:
```bash
# Test common tokens (many Dreame robots use these)
python -c "
from miio import DreameVacuumMiot
import sys

ip = 'YOUR_ROBOT_IP_HERE'  # Replace with your robot's IP
tokens = [
    '00000000000000000000000000000000',
    'FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF',
    '12345678901234567890123456789012'
]

for token in tokens:
    try:
        device = DreameVacuumMiot(ip, token)
        status = device.status()
        if status:
            print(f'SUCCESS: Token {token} works!')
            sys.exit(0)
    except:
        continue

print('No common tokens worked - try other methods')
"
```

### Method 3: Router Device List
1. Log into your router's admin interface
2. Find the list of connected devices
3. Look for your Dreame robot (usually shows as hostname like "dreame-vacuum" or similar)
4. Note the IP address

### Method 4: Cloud API Method (Requires Dreamehome Account)
If you have Dreamehome account credentials but no Android device:
```bash
# Use the automated script with cloud extraction
python scripts/get_dreame_token.py
# Select cloud extraction option when prompted
```

### Method 5: Android Backup Method (Requires Android Device)
1. Install Dreamehome app on Android device
2. Connect your Dreame D20 Pro to the app
3. Use Android backup extraction tools to get app data
4. Extract IP address and token from the backup

### Method 6: Wireshark Network Capture (Advanced)
If other methods fail, capture network traffic during robot setup:

1. Install Wireshark
2. Start capture on your WiFi interface
3. Set up your Dreame robot with the official app (borrow Android device if needed)
4. Look for TCP connections to Xiaomi servers
5. Extract authentication tokens from the captured packets

### Method 7: Xiaomi Cloud API (Advanced)
Direct cloud API access using your Dreamehome credentials:

```bash
# This requires your Dreamehome account email/password
# The automated script includes cloud extraction
python scripts/get_dreame_token.py
# Choose cloud extraction when prompted
```

## Troubleshooting Common Issues

### Robot Not Found on Network
- **Power cycle**: Turn robot off for 30 seconds, then back on
- **Network timing**: Wait 3-5 minutes after power-on for WiFi connection
- **Router placement**: Run discovery script near your router
- **Firewall**: Temporarily disable firewall for testing
- **IP range**: Check if your network uses different IP ranges

### Token Authentication Fails
- **Token format**: Must be exactly 32 hexadecimal characters
- **Case sensitivity**: Tokens are usually uppercase
- **Token expiration**: Some tokens may expire and need refresh
- **Robot reset**: Factory reset may change the token

### Connection Refused
- **Port blocked**: Ensure TCP port 54321 is not blocked
- **IP changed**: Robot IP may change if DHCP assigns new address
- **Network isolation**: Some routers isolate IoT devices

### Mock Mode Testing
If real connection fails, use mock mode for testing:
```yaml
robotics:
  dreame_d20_pro:
    enabled: true
    robot_id: "dreame_01"
    robot_type: "dreame"
    mock_mode: true  # All commands succeed but don't affect real robot
```

## Getting Help
If you're still having trouble:
1. Run the discovery script and share the output
2. Check your router's connected devices list
3. Try different network locations
4. Consider borrowing an Android device temporarily for initial setup

## Step 3: Configure Robot in MCP Server

Add your Dreame robot to the Robotics MCP configuration:

```yaml
robotics:
  dreame_d20_pro:
    enabled: true
    robot_id: "dreame_01"
    robot_type: "dreame"
    ip_address: "192.168.1.XXX"  # Replace with your robot's IP
    token: "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX"  # Replace with your token
    mock_mode: false  # Set to true for testing without real robot
```

## Step 4: Test Connection

Start the Robotics MCP server and test the connection:

```python
from robotics_mcp.tools.robot_control import robot_control
import asyncio

async def test_dreame():
    # Test basic status
    result = await robot_control(robot_id="dreame_01", action="get_status")
    print("Status result:", result)

    # Test starting cleaning
    result = await robot_control(robot_id="dreame_01", action="start_cleaning")
    print("Start cleaning result:", result)

asyncio.run(test_dreame())
```

## Step 5: Web Interface Control

1. Start the Robotics MCP web server
2. Open browser to `http://localhost:12230`
3. Select your Dreame robot from the list
4. Use the Dreame control panel for:
   - Basic cleaning controls (start/stop/return to dock)
   - Suction level settings
   - Water volume and mop humidity
   - Room cleaning, zone cleaning, spot cleaning
   - Mapping controls
   - Advanced features (fast mapping, cleaning sequences, restricted zones)

## Available Dreame Actions

### Basic Controls
- `get_status` - Get current robot status
- `start_cleaning` - Start auto cleaning
- `stop_cleaning` - Stop current cleaning
- `return_to_dock` - Return to charging station

### Settings
- `set_suction_level` - Set suction power (1-4)
- `set_water_volume` - Set mopping water volume (1-3)
- `set_mop_humidity` - Set mop pad humidity (1-3)

### Cleaning Modes
- `clean_room` - Clean specific room by ID
- `clean_zone` - Clean rectangular zones
- `clean_spot` - Intensive cleaning at specific spot
- `start_mapping` - Start room mapping
- `start_fast_mapping` - Quick room mapping

### Advanced Features
- `set_cleaning_sequence` - Set order of room cleaning
- `set_restricted_zones` - Define no-go zones
- `get_cleaning_history` - Retrieve cleaning history
- `clear_error` - Clear robot error states

## Troubleshooting

### Connection Issues
- **Wrong IP/Token**: Double-check credentials from Step 2
- **Firewall**: Ensure port 54321 is open (Dreame's default port)
- **Network**: Robot and MCP server must be on same network

### Robot Not Responding
- **Battery**: Ensure robot has sufficient charge
- **Maintenance**: Check for error codes in Dreamehome app
- **Reset**: Power cycle the robot

### App Data Extraction Issues
- **Android Version**: Method works best on Android 9-11
- **Root Access**: Some methods require rooted device
- **Alternative**: Use network sniffing tools during app connection

## Security Notes
- Robot tokens are sensitive - store securely
- Don't commit tokens to version control
- Use environment variables for production deployment

## Map Data Export and Integration

Your Dreame D20 Pro creates detailed LIDAR maps that can be exported for use by other robots:

### Export Map Data
```bash
# Export map in multiple formats
cd scripts
python export_dreame_map.py dreame_01 ./my_maps

# This creates:
# - Raw JSON data
# - ROS occupancy grid (for navigation)
# - ROS YAML map file
# - Unity NavMesh format
```

### Use Maps with Other Robots

**For Yahboom ROSMASTER Navigation:**
```python
# Use Dreame map for Yahboom navigation
await robot_control(
    robot_id="yahboom_01",
    action="navigate_to",
    x=2000,  # Coordinates from Dreame map (in mm)
    y=1500,
    use_dreame_map=True
)
```

**For Virtual Robot Testing:**
```python
# Import Dreame map into Unity/VRChat
await vbot_crud(
    operation="create",
    robot_type="scout",
    platform="unity",
    environment_map="dreame_exported_map.json"
)
```

## Advanced Integration

### Multi-Robot Coordination
- **Vacuum + Mobile Robot**: Dreame cleans while Yahboom navigates
- **Map Sharing**: Use vacuum's map for wheeled robot path planning
- **Zone Coordination**: Different robots handle different room zones

### Patrol Route Integration
```python
# Create patrol route based on Dreame room layout
patrol_route = [
    {"room": "living_room", "coordinates": [2000, 1500]},
    {"room": "kitchen", "coordinates": [3500, 2000]},
    {"room": "bedroom", "coordinates": [1500, 3500]}
]
```

## Next Steps
Once connected, you can:
1. **Export LIDAR maps** for use by other robots
2. **Integrate with patrol routes** based on room layouts
3. **Combine with Yahboom robots** for multi-robot coordination
4. **Use in virtual testing scenarios** with Unity/VRChat
5. **Create automated cleaning schedules** tied to room maps

For more advanced usage, see the main Robotics MCP documentation.