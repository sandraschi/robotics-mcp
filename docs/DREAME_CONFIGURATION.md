# Dreame D20 Pro Plus Configuration Guide

This guide explains how to configure and connect your Dreame D20 Pro Plus robot vacuum to the Robotics MCP system.

## Prerequisites

1. **Dreame D20 Pro Plus hardware** - Connected to your home WiFi network
2. **Mi Home app** - Installed on your phone and connected to the robot
3. **Python dependencies** - `python-miio` library installed

## Step 1: Obtain Dreame Token

The Dreame robot requires an authentication token to communicate with it. This token can be obtained from the Mi Home app.

### Using Mi Home App (Recommended)

1. Open the Mi Home app on your phone
2. Connect to your Dreame D20 Pro Plus
3. Navigate to robot settings
4. The token may be visible in advanced settings or device information

### Using Network Sniffing (Alternative)

If the token isn't visible in the app:

1. Connect your phone to the same WiFi as the Dreame robot
2. Use a network sniffing tool to capture the authentication handshake
3. Extract the token from the network traffic

**Note**: Token extraction methods may change with app updates. Check the [dreame-vacuum GitHub repository](https://github.com/Tasshack/dreame-vacuum) for the latest methods.

## Step 2: Configure Robotics MCP

Add the Dreame configuration to your `config.yaml`:

```yaml
robotics:
  dreame_d20_pro_plus:
    enabled: true
    robot_id: "dreame_01"
    robot_type: "dreame"
    ip_address: "192.168.1.102"  # Replace with your robot's IP
    token: "your_dreame_token_here"  # Replace with actual token
    mock_mode: false  # Set to false for real hardware

    # Default cleaning settings
    default_settings:
      suction_level: 2  # 1=Quiet, 2=Standard, 3=Turbo, 4=Max
      water_volume: 2   # 1=Low, 2=Medium, 3=High
      mop_humidity: 2   # 1=Low, 2=Medium, 3=High

    # Restricted zones (optional)
    restricted_zones:
      walls: [[0,0,100,0]]  # Virtual wall example
      zones: [[50,50,70,70]] # No-go zone example

    # Cleaning sequences (optional)
    cleaning_sequences:
      default: [1, 2, 3]  # Room IDs in order
```

## Step 3: Network Configuration

Ensure your Dreame robot and Robotics MCP server are on the same network:

1. **Find robot IP**: Use your router's admin panel or network scanning tools
2. **Static IP recommended**: Configure a static IP for the robot in your router
3. **Firewall**: Ensure UDP/TCP ports 54321 and 6667 are open between devices

## Step 4: Test Connection

1. **Start Robotics MCP server** with the configuration
2. **Check logs** for connection status messages
3. **Test basic commands** via MCP tools or web interface:
   - `robot_control get_status robot_id=dreame_01`
   - `robot_control start_cleaning robot_id=dreame_01`

## Step 5: Map Setup (First Run)

On first connection, the Dreame robot will need to map your home:

1. **Start mapping**: Use `start_mapping` or `start_fast_mapping` commands
2. **Monitor progress**: Check robot status during mapping
3. **Save map**: Maps are automatically saved to the configured directory

## Advanced Configuration

### Room Management

After initial mapping, rooms will be auto-discovered. You can:

- **Rename rooms**: Use room management features in Mi Home app
- **Set cleaning sequences**: Configure room cleaning order in config
- **Room-specific cleaning**: Use `clean_room` command with room IDs

### Restricted Zones

Configure virtual walls and no-go zones:

```yaml
restricted_zones:
  walls: [[x1,y1,x2,y2], [x3,y3,x4,y4]]  # Virtual walls
  zones: [[x1,y1,x2,y2], [x3,y3,x4,y4]]  # No-go zones
```

### Multi-Floor Support

For multi-floor homes:

```yaml
map:
  multi_floor: true
  floor_names: ["Ground Floor", "First Floor", "Basement"]
```

## Troubleshooting

### Connection Issues

1. **Check IP address**: Verify robot IP hasn't changed
2. **Token validity**: Ensure token is correct and hasn't expired
3. **Network connectivity**: Ping robot from MCP server
4. **Firewall**: Check for blocked ports

### Command Failures

1. **Robot state**: Ensure robot is idle before sending commands
2. **Battery level**: Check if robot needs charging
3. **Error codes**: Check robot status for error conditions
4. **Clear errors**: Use `clear_error` command if robot reports errors

### Mapping Issues

1. **Lighting**: Ensure adequate lighting for LiDAR mapping
2. **Obstacles**: Clear mapping path of obstacles
3. **Multiple attempts**: Try mapping multiple times if initial attempt fails
4. **Reset map**: Clear existing map and restart mapping

## Web Interface Usage

The web interface provides intuitive controls for Dreame operations:

- **Cleaning Controls**: Start/stop cleaning with one click
- **Settings Panel**: Adjust suction, water, and mop settings
- **Mode Selection**: Choose between room, zone, or spot cleaning
- **Map Display**: Visualize cleaning progress and room layout

## MCP Tool Usage

All Dreame functions are available via MCP tools:

```python
# Basic control
await robot_control(robot_id="dreame_01", action="start_cleaning")
await robot_control(robot_id="dreame_01", action="stop_cleaning")

# Settings
await robot_control(robot_id="dreame_01", action="set_suction_level", suction_level=3)

# Cleaning modes
await robot_control(robot_id="dreame_01", action="clean_room", room_id=1)
await robot_control(robot_id="dreame_01", action="clean_zone", zones=[[0,0,100,100]])

# Mapping
await robot_control(robot_id="dreame_01", action="get_map")
await robot_control(robot_id="dreame_01", action="start_mapping")
```

## Security Considerations

- **Token storage**: Store Dreame tokens securely, not in plain text configs
- **Network isolation**: Consider isolating IoT devices on separate VLAN
- **Access control**: Limit who can control the robot via MCP
- **Regular updates**: Keep robot firmware updated for security patches

## Performance Optimization

- **Map caching**: Maps are cached locally for faster subsequent operations
- **Connection pooling**: MCP maintains persistent connections to robots
- **Batch operations**: Use cleaning sequences for efficient multi-room cleaning
- **Schedule optimization**: Configure cleaning schedules to avoid peak usage times