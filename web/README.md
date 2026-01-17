# Robotics MCP Web Interface

A modern web-based control panel for managing and controlling robots through the Robotics MCP server.

## Features

- **Real-time Robot Control**: Control physical and virtual robots with an intuitive interface
- **Live Status Monitoring**: View robot status, battery levels, position, and sensor data
- **Movement Controls**: Easy-to-use directional controls for robot navigation
- **Special Actions**: Home patrol, camera capture, and status updates
- **Arm & Gripper Control**: Full control over robotic arms and grippers (when equipped)
- **Command Logging**: Real-time log of all commands and responses
- **Responsive Design**: Works on desktop and mobile devices

## Getting Started

### Prerequisites

- Robotics MCP server running with HTTP API enabled (default port 12230)
- At least one robot configured (physical or virtual)

### Accessing the Interface

1. Start the Robotics MCP server:
   ```bash
   python -m robotics_mcp --mode http --port 12230
   ```

2. Open your web browser and navigate to:
   ```
   http://localhost:12230
   ```

3. The web interface will automatically detect available robots and display them for selection.

## Interface Overview

### Robot Selection
- View all configured robots with their connection status
- Click on a robot card to select it for control
- Status indicators show connection state and mock/real mode

### Movement Controls
- **Directional Buttons**: Forward, backward, turn left, turn right
- **Stop Button**: Emergency stop for all movement
- **Speed Control**: Adjustable linear speed (0.1 - 0.5 m/s)

### Special Actions
- **🏠 Patrol**: Start autonomous home patrol routine
- **📷 Capture**: Capture image from robot's camera
- **📊 Status**: Refresh robot status information

### Arm & Gripper Control (when equipped)
- **Joint Control**: Sliders for controlling each joint angle (-180° to 180°)
- **🦾 Move Arm**: Execute arm movement with current joint angles
- **👐 Open/✊ Close**: Control gripper open/close actions

### Status Display
- **Model**: Robot model/type
- **Connection**: Online/offline status
- **Battery**: Current battery level and charging status
- **Position**: Current X/Y coordinates and orientation

## Supported Robots

The web interface works with all robots supported by the Robotics MCP server:

- **Yahboom ROSMASTER Series** (primary focus): Full ROS2 integration, multiple sizes (M1/X3/X3 Plus), camera, navigation, optional arm/gripper, LiDAR add-on available
- **Moorebot Scout** (legacy ROS1): Mecanum wheeled robot with LiDAR
- **Unitree Go2/G1**: Advanced quadrupedal and humanoid robots
- **Virtual Robots**: Unity3D and VRChat-based virtual robots

## API Integration

The web interface communicates with the Robotics MCP server via HTTP REST API:

- `GET /api/v1/robots` - List all robots
- `GET /api/v1/robots/{robot_id}/status` - Get robot status
- `POST /api/v1/robots/{robot_id}/control` - Send control commands

## Dreame D20 Pro Setup

For Dreame robot vacuum control, see the **[Dreame Setup Guide](../docs/DREAME_SETUP_GUIDE.md)** for complete setup instructions including:
- Getting your robot's IP address and authentication token
- Installing required libraries (`python-miio`)
- Configuring the robot in the MCP server
- Testing the connection

## Configuration

Robots are configured through the Robotics MCP server's YAML configuration file:

```yaml
robotics:
  yahboom_raspbot_v2:
    enabled: true
    robot_id: "yahboom_01"
    ip_address: "192.168.1.101"
    mock_mode: true  # Set false for real hardware
    # Arm support is automatically enabled for Yahboom robots
```

## Troubleshooting

### No Robots Found
- Ensure the Robotics MCP server is running
- Check that robots are properly configured in the YAML config
- Verify HTTP API is enabled (default port 12230)

### Robot Not Responding
- Check robot connection status in the interface
- Ensure robot is powered on and connected to network
- For physical robots, verify ROS bridge is running
- For mock mode, all commands will succeed but perform no real actions

### Web Interface Not Loading
- Verify the web files are present in the `web/` directory
- Check that the server is started with HTTP mode enabled
- Try refreshing the page or clearing browser cache

## Development

The web interface is built with:
- **HTML5/CSS3**: Modern responsive design
- **Vanilla JavaScript**: No framework dependencies
- **REST API**: Clean separation between UI and backend

### File Structure
```
web/
├── index.html      # Main HTML interface
├── styles.css      # CSS styling
├── script.js       # JavaScript logic
└── README.md       # This file
```

### Adding New Features

1. **UI Elements**: Add HTML elements to `index.html`
2. **Styling**: Add CSS rules to `styles.css`
3. **Functionality**: Add JavaScript functions to `script.js`
4. **API Calls**: Use the existing `sendCommand()` method pattern

## Browser Support

- Chrome/Edge 90+
- Firefox 88+
- Safari 14+
- Mobile browsers (responsive design)