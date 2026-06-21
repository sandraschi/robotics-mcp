# Yahboom Raspbot V2 - Get the robot moving

The Raspbot V2 is a ROS2 Humble robot car (Raspberry Pi 5, Mecanum wheels). It can be controlled from this MCP server over **WiFi hotspot** or **Ethernet** once the robot and rosbridge are running.

## Automated setup (recommended)

**On your PC (from robotics-mcp repo):**

1. Connect the PC to the robot’s WiFi **Raspbot** (password `12345678`), or connect the robot via Ethernet to your LAN.
2. Create/update MCP config and optionally probe the robot:
   ```powershell
   cd D:\Dev\repos\robotics-mcp
   .\scripts\Ensure-RaspbotConfig.ps1 -Probe
   ```
   Default is `192.168.0.250` (Ethernet). Use `-RobotIp 192.168.1.11` for hotspot. Omit `-Probe` to skip the connectivity check.
3. Or use Python directly:
   ```powershell
   uv run raspbot-setup --ensure-config --ip 192.168.0.250 --probe
   ```

**On the robot (Raspberry Pi):**

1. Copy `scripts/robot-start-ros.sh` to the Pi (e.g. via SCP or clone the repo).
2. Make it executable and run it (starts bringup in background, rosbridge in foreground):
   ```bash
   chmod +x robot-start-ros.sh
   ./robot-start-ros.sh
   ```
   Or run the two `ros2 launch` commands manually in two terminals (see section 2 below).

Restart the robotics-mcp server so it reloads config, then use `robot_control(robot_id="yahboom_01", action="move", ...)`.

---

## 1. Connectivity

- **Ethernet**: Connect robot to your LAN. Default robot IP used in this repo is **192.168.0.250**. Adjust in config if different.
- **WiFi hotspot**: Robot creates SSID `Raspbot`, password `12345678`. Connect your PC to this WiFi. Robot IP is **192.168.1.11** (use `-RobotIp 192.168.1.11` or `--ip 192.168.1.11` when ensuring config).

## 2. On the robot (Raspberry Pi)

SSH to the robot (e.g. `ssh yahboom@192.168.1.11` when on its hotspot). Start:

1. **Base driver** (publishes `/cmd_vel`, drives motors):

   ```bash
   ros2 launch yahboomcar_bringup yahboomcar_bringup_launch.py
   ```

2. **Rosbridge** (so the MCP server can send commands from your PC):

   ```bash
   ros2 launch rosbridge_server rosbridge_websocket_launch.xml
   ```

Leave both running. If using a single terminal, run the bringup first, then in another terminal/session start rosbridge.

## 3. MCP config

Ensure a Yahboom robot is enabled in `~/.robotics-mcp/config.yaml` with the robot’s IP and `mock_mode: false`:

```yaml
robotics:
  yahboom_raspbot_v2:
    enabled: true
    robot_id: yahboom_01
    ip_address: 192.168.0.250   # Ethernet default; use 192.168.1.11 for WiFi hotspot
    port: 9090
    mock_mode: false
    camera_enabled: true
    navigation_enabled: true
    arm_enabled: false
```

If the config file does not exist, create `~/.robotics-mcp/config.yaml` with the block above (and any other robots you use). The server loads it at startup and registers `yahboom_01`.

## 4. Move the car

With robotics-mcp running and the robot registered:

- **Move**: `robot_control(robot_id="yahboom_01", action="move", linear=0.3, angular=0.0, duration=2.0)`
- **Stop**: `robot_control(robot_id="yahboom_01", action="stop")`
- **Status**: `robot_control(robot_id="yahboom_01", action="get_status")`

`linear` (m/s) and `angular` (rad/s) are sent as `geometry_msgs/Twist` on `/cmd_vel`. Raspbot V2 has Mecanum wheels (forward/back, strafe, rotate).

## 5. Troubleshooting

- **No movement**: Confirm bringup is running on the robot and that `/cmd_vel` is published (e.g. `ros2 topic echo /cmd_vel` on the robot while you send a move command).
- **Connection refused / timeout**: Check PC is on the same network as the robot (WiFi “Raspbot” or same LAN via Ethernet). Ping the robot IP. Ensure rosbridge is running and nothing else is bound to port 9090.
- **Simulated mode**: If `mock_mode` is true or rosbridge is unreachable, the client falls back to simulated mode (no real movement). Set `mock_mode: false` and fix connectivity so the server can open a WebSocket to the robot.
- **Dependencies**: Real control needs `pip install roslibpy`. Without it, the client stays in simulated mode.

## References

- [Yahboom Raspbot V2 study](https://www.yahboom.net/study/RASPBOT-V2)
- [ROS Robot App Mapping (rosbridge)](https://www.yahboom.net/public/upload/upload-html/1734925545/ROS%20Robot%20App%20Mapping.html)
- [Raspbot V2 GitHub](https://github.com/YahboomTechnology/Raspbot-V2)
