# Quick Start: Get Scout into VRChat

**Time**: ~30 minutes  
**Goal**: Spawn and control Scout robot in VRChat

## Prerequisites

- VRChat installed and running
- OSC enabled in VRChat (Settings → OSC → Enable)
- robotics-mcp server running
- vrchat-mcp and osc-mcp available

## Step 1: Start robotics-mcp Server

```bash
cd robotics-mcp
python -m robotics_mcp.server --mode dual --port 8080
```

## Step 2: Spawn Scout in VRChat

### Option A: Using MCP Tools (Claude Desktop)

```python
# Spawn Scout
await virtual_robotics(
    robot_type="scout",
    action="spawn_robot",
    platform="vrchat",
    robot_id="scouty",
    position={"x": 0, "y": 1, "z": 0},
    scale=1.0
)
```

### Option B: Using HTTP API

```bash
curl -X POST http://localhost:8080/api/v1/tools/virtual_robotics \
  -H "Content-Type: application/json" \
  -d '{
    "robot_type": "scout",
    "action": "spawn_robot",
    "platform": "vrchat",
    "robot_id": "scouty",
    "position": {"x": 0, "y": 1, "z": 0}
  }'
```

## Step 3: Control Scout

```python
# Move forward
await robot_control(
    robot_id="scouty",
    action="move",
    linear=0.2,
    angular=0.0
)

# Turn
await robot_control(
    robot_id="scouty",
    action="move",
    linear=0.0,
    angular=0.5
)

# Stop
await robot_control(
    robot_id="scouty",
    action="stop"
)
```

## Step 4: Check Status

```python
# Get robot status
status = await robot_control(
    robot_id="scouty",
    action="get_status"
)

# List all robots
robots = await list_robots()
```

## Troubleshooting

### Scout doesn't spawn
- Check VRChat OSC is enabled
- Verify OSC port (default: 9000)
- Check world supports OSC spawning
- Review server logs

### Movement doesn't work
- Verify robot is registered: `list_robots()`
- Check OSC messages are being sent
- Verify world OSC handlers are active

### MCP servers not mounted
- Check config: `mcp_integration` section
- Verify vrchat-mcp and osc-mcp are installed
- Review server startup logs

## Next Steps

- Add Scout 3D model for visual representation
- Create custom VRChat world with robot spawn points
- Add more robot behaviors (patrol, follow, etc.)
- Test with multiple robots

---

**Ready to get Scouty into VRChat!** 🚀

