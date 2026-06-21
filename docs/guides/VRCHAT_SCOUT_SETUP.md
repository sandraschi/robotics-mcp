# VRChat Scout Setup - Complete Guide

**Goal**: Get Scouty (Moorebot Scout) into VRChat by tomorrow!

## 🎯 Overview

This guide covers three approaches to get Scout into VRChat:
1. **OSC-Controlled Spawnable Object** (Fastest - no Unity build needed)
2. **Unity → VRChat Avatar** (Best for permanent integration)
3. **Unity → VRChat World Object** (Best for world-specific robots)

## Method 1: OSC-Controlled Spawnable (Recommended for Tomorrow)

### Prerequisites
- VRChat running with OSC enabled
- VRChat world that supports OSC spawning (or create simple test world)
- robotics-mcp server running

### Step 1: Prepare VRChat World

If you have a world with OSC support:

```python
# Just spawn via OSC
await virtual_robotics(
    robot_type="scout",
    action="spawn_robot",
    platform="vrchat",
    robot_id="scouty",
    position={"x": 0, "y": 1, "z": 0}
)
```

If you need to create a test world:

1. **Create Simple Unity Project**:
   ```python
   # Use unity3d-mcp
   await unity_create_unity_project(
       project_name="ScoutTestWorld",
       project_path="D:/Projects",
       template="3D"
   )
   ```

2. **Add OSC Handler Script** (UdonSharp):
   ```csharp
   // Simple spawner that responds to OSC
   public class ScoutSpawner : UdonSharpBehaviour
   {
       public GameObject scoutPrefab;
       
       public void OnOSCMessage(string address, object[] args)
       {
           if (address == "/world/spawn/scout")
           {
               Vector3 pos = new Vector3(
                   (float)args[0],
                   (float)args[1],
                   (float)args[2]
               );
               float scale = (float)args[3];
               
               GameObject scout = Instantiate(scoutPrefab);
               scout.transform.position = pos;
               scout.transform.localScale = Vector3.one * scale;
           }
       }
   }
   ```

3. **Upload to VRChat**:
   ```python
   await unity_vrchat_upload_world(
       project_path="D:/Projects/ScoutTestWorld",
       world_name="Scout Test World"
   )
   ```

### Step 2: Spawn Scout

```python
# Spawn Scout in VRChat
result = await virtual_robotics(
    robot_type="scout",
    action="spawn_robot",
    platform="vrchat",
    robot_id="scouty",
    position={"x": 0, "y": 1, "z": 0},
    scale=1.0
)

print(f"Scout spawned: {result}")
```

### Step 3: Control Scout

```python
# Move Scout
await robot_control(
    robot_id="scouty",
    action="move",
    linear=0.2,
    angular=0.0
)

# Stop Scout
await robot_control(
    robot_id="scouty",
    action="stop"
)
```

## Method 2: Unity → VRChat Avatar

### Step 1: Import Scout Model to Unity

```python
# Import Scout 3D model
await unity_import_model(
    model_path="assets/models/scout/scout_base.fbx",
    project_path="D:/Projects/VRChatScout"
)
```

### Step 2: Set up as VRChat Avatar

```python
# Check VRChat auth
await unity_vrchat_check_auth()

# Set up VRChat SDK
await unity_vrchat_setup_sdk(
    project_path="D:/Projects/VRChatScout"
)

# Configure avatar
await unity_vrchat_configure_avatar(
    project_path="D:/Projects/VRChatScout",
    avatar_path="Assets/Scout/scout_base.fbx"
)
```

### Step 3: Upload Avatar

```python
# Upload to VRChat
await unity_vrchat_upload_avatar(
    project_path="D:/Projects/VRChatScout",
    avatar_name="Scouty"
)
```

### Step 4: Use as Avatar

Once uploaded, you can:
- Change to Scout avatar in VRChat
- Control via avatar-mcp tools
- Use OSC for parameter control

## Method 3: Unity → VRChat World Object

### Step 1: Create World with Scout

```python
# Create Unity project
await unity_create_unity_project(
    project_name="ScoutWorld",
    project_path="D:/Projects"
)

# Import Scout model
await unity_import_model(
    model_path="assets/models/scout/scout_base.fbx",
    project_path="D:/Projects/ScoutWorld"
)

# Add Scout to scene
await unity_execute_method(
    class_name="SceneSetup",
    method_name="AddScoutToScene",
    project_path="D:/Projects/ScoutWorld"
)
```

### Step 2: Build and Upload World

```python
# Build for VRChat
await unity_build_unity_project(
    project_path="D:/Projects/ScoutWorld",
    build_target="StandaloneWindows64"
)

# Upload world
await unity_vrchat_upload_world(
    project_path="D:/Projects/ScoutWorld",
    world_name="Scout's World"
)
```

## 🚀 Quick Test Script

```python
# Complete workflow to get Scout in VRChat
import asyncio
from robotics_mcp.server import RoboticsMCP, RoboticsConfig

async def main():
    # Start server
    config = RoboticsConfig(enable_http=True)
    server = RoboticsMCP(config)
    
    # Spawn Scout
    spawn_result = await server.virtual_robotics._spawn_robot(
        robot_type="scout",
        robot_id="scouty",
        position={"x": 0, "y": 1, "z": 0},
        scale=1.0,
        platform="vrchat"
    )
    print(f"Spawned: {spawn_result}")
    
    # Move Scout
    move_result = await server.robot_control.handle_action(
        "scouty",
        "move",
        {"linear": 0.2, "angular": 0.0}
    )
    print(f"Move: {move_result}")

if __name__ == "__main__":
    asyncio.run(main())
```

## 📋 Tomorrow's Checklist

- [ ] Scout 3D model ready (`.fbx` or `.obj`)
- [ ] VRChat world with OSC support
- [ ] robotics-mcp server running
- [ ] Test spawn: `virtual_robotics(action="spawn_robot", platform="vrchat")`
- [ ] Test movement: `robot_control(action="move")`
- [ ] Test status: `robot_control(action="get_status")`
- [ ] Verify in VRChat world

## 🐛 Troubleshooting

### "Robot not found"
- Check robot is registered: `list_robots()`
- Verify robot_id matches spawn ID

### "OSC not working"
- Check VRChat OSC is enabled
- Verify port (default: 9000)
- Check world has OSC handlers

### "MCP servers not mounted"
- Check config file
- Verify vrchat-mcp and osc-mcp installed
- Review server logs

---

**Let's get Scouty into VRChat!** 🚀🤖

