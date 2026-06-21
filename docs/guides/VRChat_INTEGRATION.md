# VRChat Integration Guide - Getting Scout into VRChat

**Goal**: Get Moorebot Scout ("Scouty") into VRChat worlds by tomorrow!

## 🎯 Quick Start: Scout in VRChat

### Method 1: OSC-Based Spawning (Recommended)

VRChat worlds can have spawnable objects controlled via OSC. We'll use OSC to spawn and control Scout.

```python
# 1. Spawn Scout in VRChat world
await virtual_robotics(
    robot_type="scout",
    action="spawn_robot",
    platform="vrchat",
    position={"x": 0, "y": 0, "z": 0},
    scale=1.0
)

# 2. Control Scout movement via OSC
await robot_control(
    robot_id="vbot_scout_01",
    action="move",
    linear=0.2,
    angular=0.0
)
```

### Method 2: Unity → VRChat Pipeline

1. **Create Scout 3D Model** (if not available):
   - Use Blender MCP or manual creation
   - Export as `.fbx` or `.obj`
   - Reference: `mcp-central-docs/docs/robotics/SCOUT_3D_MODEL.md`

2. **Import to Unity**:
   ```python
   # Use unity3d-mcp
   await unity_import_model(
       model_path="assets/models/scout/scout_base.fbx",
       project_path="D:/Projects/VRChatScout"
   )
   ```

3. **Set up VRChat SDK**:
   ```python
   # Use unity3d-mcp VRChat tools
   await unity_vrchat_check_auth()
   await unity_vrchat_setup_sdk(project_path="D:/Projects/VRChatScout")
   ```

4. **Build and Upload**:
   ```python
   # Build for VRChat
   await unity_build_unity_project(
       project_path="D:/Projects/VRChatScout",
       build_target="StandaloneWindows64"
   )
   ```

## 🔧 VRChat World Setup

### OSC Addresses for Robot Control

VRChat worlds can expose OSC addresses for object control:

```
/world/spawn/scout          # Spawn Scout robot
/robot/{robot_id}/move      # Move robot (linear, angular)
/robot/{robot_id}/stop      # Stop robot
/robot/{robot_id}/status    # Get robot status
```

### World Requirements

For OSC-based spawning, the VRChat world needs:
- OSC-enabled spawnable objects
- Custom OSC handlers for robot control
- World scripts that respond to OSC messages

## 🚀 Implementation Steps

### Step 1: Prepare Scout 3D Model

1. **Get/Create Scout Model**:
   - Check `external/moorebot-scout-sdk/` for any model files
   - Or create using Blender (see SCOUT_3D_MODEL.md)
   - Export as `.fbx` for Unity/VRChat

2. **Model Specifications**:
   - Size: 11.5×10×8 cm (actual Scout)
   - Scale: 1.0 = actual size, 6.1 = Go2 size (70cm)
   - Format: `.fbx` or `.obj` with textures

### Step 2: VRChat World Integration

**Option A: OSC-Controlled Spawnable Object**

1. Create spawnable object prefab in Unity
2. Add OSC receiver script
3. Upload to VRChat as world
4. Control via OSC from robotics-mcp

**Option B: Avatar-Based Robot**

1. Create Scout as VRChat avatar
2. Use avatar-mcp for control
3. Spawn in world as player avatar
4. Control via avatar parameters

### Step 3: Control via robotics-mcp

```python
# Spawn Scout
result = await virtual_robotics(
    robot_type="scout",
    action="spawn_robot",
    platform="vrchat",
    robot_id="scouty_01",
    position={"x": 0, "y": 0, "z": 0}
)

# Move Scout
await robot_control(
    robot_id="scouty_01",
    action="move",
    linear=0.2,
    angular=0.0
)

# Get status
status = await robot_control(
    robot_id="scouty_01",
    action="get_status"
)
```

## 📋 Checklist for Tomorrow

- [ ] Scout 3D model ready (`.fbx` or `.obj`)
- [ ] VRChat world with OSC support (or use existing world)
- [ ] OSC addresses configured in world
- [ ] robotics-mcp server running
- [ ] vrchat-mcp and osc-mcp mounted
- [ ] Test spawn in VRChat
- [ ] Test movement control
- [ ] Test status monitoring

## 🎮 Example VRChat World Script

If creating custom world, add OSC handler:

```csharp
// VRChat UdonSharp script
using UdonSharp;
using VRC.SDKBase;
using VRC.Udon;

public class RobotSpawner : UdonSharpBehaviour
{
    public GameObject scoutPrefab;
    
    public void OnOSCMessage(string address, object[] args)
    {
        if (address == "/world/spawn/scout")
        {
            float x = (float)args[0];
            float y = (float)args[1];
            float z = (float)args[2];
            float scale = (float)args[3];
            
            GameObject scout = Instantiate(scoutPrefab);
            scout.transform.position = new Vector3(x, y, z);
            scout.transform.localScale = Vector3.one * scale;
        }
    }
}
```

## 🔗 Resources

- VRChat OSC Documentation: https://docs.vrchat.com/docs/osc-overview
- VRChat World Creation: https://docs.vrchat.com/docs/worlds
- Scout 3D Model Guide: `mcp-central-docs/docs/robotics/SCOUT_3D_MODEL.md`
- VRChat MCP: `vrchat-mcp/`
- OSC MCP: `osc-mcp/`

---

**Goal**: Scouty in VRChat by tomorrow! 🚀🤖

