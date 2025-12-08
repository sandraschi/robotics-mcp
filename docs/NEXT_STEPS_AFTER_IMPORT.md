# Next Steps After Importing Living Room Environment

**Date**: 2025-12-02  
**Status**: Environment imported ✅ | Scout model ready ✅ | Setup needed ⏳

## Overview

You've successfully imported:
- ✅ Collider GLB (`Modern Tropical Luxury Residence_collider.glb`)
- ✅ Visual Splats (2× PLY files)
- ✅ Scout FBX model (`scout_model.fbx`)

**Next**: Set up Unity scene, create Scout prefab, configure VbotSpawner, bake NavMesh, and test spawning.

---

## Step 1: Verify Imported Assets in Unity

1. **Open Unity Editor** (if not already open)
   - Project: `C:\Users\sandr\My project`

2. **Check Assets Window**:
   - Look for `Assets/WorldLabs/` folder (created by import)
   - Verify collider GLB is present
   - Verify PLY splat files are present (may take time to import)

3. **Check Scout Model**:
   - Navigate to `Assets/Models/` (or wherever `scout_model.fbx` was imported)
   - Expand `scout_model` to see all 7 objects:
     - `scout_body`
     - `scout_wheel_1` through `scout_wheel_4`
     - `scout_camera`
     - `scout_mounting_plate`

4. **Wait for Import**:
   - Unity may still be importing assets (check bottom-right progress bar)
   - PLY files may take several minutes to process
   - GLB should import quickly

---

## Step 2: Create Scout Prefab

1. **In Unity Project Window**:
   - Navigate to `Assets/Models/scout_model.fbx`
   - **Expand it** (click arrow) to see all 7 child objects

2. **Create Prefab Folder** (if needed):
   - Right-click in Project window → Create → Folder
   - Name it `Prefabs`

3. **Create Prefab**:
   - Drag `scout_model` (root object) from Project to **Scene Hierarchy**
   - Position it at origin: `(0, 0, 0)` (or wherever you want)
   - In Scene, select the `scout_model` GameObject
   - Drag it from Hierarchy back to `Assets/Prefabs/` folder
   - This creates a `.prefab` file
   - Name it `Scout` or `ScoutPrefab`
   - **Delete the instance from Scene** (keep the Prefab)

4. **Configure Prefab** (optional but recommended):
   - Select the Prefab in Project window
   - In Inspector, add components:
     - **Rigidbody** (for physics)
     - **BoxCollider** or **CapsuleCollider** (for collision)
   - Save changes (Ctrl+S)

---

## Step 3: Set Up VbotSpawner Script

1. **Copy Script to Unity**:
   ```powershell
   # Ensure Scripts folder exists
   New-Item -ItemType Directory -Force -Path "C:\Users\sandr\My project\Assets\Scripts"
   
   # Copy VbotSpawner.cs
   Copy-Item "D:\Dev\repos\robotics-mcp\Assets\Scripts\VbotSpawner.cs" -Destination "C:\Users\sandr\My project\Assets\Scripts\VbotSpawner.cs"
   ```

2. **In Unity**:
   - Unity should auto-compile the script
   - Check **Console** (Window > General > Console) for any compilation errors
   - If errors, fix them (usually missing `using` statements)

---

## Step 4: Create VbotSpawner GameObject in Scene

1. **Create Empty GameObject**:
   - Right-click in Hierarchy → Create Empty
   - Name it `VbotSpawner`
   - Position: `(0, 0, 0)`

2. **Add VbotSpawner Component**:
   - Select `VbotSpawner` GameObject
   - In Inspector, click **Add Component**
   - Search for `VbotSpawner`
   - Add it

3. **Configure Robot Prefabs**:
   - In Inspector, find `Robot Prefabs` list
   - Set **Size** to `1`
   - **Element 0**:
     - `Robot Type`: `scout` (lowercase, must match exactly)
     - `Prefab`: Drag your `Scout` Prefab from `Assets/Prefabs/` here

4. **Save Scene**:
   - File > Save Scene (Ctrl+S)
   - Name it `MainScene` or `LivingRoomScene`

---

## Step 5: Set Up Environment in Scene

1. **Add Collider GLB to Scene**:
   - Drag `Modern Tropical Luxury Residence_collider.glb` from `Assets/WorldLabs/` to Scene Hierarchy
   - Position at origin: `(0, 0, 0)`
   - This provides collision and NavMesh surface

2. **Add PLY Splats** (optional, for visual rendering):
   - Drag PLY files from `Assets/WorldLabs/` to Scene
   - Position at origin: `(0, 0, 0)`
   - **Note**: PLY splats require Gaussian Splatting plugin (already installed)
   - These are visual only, not for collision

3. **Add Lighting** (if needed):
   - GameObject > Light > Directional Light
   - Adjust rotation to illuminate the scene

4. **Add Camera** (if needed):
   - GameObject > Camera
   - Position: `(0, 5, -10)` (looking at origin)
   - Or use existing Main Camera

---

## Step 6: Bake NavMesh for Navigation

1. **Open Navigation Window**:
   - Window > AI > Navigation

2. **Select Navigation Surface**:
   - Select the collider GLB GameObject in Hierarchy
   - In Inspector, ensure it has a **Mesh Collider** or **Box Collider**
   - In Navigation window, check **Navigation Static**

3. **Bake NavMesh**:
   - In Navigation window, go to **Bake** tab
   - Click **Bake** button
   - Wait for baking to complete (may take a minute)
   - You should see a blue overlay on walkable surfaces

4. **Verify NavMesh**:
   - In Scene view, you should see blue areas (walkable)
   - Non-walkable areas remain uncolored

---

## Step 7: Test Spawning Scout Vbot

1. **Ensure Unity Editor is Running**

2. **Test via Python Script**:
   ```python
   # Create a test script: test_spawn_scout.py
   import asyncio
   from robotics_mcp.server import RoboticsMCP, RoboticsConfig
   from fastmcp import Client

   async def test_spawn():
       config = RoboticsConfig()
       server = RoboticsMCP(config)
       
       async with Client(server.mcp) as client:
           # Spawn Scout at origin
           result = await client.call_tool(
               "vbot_crud",
               {
                   "operation": "create",
                   "robot_type": "scout",
                   "robot_id": "scout_01",
                   "platform": "unity",
                   "position": {"x": 0.0, "y": 0.0, "z": 0.0},
                   "scale": 1.0
               }
           )
           print(f"Spawn result: {result}")
   
   asyncio.run(test_spawn())
   ```

3. **Or Test via Cursor/Claude**:
   ```
   Use vbot_crud tool:
   - operation: "create"
   - robot_type: "scout"
   - robot_id: "scout_01"
   - platform: "unity"
   - position: {"x": 0.0, "y": 0.0, "z": 0.0}
   - scale: 1.0
   ```

4. **Check Unity Scene**:
   - Scout should appear at position `(0, 0, 0)`
   - Check **Console** for any errors
   - If Scout is too small, try `scale: 10.0` to make it 10× bigger

---

## Step 8: Test New Tools (Priority 1)

### Test Robot Animation

```python
# Set wheel speed
await client.call_tool(
    "robot_animation",
    {
        "robot_id": "scout_01",
        "operation": "set_wheel_speed",
        "speed": 0.5
    }
)
```

### Test Robot Camera

```python
# Capture image
await client.call_tool(
    "robot_camera",
    {
        "robot_id": "scout_01",
        "operation": "capture_image",
        "format": "png",
        "output_path": "C:/Users/sandr/Desktop/scout_view.png"
    }
)
```

### Test Robot Navigation

```python
# Plan path to target
await client.call_tool(
    "robot_navigation",
    {
        "robot_id": "scout_01",
        "operation": "plan_path",
        "target_position": {"x": 5.0, "y": 0.0, "z": 5.0}
    }
)

# Follow path
await client.call_tool(
    "robot_navigation",
    {
        "robot_id": "scout_01",
        "operation": "follow_path",
        "speed": 0.5
    }
)
```

**Note**: These tools require Unity C# scripts (`RobotAnimator.cs`, `RobotCamera.cs`, `RobotNavigator.cs`) to be added to the Scout prefab. See `docs/PRIORITY_1_IMPLEMENTATION.md` for details.

---

## Troubleshooting

### Scout Not Appearing

- **Check VbotSpawner**: Ensure GameObject is in scene and component is added
- **Check Prefab**: Verify Scout Prefab is assigned in VbotSpawner's Robot Prefabs list
- **Check robot_type**: Must be exactly `"scout"` (lowercase)
- **Check Position**: Scout might be at origin (0,0,0) or underground
- **Check Scale**: Try `scale: 10.0` to make it bigger
- **Check Console**: Look for errors in Unity Console

### NavMesh Not Baking

- **Check Navigation Static**: Ensure collider GLB has "Navigation Static" checked
- **Check Collider**: Ensure GameObject has a Collider component
- **Check Mesh**: Ensure mesh is valid (not empty or corrupted)
- **Check Bake Settings**: In Navigation window, adjust Agent Radius/Height if needed

### Tools Not Working

- **Check Unity Scripts**: Ensure `RobotAnimator.cs`, `RobotCamera.cs`, `RobotNavigator.cs` are in `Assets/Scripts/`
- **Check Components**: Add these scripts as components to Scout Prefab
- **Check robotics-mcp**: Verify server is running and `unity3d-mcp` is mounted
- **Check Console**: Look for errors in Unity Console

### PLY Splats Not Rendering

- **Check Plugin**: Verify Gaussian Splatting plugin is installed (check `Packages/manifest.json`)
- **Check Import**: PLY files may take time to import (check progress bar)
- **Check Material**: PLY splats may need a specific material/shader
- **Check Camera**: Ensure camera can see the splat position

---

## Verification Checklist

- [ ] Imported assets visible in `Assets/WorldLabs/`
- [ ] Scout model visible in `Assets/Models/scout_model.fbx`
- [ ] Scout Prefab created in `Assets/Prefabs/`
- [ ] VbotSpawner.cs in `Assets/Scripts/`
- [ ] VbotSpawner GameObject in Scene
- [ ] Scout Prefab assigned to VbotSpawner
- [ ] Collider GLB added to Scene
- [ ] NavMesh baked successfully
- [ ] Test spawn successful (Scout appears in Scene)
- [ ] Unity Console shows no errors

---

## Next Steps After Setup

1. **Test Multiple Scouts**:
   - Spawn multiple Scouts with different IDs
   - Test positioning and scaling

2. **Add Physics**:
   - Add Rigidbody to Scout Prefab
   - Test collisions with environment

3. **Test Navigation**:
   - Use `robot_navigation` tool to move Scout around
   - Test path planning and following

4. **Test Camera**:
   - Use `robot_camera` tool to capture images
   - Test camera angle adjustments

5. **Test Animation**:
   - Use `robot_animation` tool to control wheel speed
   - Test different animation states

6. **Add More Robots**:
   - Create prefabs for other robot types (Robbie, Go2, G1)
   - Add them to VbotSpawner's Robot Prefabs list

---

## Quick Reference

**Unity Project Path**: `C:\Users\sandr\My project`  
**Scout Model**: `Assets/Models/scout_model.fbx`  
**Scout Prefab**: `Assets/Prefabs/Scout.prefab`  
**VbotSpawner Script**: `Assets/Scripts/VbotSpawner.cs`  
**Environment Collider**: `Assets/WorldLabs/Modern Tropical Luxury Residence_collider.glb`  
**Environment Splats**: `Assets/WorldLabs/*.ply`

**robotics-mcp Tools**:
- `vbot_crud`: Create/update/delete/list vbots
- `robot_animation`: Control animations and behaviors
- `robot_camera`: Capture images and manage camera
- `robot_navigation`: Path planning and navigation
- `virtual_robotics`: High-level virtual robot operations

---

**Austrian Precision**: Follow each step carefully, verify at each stage, and check Unity Console for errors! 🇦🇹🤖

