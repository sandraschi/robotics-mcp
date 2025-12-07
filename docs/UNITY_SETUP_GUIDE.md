# Unity Setup Guide - Scout Vbot Spawning

**Date**: 2025-12-07  
**Status**: Scout model imported ✅

## Current Status

✅ Scout FBX imported to Unity  
✅ All 7 objects visible in Unity  
⏳ Prefab creation needed  
⏳ VbotSpawner setup needed  
⏳ Spawning test needed

## Step-by-Step Setup

### Step 1: Create Scout Prefab

1. **In Unity Project Window**:
   - Navigate to `Assets/Models/`
   - Find `scout_model.fbx`
   - **Expand it** (click arrow) to see all 7 objects

2. **Create Prefab**:
   - Drag `scout_model` (root object) from Project to Scene
   - Position it at origin (0, 0, 0) or wherever you want
   - In Scene, select the `scout_model` GameObject
   - Drag it from Scene back to Project window (creates Prefab)
   - Name it `Scout` or `ScoutPrefab`
   - Delete the instance from Scene (keep the Prefab)

### Step 2: Add VbotSpawner Script

1. **Copy Script to Unity**:
   ```powershell
   Copy-Item "D:\Dev\repos\robotics-mcp\Assets\Scripts\VbotSpawner.cs" -Destination "C:\Users\sandr\My project\Assets\Scripts\VbotSpawner.cs"
   ```

2. **In Unity**:
   - Create `Assets/Scripts/` folder if it doesn't exist
   - Unity should auto-compile the script
   - Check Console for any compilation errors

### Step 3: Setup VbotSpawner in Scene

1. **Create GameObject**:
   - GameObject > Create Empty
   - Name it `VbotSpawner`
   - Position: (0, 0, 0)

2. **Add Component**:
   - Select `VbotSpawner` GameObject
   - Add Component > Scripts > VbotSpawner

3. **Configure Robot Prefabs**:
   - In Inspector, find `Robot Prefabs` list
   - Set Size to `1`
   - Element 0:
     - `Robot Type`: `scout`
     - `Prefab`: Drag your Scout Prefab here

### Step 4: Test Spawning

1. **Ensure Unity Editor is running**
2. **Use robotics-mcp**:
   ```python
   await vbot_crud(
       operation="create",
       robot_type="scout",
       robot_id="scout_01",
       platform="unity",
       position={"x": 0, "y": 0, "z": 0},
       scale=1.0
   )
   ```

3. **Check Unity Scene**:
   - Scout should appear at position (0, 0, 0)
   - Check Console for any errors

## Troubleshooting

### Script Not Found
- Check `Assets/Scripts/VbotSpawner.cs` exists
- Check Unity Console for compilation errors
- Ensure script is in correct namespace (no namespace = global)

### Prefab Not Found
- Verify Prefab is assigned in VbotSpawner component
- Check `robotType` matches exactly: `"scout"` (lowercase)
- Verify Prefab exists in Project window

### Spawn Fails
- Check Unity Console for errors
- Verify `unity3d-mcp` is mounted (check `robotics_system` status)
- Check `execute_unity_method` tool is available
- Verify VbotSpawner GameObject is in active scene

### Robot Not Visible
- Check position (might be at origin or underground)
- Check scale (might be too small: 0.115m = 11.5cm)
- Try larger scale: `scale=10.0` to make it 10× bigger
- Check Scene view (not just Game view)

## Verification Checklist

- [ ] Scout Prefab created
- [ ] VbotSpawner.cs in Assets/Scripts/
- [ ] VbotSpawner GameObject in Scene
- [ ] Scout Prefab assigned to robotPrefabs list
- [ ] robotType set to "scout"
- [ ] Unity Editor running
- [ ] robotics-mcp server running
- [ ] unity3d-mcp mounted and available
- [ ] Test spawn successful

## Next Steps After Setup

1. **Test Movement**:
   - Update position via `vbot_crud(operation="update", ...)`
   - Test scaling
   - Test multiple Scouts

2. **Add Physics**:
   - Add Rigidbody component
   - Add Colliders
   - Test physics interactions

3. **Add Controls**:
   - Movement scripts
   - Wheel rotation
   - Camera controls

