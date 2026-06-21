# Importing Scout Model to Unity3D

## Overview

The Scout 3D model has been exported as FBX (`D:\Models\scout_model.fbx`, 14.9 KB). This guide shows how to import it into Unity3D using `unity3d-mcp` tools.

## Why FBX is Smaller Than .blend

- **FBX**: Compressed binary format, optimized for runtime
- **.blend**: Contains full Blender scene data, undo history, metadata, materials, etc.

14.9 KB is normal for a simple robot model with 7 objects (body, 4 wheels, camera, mounting plate).

## Import Methods

### Method 1: Direct File Copy (Simplest)

Unity automatically imports files placed in the `Assets/` folder. Simply copy the FBX:

```powershell
# Copy FBX to Unity project Assets folder
Copy-Item "D:\Models\scout_model.fbx" -Destination "D:\Projects\UnityProject\Assets\Models\scout_model.fbx"
```

Unity will detect the new file and import it automatically.

### Method 2: Using unity3d-mcp Tools

The `unity3d-mcp` server provides tools for importing assets. However, it currently focuses on:
- `.unitypackage` files (`import_asset_package`)
- VRM avatars (`import_vrm_avatar`)
- World Labs Marble worlds (`import_marble_world`)

For FBX files, we can:
1. Use the file copy method above
2. Or extend `unity3d-mcp` with an `import_model` tool

### Method 3: Using robotics-mcp Integration

Since `robotics-mcp` mounts `unity3d-mcp`, we can create a workflow that:
1. Copies the FBX to Unity Assets folder
2. Optionally configures import settings via Unity Editor API
3. Creates a prefab for easy instantiation

## Next Steps

1. **Copy FBX to Unity project**:
   ```powershell
   # Ensure Models folder exists
   New-Item -ItemType Directory -Force -Path "D:\Projects\UnityProject\Assets\Models"
   Copy-Item "D:\Models\scout_model.fbx" -Destination "D:\Projects\UnityProject\Assets\Models\"
   ```

2. **Unity will auto-import** - Check Unity Console for import status

3. **Create Prefab** (optional):
   - Drag the imported model from Project window to Scene
   - Drag from Scene to Project window to create prefab
   - Or use `unity3d-mcp` tools if available

4. **Use VbotSpawner** (from `UNITY_VBOT_INSTANTIATION.md`):
   - Add `VbotSpawner.cs` to scene
   - Assign Scout prefab to `robotPrefabs` list
   - Use `vbot_crud` tool to spawn Scout in scene

## Unity Import Settings

After import, check these settings in Unity:
- **Scale Factor**: 1 (model is already in meters)
- **Mesh Compression**: Off (for small models)
- **Read/Write Enabled**: Enabled (if needed for runtime manipulation)
- **Generate Colliders**: Optional (for physics)

## Integration with robotics-mcp

The `vbot_crud` tool can spawn the Scout once it's imported:

```python
# Spawn Scout in Unity scene
await vbot_crud(
    operation="create",
    robot_type="scout",
    robot_id="scout_01",
    platform="unity",
    position={"x": 0, "y": 0, "z": 0},
    scale=1.0
)
```

This requires:
1. Scout FBX imported to Unity
2. Scout prefab created
3. `VbotSpawner.cs` in scene with prefab assigned
4. `unity3d-mcp` running and connected

## Complete Setup Workflow

### Step 1: Import FBX to Unity

Run the import script:
```powershell
python scripts/import_scout_to_unity.py
```

Or manually copy:
```powershell
Copy-Item "D:\Models\scout_model.fbx" -Destination "D:\Projects\UnityProject\Assets\Models\"
```

### Step 2: Create Prefab in Unity

1. In Unity Editor, find `Assets/Models/scout_model.fbx` in Project window
2. Drag it to the Scene (creates an instance)
3. Position it where you want (or leave at origin)
4. Drag the GameObject from Scene to Project window (creates Prefab)
5. Name it `Scout` or `ScoutPrefab`
6. Delete the instance from Scene (keep the Prefab)

### Step 3: Setup VbotSpawner

1. Copy `Assets/Scripts/VbotSpawner.cs` to your Unity project's `Assets/Scripts/` folder
2. In Unity, create an empty GameObject in Scene (GameObject > Create Empty)
3. Name it `VbotSpawner`
4. Add `VbotSpawner` component to it (Add Component > Scripts > VbotSpawner)
5. In Inspector, expand `Robot Prefabs` list
6. Set Size to 1
7. Drag your Scout Prefab to Element 0's `Prefab` field
8. Set `Robot Type` to "scout"

### Step 4: Spawn via robotics-mcp

Now you can spawn Scouts using the `vbot_crud` tool:

```python
# Spawn Scout at origin
await vbot_crud(
    operation="create",
    robot_type="scout",
    robot_id="scout_01",
    platform="unity",
    position={"x": 0, "y": 0, "z": 0},
    scale=1.0
)
```

The tool will:
1. Call `VbotSpawner.SpawnRobot()` via `unity3d-mcp`'s `execute_unity_method`
2. VbotSpawner instantiates the Scout prefab
3. Returns success with robot_id

## Troubleshooting

**FBX not importing:**
- Check Unity Console for errors
- Ensure FBX file is valid (try opening in Blender)
- Check file permissions

**VbotSpawner not found:**
- Ensure `VbotSpawner.cs` is in `Assets/Scripts/`
- Check Unity Console for compilation errors
- Ensure VbotSpawner GameObject is in the active scene

**Prefab not found:**
- Check `robotPrefabs` list in VbotSpawner component
- Ensure `robotType` matches exactly ("scout")
- Verify prefab is assigned to the list

**unity3d-mcp connection issues:**
- Check if `unity3d-mcp` is mounted in `robotics-mcp`
- Verify Unity Editor is running
- Check `unity3d-mcp` server logs

