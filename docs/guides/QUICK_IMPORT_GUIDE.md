# Quick Guide: Import Marble Living Room

## TL;DR

1. **Re-export from Marble** as `.fbx` or `.glb` (save to Downloads)
2. **Run**: `python scripts/import_living_room.py`
3. **Open Unity** - assets will auto-import
4. **Done!**

## Detailed Steps

### 1. Re-export from Marble

**In World Labs Marble**:
- Open project: "Modern Tropical Luxury Residence"
- Click Export/Download
- **Choose**: `.fbx` or `.glb` format
- **Save to**: `C:\Users\sandr\Downloads\`

**Why `.fbx`/`.glb`?**
- Native Unity support (no plugins)
- Better for NavMesh (robot navigation)
- Better performance

### 2. Import to Unity

**Option A: Use Script** (Easiest)
```powershell
cd D:\Dev\repos\robotics-mcp
python scripts/import_living_room.py
```

**Option B: Use Tool Directly**
```python
await virtual_robotics(
    action="load_environment",
    environment_path="C:\\Users\\sandr\\Downloads\\Modern Tropical Luxury Residence.fbx",
    platform="unity",
    project_path="C:\\Users\\sandr\\My project"
)
```

### 3. Open Unity

1. Open Unity Editor
2. Open project: `C:\Users\sandr\My project`
3. Wait for assets to import
4. Find environment in: `Assets/WorldLabs/Modern Tropical Luxury Residence/`
5. Drag to Scene

### 4. Add NavMesh (for Robot Navigation)

1. Window > AI > Navigation
2. Select environment mesh
3. Click "Bake"
4. NavMesh ready for robot pathfinding

### 5. Spawn Scout in Environment

```python
await vbot_crud(
    operation="create",
    robot_type="scout",
    platform="unity",
    position={"x": 0, "y": 0.1, "z": 0}  # Slightly above ground
)
```

## File Formats

| Format | Unity Support | Navigation | Visual Quality | Plugin Needed |
|--------|--------------|------------|----------------|---------------|
| `.fbx` | ✅ Native | ✅ Excellent | Good | ❌ No |
| `.glb` | ✅ Native | ✅ Excellent | Good | ❌ No |
| `.ply` | ✅ With plugin | ⚠️ Limited | ✅ Excellent | ✅ Yes (installed) |
| `.spz` | ❌ No | ❌ No | N/A | ❌ N/A |

## Current Status

- ✅ Gaussian Splatting plugin: Installed
- ✅ Import script: Ready
- ⏳ **Waiting for**: Re-exported `.fbx`/`.glb` file

## Troubleshooting

**"No supported files found"**
→ Re-export from Marble as `.fbx` or `.glb`

**"Unity project not found"**
→ Check path: `C:\Users\sandr\My project`

**"Import failed"**
→ Check Unity Editor is closed, or wait for current import to finish

