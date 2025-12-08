# Import Marble Living Room to Unity - Step by Step

**Date**: 2025-12-07

## Current Situation

You have: `Modern Tropical Luxury Residence.spz` (31MB)  
**Problem**: Unity does NOT support `.spz` files  
**Solution**: Re-export from Marble in a supported format

## Step-by-Step Instructions

### Step 1: Re-export from Marble

1. **Open World Labs Marble** (web or app)
2. **Open your project**: "Modern Tropical Luxury Residence"
3. **Export the environment**:
   - Click Export or Download
   - **Choose format**:
     - **Option A**: `.fbx` or `.glb` (Recommended for robotics)
     - **Option B**: `.ply` (For visual quality, requires plugin)
4. **Save to**: `C:\Users\sandr\Downloads\`

### Step 2: Import to Unity

Once you have the exported file, run:

```python
# Via robotics-mcp tool
await virtual_robotics(
    action="load_environment",
    environment="Modern Tropical Luxury Residence",
    environment_path="C:\\Users\\sandr\\Downloads\\Modern Tropical Luxury Residence.fbx",  # or .glb or .ply
    platform="unity",
    project_path="C:\\Users\\sandr\\My project",
    include_colliders=True
)
```

Or use the helper script:
```powershell
python scripts/import_living_room.py
```

## Recommended Format

### For Robot Navigation (Your Use Case):

**Use `.fbx` or `.glb`**:
- ✅ Native Unity support (no plugins)
- ✅ Better for NavMesh pathfinding
- ✅ Better performance
- ✅ Easier to work with colliders

### For Visual Quality:

**Use `.ply`**:
- ✅ Better visual quality (Gaussian splats)
- ✅ Requires Gaussian Splatting plugin (already installed)
- ⚠️ Harder to use for navigation

## What Happens After Import

1. **File copied to Unity project**: `Assets/WorldLabs/Modern Tropical Luxury Residence/`
2. **Organized structure**:
   - `Visuals/` - Main mesh/splat files
   - `Colliders/` - Collision geometry (if included)
   - `Splats/` - Gaussian splat files (if .ply)
3. **Unity auto-imports** when you open the project
4. **Ready to use** in your scene

## Next Steps After Import

1. **Open Unity Editor**
2. **Wait for import** to complete
3. **Add to scene**:
   - Drag from `Assets/WorldLabs/...` to Scene
   - Or use `vbot_crud` to spawn robots in the environment
4. **Add NavMesh** (for robot navigation):
   - Window > AI > Navigation
   - Select environment mesh
   - Bake NavMesh
5. **Test with Scout vbot**:
   ```python
   await vbot_crud(
       operation="create",
       robot_type="scout",
       platform="unity",
       position={"x": 0, "y": 0, "z": 0}
   )
   ```

## Troubleshooting

### "File not found"
- Check the file path is correct
- Ensure file is in Downloads folder
- Check file extension matches (.fbx, .glb, .ply, .obj)

### "Unsupported format"
- Ensure you exported as `.fbx`, `.glb`, `.ply`, or `.obj`
- `.spz` files are NOT supported

### "Unity project not found"
- Verify Unity project path: `C:\Users\sandr\My project`
- Ensure `Packages/manifest.json` exists

## Quick Reference

**Current file**: `Modern Tropical Luxury Residence.spz` ❌  
**Needed**: `Modern Tropical Luxury Residence.fbx` ✅ (or .glb, .ply)

**Export from**: World Labs Marble  
**Save to**: `C:\Users\sandr\Downloads\`  
**Import via**: `virtual_robotics.load_environment` tool

