# Marble .spz File Support

**Date**: 2025-12-07  
**Status**: ⚠️ Requires Conversion

## Issue

The Marble file `Modern Tropical Luxury Residence.spz` (31MB) is a **compressed Gaussian splat file** using Adobe's `.spz` format. Unity3D does **NOT** natively support `.spz` format.

## Current Support

### unity3d-mcp Support
- ✅ Supports `.ply` (Gaussian splat format)
- ✅ Supports `.splat` (Gaussian splat format)
- ❌ Does **NOT** support `.spz` (compressed format)

### Unity Gaussian Splatting Plugin
- **Package**: `aras-p/UnityGaussianSplatting` (via GitHub)
- **Supported formats**: `.ply`, `.splat`
- **NOT supported**: `.spz` (compressed format)

## Solution Options

### Option 1: Re-export from Marble as .ply (Recommended) ⭐

**Best option** - Get the format Unity needs directly:

1. Open Marble project in World Labs
2. Export as **`.ply`** instead of `.spz`
3. Use `.ply` file directly with `unity3d-mcp`

**Advantages**:
- No conversion needed
- Direct compatibility
- Full quality preserved

### Option 2: Convert .spz to .ply

The `.spz` format requires conversion. Options:

1. **Adobe's spz-tools** (if available):
   ```bash
   # Would need Adobe's converter tool
   spz-decompress input.spz output.ply
   ```

2. **Marble Export Tool**:
   - Check if Marble has export to `.ply` option
   - Re-export from Marble interface

3. **Custom Converter**:
   - Use Adobe's C++ library: https://github.com/adobe/spz
   - Build converter tool
   - Convert `.spz` → `.ply`

### Option 3: Use Mesh Export Instead

If Gaussian splats are problematic:

1. Export from Marble as **mesh** (`.fbx`, `.glb`, `.obj`)
2. Use mesh format (fully supported by Unity)
3. **Trade-off**: Less visual quality, but better performance and compatibility

**Advantages**:
- Full Unity support (no plugins needed)
- Better performance
- Easier to work with (NavMesh, colliders, etc.)

## Required Unity Plugin (for .ply/.splat)

If using Gaussian splats (`.ply` or `.splat`):

**Package**: `com.aras-p.gaussian-splatting`  
**Git**: `https://github.com/aras-p/UnityGaussianSplatting.git`

**Install via unity3d-mcp**:
```python
await client.call_tool(
    "unity_install_gaussian_splatting",
    {
        "project_path": "C:\\Users\\sandr\\My project"
    }
)
```

**Or manually**:
1. Open Unity Package Manager
2. Click `+` → `Add package from git URL`
3. Enter: `https://github.com/aras-p/UnityGaussianSplatting.git`
4. Wait for import

## Recommended Workflow

### For Navigation/Robotics Testing:

**Use Mesh Export** (Option 3) - Better for robotics:
1. Export from Marble as `.fbx` or `.glb`
2. Import to Unity (native support)
3. Add NavMesh for pathfinding
4. Add colliders for physics
5. Test robot navigation

**Why mesh is better for robotics**:
- NavMesh requires mesh geometry
- Colliders work better with meshes
- Performance is better for real-time navigation
- No special plugins needed

### For Visual Quality:

**Use .ply Export** (Option 1):
1. Re-export from Marble as `.ply`
2. Install Gaussian Splatting plugin
3. Import `.ply` via `unity3d-mcp`
4. Use for visual reference/rendering

## Next Steps

1. ✅ Identified `.spz` format issue
2. ⏳ **Re-export from Marble as `.ply` or `.fbx`**
3. ⏳ If using `.ply`, install Gaussian Splatting plugin
4. ⏳ Import to Unity via `virtual_robotics.load_environment`
5. ⏳ For navigation, use mesh format (`.fbx`/`.glb`)

## References

- [Adobe .spz format](https://github.com/adobe/spz) - C++ library for .spz
- [Unity Gaussian Splatting](https://github.com/aras-p/UnityGaussianSplatting) - Unity plugin
- [unity3d-mcp World Labs docs](../unity3d-mcp/src/unity3d_mcp/worldlabs/__init__.py)

## Summary

**For your use case (robot navigation in apartment)**:
- **Best**: Export from Marble as **`.fbx` or `.glb`** (mesh)
- **Why**: NavMesh, colliders, and navigation work better with meshes
- **Alternative**: Export as **`.ply`** if you need visual quality (requires plugin)

The `.spz` file cannot be used directly - it needs conversion or re-export.
