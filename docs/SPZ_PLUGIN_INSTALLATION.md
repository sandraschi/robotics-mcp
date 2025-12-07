# SPZ Plugin Installation Guide

**Date**: 2025-12-07  
**Status**: ⚠️ No Official Plugin Available

## Important Note

**There is NO official Unity plugin for `.spz` files.** Adobe's `.spz` format (compressed Gaussian splats) is not natively supported by Unity, and no third-party plugin currently exists.

## Available Solutions

### Option 1: Re-export from Marble (Recommended) ⭐

**Best solution** - Get the format Unity needs directly:

1. Open your Marble project in World Labs
2. Export as **`.ply`** (for Gaussian splats) or **`.fbx`/`.glb`** (for meshes)
3. Use the exported file directly with `unity3d-mcp`

**Why this is best**:
- No conversion needed
- Direct compatibility
- Full quality preserved
- No plugins required (for mesh formats)

### Option 2: Install Gaussian Splatting Plugin (for .ply files)

If you re-export as `.ply` from Marble:

```python
# Install Gaussian Splatting plugin via unity3d-mcp
await client.call_tool(
    "unity_install_gaussian_splatting",
    {
        "project_path": "C:\\Users\\sandr\\My project"
    }
)
```

**Package**: `aras-p/UnityGaussianSplatting`  
**Git**: `https://github.com/aras-p/UnityGaussianSplatting.git`

### Option 3: Use Mesh Format (Best for Robotics)

For robot navigation and testing, **mesh formats are better**:

1. Export from Marble as **`.fbx`** or **`.glb`**
2. Full Unity support (no plugins needed)
3. Better for NavMesh and navigation
4. Better performance

## SPZ Converter Tool

The `spz_converter` tool provides:

### Check Support
```python
await spz_converter(operation="check_spz_support")
```

### Extract Info
```python
await spz_converter(
    operation="extract_spz_info",
    spz_path="C:\\Users\\sandr\\Downloads\\Modern Tropical Luxury Residence.spz"
)
```

### Attempt Conversion
```python
await spz_converter(
    operation="convert_spz",
    spz_path="C:\\Users\\sandr\\Downloads\\Modern Tropical Luxury Residence.spz",
    output_path="C:\\Output\\residence.ply",
    output_format="ply"
)
```

**Note**: Conversion requires Adobe's spz-tools or Python library, which may not be available.

### Install Unity Plugin (Information Only)
```python
await spz_converter(
    operation="install_unity_spz_plugin",
    unity_project_path="C:\\Users\\sandr\\My project"
)
```

**Note**: This will return information about alternatives, as no plugin exists.

## Why No Plugin?

1. **`.spz` is Adobe's proprietary compressed format**
2. **No official Unity support** from Adobe
3. **No community plugins** have been created
4. **Re-exporting is simpler** than building a converter

## Recommended Workflow

### For Your Use Case (Robot Navigation):

1. **Re-export from Marble as `.fbx` or `.glb`**
2. **Import to Unity** via `virtual_robotics.load_environment`
3. **Add NavMesh** for pathfinding
4. **Test robot navigation**

**Why mesh is better**:
- NavMesh requires mesh geometry
- Better performance
- No special plugins
- Easier to work with

## References

- [Adobe .spz format](https://github.com/adobe/spz) - C++ library (no Unity plugin)
- [Unity Gaussian Splatting](https://github.com/aras-p/UnityGaussianSplatting) - For .ply files
- [unity3d-mcp World Labs docs](../unity3d-mcp/src/unity3d_mcp/worldlabs/__init__.py)

## Summary

**There is no Unity plugin for `.spz` files.** The recommended solution is to **re-export from Marble** as:
- **`.fbx`/`.glb`** for robotics/navigation (best option)
- **`.ply`** for visual quality (requires Gaussian Splatting plugin)

The `spz_converter` tool provides information and conversion attempts, but re-exporting is the most reliable solution.

