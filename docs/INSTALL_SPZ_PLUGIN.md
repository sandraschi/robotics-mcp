# Install SPZ Plugin - Quick Guide

**Date**: 2025-12-07

## TL;DR

**There is NO Unity plugin for `.spz` files.** However, you have options:

## Quick Solutions

### ✅ Option 1: Install Gaussian Splatting Plugin (for .ply files)

If you re-export from Marble as `.ply`:

```python
# Via unity3d-mcp (mounted in robotics-mcp)
await client.call_tool(
    "unity_install_gaussian_splatting",
    {
        "project_path": "C:\\Users\\sandr\\My project"
    }
)
```

**What this does**:
- Adds `com.aras-p.gaussian-splatting` package to Unity's `manifest.json`
- Unity will download and import the package automatically
- Enables rendering of `.ply` Gaussian splat files

### ✅ Option 2: Use Mesh Format (No Plugin Needed)

**Best for robotics** - Export from Marble as `.fbx` or `.glb`:
- Full Unity support (native)
- No plugins required
- Better for NavMesh and navigation
- Better performance

## Check Your .spz File

```python
# Check what conversion tools are available
await spz_converter(operation="check_spz_support")

# Get info about your .spz file
await spz_converter(
    operation="extract_spz_info",
    spz_path="C:\\Users\\sandr\\Downloads\\Modern Tropical Luxury Residence.spz"
)
```

## Recommended Action

**For robot navigation testing**:
1. Re-export from Marble as **`.fbx`** or **`.glb`**
2. Import to Unity (no plugins needed)
3. Add NavMesh
4. Test navigation

**For visual quality**:
1. Re-export from Marble as **`.ply`**
2. Install Gaussian Splatting plugin (see Option 1 above)
3. Import `.ply` file

## Summary

- ❌ No Unity plugin exists for `.spz` files
- ✅ Install Gaussian Splatting plugin for `.ply` files
- ✅ Use mesh formats (`.fbx`/`.glb`) - no plugins needed
- ✅ Re-export from Marble is the recommended solution

