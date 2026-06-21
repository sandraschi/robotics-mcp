# How to Render PLY Gaussian Splats in Unity

**Date**: 2025-12-02  
**Status**: Plugin installed ✅ | Need to render splats ⏳

## Quick Steps to See Your Splats

### Step 1: Verify Plugin is Loaded

1. **Open Unity Editor**
2. **Window > Package Manager**
3. **In Packages dropdown**: Select "My Assets" or "In Project"
4. **Look for**: "Gaussian Splatting" or "com.aras-p.gaussian-splatting"
5. **Status should be**: "Installed" or "Downloading..."

**If not installed yet**:
- Wait for Unity to download the package (may take a few minutes)
- Check Console for any errors
- The package URL is: `https://github.com/aras-p/UnityGaussianSplatting.git`

### Step 2: Add Splat Renderer Script

1. **Copy Script to Unity**:
   ```powershell
   Copy-Item "D:\Dev\repos\robotics-mcp\Assets\Scripts\GaussianSplatRenderer.cs" -Destination "C:\Users\sandr\My project\Assets\Scripts\GaussianSplatRenderer.cs"
   ```

2. **In Unity**:
   - Unity should auto-compile the script
   - Check Console for compilation errors

### Step 3: Create Splat GameObject

1. **Create Empty GameObject**:
   - Right-click in Hierarchy → Create Empty
   - Name it `GaussianSplatRenderer` or `LivingRoomSplats`

2. **Add Component**:
   - Select the GameObject
   - Add Component → Scripts → GaussianSplatRenderer

3. **Configure PLY Path**:
   - In Inspector, find "PLY File Path"
   - Set to: `Assets/WorldLabs/Modern Tropical Luxury Residence/Splats/cafcb2c0-c073-435e-adc5-406e6588e0db_ceramic_500k.ply`
   - Or browse to the PLY file in Project window and drag it

4. **Set Transform**:
   - Position: `(0, 0, 0)`
   - Rotation: `(0, 0, 0)`
   - Scale: `(1, 1, 1)`

5. **Load the Splat**:
   - Right-click the component in Inspector
   - Select "Load Splat"
   - Or: The script will auto-load on Play mode

### Step 4: Manual Method (If Script Doesn't Work)

The aras-p/UnityGaussianSplatting package should provide components. Here's how to use them:

1. **Find the PLY File**:
   - In Project window: `Assets/WorldLabs/.../Splats/`
   - Select one of the `.ply` files

2. **Check Import Settings**:
   - Select the PLY file
   - Inspector should show import settings
   - If it shows "Gaussian Splat" importer, it's working!

3. **Drag to Scene**:
   - Drag the PLY file from Project to Scene Hierarchy
   - It should create a GameObject with the splat renderer

4. **If No Importer Shows**:
   - The package may not be fully loaded
   - Wait for Package Manager to finish
   - Restart Unity if needed

### Step 5: Alternative - Use Unity3D MCP Tool

You can also use the robotics-mcp tool to help set this up:

```python
# Check if plugin is installed
await client.call_tool(
    "unity_check_gaussian_splatting",
    {"project_path": "C:/Users/sandr/My project"}
)
```

## Troubleshooting

### Plugin Not Showing in Package Manager

- **Wait**: Package download can take 5-10 minutes
- **Check Internet**: Package downloads from GitHub
- **Check Console**: Look for package download errors
- **Manual Install**: 
  - Window > Package Manager
  - Click "+" button > Add package from git URL
  - Enter: `https://github.com/aras-p/UnityGaussianSplatting.git`

### PLY Files Not Importing

- **Check File Location**: Ensure PLY files are in `Assets/WorldLabs/...`
- **Check File Size**: Large PLY files (100+ MB) take time to import
- **Check Console**: Look for import errors
- **Re-import**: Right-click PLY file > Reimport

### Splats Not Rendering

- **Check Camera**: Ensure camera can see the splat position
- **Check Scale**: Splats might be tiny or huge - try different scales
- **Check Material**: Splats may need specific shader
- **Check Package**: Ensure Gaussian Splatting package is fully loaded

### Script Errors

- **Check Namespace**: The package component name might be different
- **Check Package Version**: Different versions may have different APIs
- **Check Console**: Look for specific error messages
- **Manual Setup**: Use the manual method above instead

## Package Documentation

The aras-p/UnityGaussianSplatting package should provide:
- A custom importer for `.ply` files
- A renderer component for Gaussian splats
- Shaders for rendering splats

**GitHub**: https://github.com/aras-p/UnityGaussianSplatting

## Quick Reference

**PLY File Locations**:
- `Assets/WorldLabs/Modern Tropical Luxury Residence/Splats/cafcb2c0-c073-435e-adc5-406e6588e0db_ceramic_500k.ply`
- `Assets/WorldLabs/Modern Tropical Luxury Residence/Splats/86b22779-6d6e-4382-a7d4-7d606b3d7611_ceramic.ply`

**Package**: `com.aras-p.gaussian-splatting`  
**Source**: `https://github.com/aras-p/UnityGaussianSplatting.git`

**Script Location**: `Assets/Scripts/GaussianSplatRenderer.cs`

---

**Austrian Precision**: Check Package Manager first, then drag PLY to scene, or use the script! 🇦🇹✨

