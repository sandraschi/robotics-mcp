# Adding WorldLabs Environment to Unity Scene

**Date**: 2025-12-02  
**Status**: Assets imported ✅ | Need to add to scene ⏳

## Quick Steps

### Step 1: Add Collider GLB (Required for Navigation)

1. **In Unity Project Window**:
   - Navigate to `Assets/WorldLabs/`
   - Find `Modern Tropical Luxury Residence_collider.glb` (or similar name)
   - **Drag it** from Project window to **Scene Hierarchy** (left panel)

2. **Position the Collider**:
   - Select the GameObject in Hierarchy
   - In Inspector, set Transform:
     - **Position**: `(0, 0, 0)`
     - **Rotation**: `(0, 0, 0)`
     - **Scale**: `(1, 1, 1)`

3. **Verify it's in Scene**:
   - You should see the living room geometry in Scene view
   - If not visible, check:
     - Camera position (might be looking at wrong angle)
     - Scale (might be too small/large)
     - Mesh Renderer component is enabled

### Step 2: Add PLY Splats (Optional, for Visuals)

**Note**: PLY splats require Gaussian Splatting plugin (already installed).

1. **In Unity Project Window**:
   - Navigate to `Assets/WorldLabs/`
   - Find the PLY files (e.g., `cafcb2c0-c073-435e-adc5-406e6588e0db_ceramic_500k.ply`)
   - **Drag them** to Scene Hierarchy

2. **Position Splats**:
   - Set Transform Position to `(0, 0, 0)` (same as collider)
   - They should align with the collider geometry

3. **If Splats Don't Render**:
   - Check if Gaussian Splatting plugin is installed (Window > Package Manager)
   - PLY files may need specific material/shader
   - Check Unity Console for import errors

### Step 3: Verify in Scene View

1. **Open Scene View**:
   - Click on "Scene" tab (next to "Game" tab at top)
   - Or: Window > General > Scene

2. **Frame the View**:
   - Select the collider GameObject in Hierarchy
   - Press `F` key (frames selected object)
   - Or: Right-click in Scene view > Frame Selected

3. **Check Camera**:
   - Select Main Camera in Hierarchy
   - In Scene view, you should see the camera preview
   - Adjust camera position if needed:
     - Position: `(0, 5, -10)` (looking at origin)
     - Rotation: `(15, 0, 0)` (slight downward angle)

### Step 4: Add Lighting (If Needed)

1. **Check for Lights**:
   - Look in Hierarchy for "Directional Light"
   - If missing: GameObject > Light > Directional Light

2. **Adjust Light**:
   - Position: `(0, 10, 0)`
   - Rotation: `(50, -30, 0)` (angled down toward scene)
   - Intensity: `1.0` (adjust as needed)

## Troubleshooting

### Nothing Visible in Scene

- **Check Scale**: Collider might be tiny or huge
  - Try setting Scale to `(10, 10, 10)` or `(0.1, 0.1, 0.1)`
- **Check Position**: Object might be far from camera
  - Reset Position to `(0, 0, 0)`
- **Check Mesh Renderer**: Ensure component is enabled
  - Select GameObject > Inspector > Mesh Renderer > Checkbox enabled
- **Check Camera**: Camera might be looking wrong direction
  - Select Main Camera > Inspector > Transform
  - Set Position: `(0, 5, -10)`, Rotation: `(0, 0, 0)`

### Collider Not Showing

- **Check Import Settings**:
  - Select GLB file in Project window
  - Inspector > Model tab > Scale Factor: `1`
  - Inspector > Materials tab > Material Creation Mode: Standard
- **Check Mesh**:
  - Select GameObject in Hierarchy
  - Inspector > Mesh Filter > Mesh should be assigned
  - Inspector > Mesh Renderer > Material should be assigned

### PLY Splats Not Rendering

- **Check Plugin**:
  - Window > Package Manager
  - Look for "Gaussian Splatting" package
  - Should show as installed
- **Check Import**:
  - PLY files may still be importing (check progress bar)
  - Check Unity Console for errors
- **Check Material**:
  - PLY splats may need specific shader
  - Check GameObject > Inspector > Material settings

### Scene View is Empty

- **Check Scene Tab**: Make sure you're in Scene view, not Game view
- **Frame All**: Right-click in Scene view > Frame All
- **Reset View**: Scene view toolbar > Click "2D" to toggle, then back to 3D
- **Check Layers**: Scene view toolbar > Layers dropdown > Ensure "Default" is checked

## Quick Reference

**Unity Project**: `C:\Users\sandr\My project`  
**Assets Location**: `Assets/WorldLabs/`  
**Collider GLB**: `Modern Tropical Luxury Residence_collider.glb`  
**PLY Splats**: Files with `.ply` extension in `Assets/WorldLabs/`

**Transform Settings**:
- Position: `(0, 0, 0)`
- Rotation: `(0, 0, 0)`
- Scale: `(1, 1, 1)` (adjust if needed)

**Camera Settings** (if needed):
- Position: `(0, 5, -10)`
- Rotation: `(15, 0, 0)`

---

**Austrian Precision**: Drag from Project to Hierarchy, set Transform, frame view with F key! 🇦🇹🏠

