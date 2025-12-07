# Unity FBX Import - Showing Only Cube Fix

## Problem

Unity was showing only a cube instead of the full Scout model with all components (body, 4 wheels, camera, mounting plate).

## Root Cause

The FBX export was running in a separate Blender instance that didn't have the Scout objects loaded. The export script now loads the `.blend` file first before exporting.

## Solution

The export handler now:
1. Loads the `.blend` file before exporting
2. Selects all mesh objects from the loaded scene
3. Exports all objects to FBX

## Verification

The FBX now contains all 7 objects:
- `scout_body` (main body)
- `scout_wheel_1` through `scout_wheel_4` (4 mecanum wheels)
- `scout_camera` (front camera)
- `scout_mounting_plate` (top plate for LiDAR)

File size: 39.8 KB (increased from 14.9 KB when only 1 object was exported)

## Next Steps in Unity

1. **Delete the old import** (if it exists):
   - In Unity Project window, delete `Assets/Models/scout_model.fbx`
   - Or right-click > Reimport

2. **The updated FBX has been copied** to:
   - `C:\Users\sandr\My project\Assets\Models\scout_model.fbx`

3. **Unity will auto-reimport**:
   - Unity should detect the file change and reimport automatically
   - Check Unity Console for import status

4. **Expand the hierarchy**:
   - In Unity Project window, click the arrow next to `scout_model.fbx`
   - You should see all 7 objects listed
   - Drag the root `scout_model` to Scene (not individual objects)

5. **Create Prefab**:
   - Drag the imported model from Project to Scene
   - Drag from Scene back to Project to create Prefab
   - Name it `Scout` or `ScoutPrefab`

## If Still Showing Only Cube

1. **Check Import Settings**:
   - Select `scout_model.fbx` in Project window
   - In Inspector, check "Model" tab
   - Ensure "Scale Factor" is 1
   - Check "Meshes" tab - should show 7 meshes

2. **Try Manual Reimport**:
   - Right-click `scout_model.fbx` in Project
   - Select "Reimport"

3. **Check Hierarchy**:
   - Expand `scout_model.fbx` in Project window
   - You should see all 7 objects listed
   - If only 1 object, the FBX might not have updated - check file timestamp

4. **Verify FBX Contents**:
   - Run: `python scripts/verify_fbx_contents.py`
   - Should show "SUCCESS: All Scout components found in FBX!"

