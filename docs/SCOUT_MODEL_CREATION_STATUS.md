# Scout Model Creation Status

**Last Updated**: 2025-12-06  
**Status**: Ready to create - All specifications available

---

## ✅ Specifications Available

**From Pilot Labs Repository & Hardware Docs:**

### Dimensions (Confirmed)
- **Length**: 11.5 cm (0.115 m) - Front to back
- **Width**: 10 cm (0.10 m) - Side to side
- **Height**: 8 cm (0.08 m) - Base to top (without LiDAR)

### Features to Model
- ✅ **4× Mecanum Wheels**: Omnidirectional wheels at corners
  - Wheel diameter: ~4 cm (0.04 m)
  - Wheel width: ~1 cm (0.01 m)
  - Position: At each corner of the body

- ✅ **Camera**: Front-facing 2MP camera
  - Position: Front center, slightly above body
  - Size: ~1 cm cube
  - Field of View: 120° (wide-angle)

- ✅ **Body Shape**: Rectangular box
  - Low profile design
  - Flat top surface for LiDAR mounting
  - Rounded corners (optional detail)

### Sources
- ✅ `mcp-central-docs/docs/robotics/SCOUT_HARDWARE.md` - Complete hardware specs
- ✅ `external/moorebot-scout-sdk/` - Pilot Labs repository (no 3D models, but ROS code)
- ✅ Hardware documentation confirmed

---

## 🛠️ Creation Method

### Automated Creation (Recommended)

**Tool**: `robot_model_create`

**Command**:
```python
await robot_model_create(
    robot_type="scout",
    output_path="D:/Models/scout_model.fbx",
    format="fbx",
    dimensions={
        "length": 0.115,  # 11.5 cm
        "width": 0.10,    # 10 cm
        "height": 0.08    # 8 cm
    },
    create_textures=True,
    texture_style="realistic"
)
```

**What it does:**
1. Creates base body (cube: 0.115×0.10×0.08 m)
2. Adds 4 mecanum wheels (cylinders at corners)
3. Adds camera (small cube on front)
4. Creates textures via gimp-mcp (if available)
5. Applies materials
6. Exports to FBX

---

## 📋 Next Steps

1. **Mount blender-mcp and gimp-mcp** (if not already mounted)
2. **Call `robot_model_create` tool** with Scout specifications
3. **Verify model** in Blender or Unity
4. **Import to Unity** using `robot_model_import`
5. **Create vbot** using `vbot_crud`

---

## 📐 Detailed Specifications

### Wheel Positions (from body center)
- **Front-Left**: (-0.045, 0.045, 0.02) m
- **Front-Right**: (0.045, 0.045, 0.02) m
- **Back-Left**: (-0.045, -0.045, 0.02) m
- **Back-Right**: (0.045, -0.045, 0.02) m

### Camera Position
- **Location**: (0.0575, 0, 0.04) m (front center, at body height)
- **Size**: 0.01×0.01×0.01 m cube

### Body
- **Center**: (0, 0, 0.04) m (half height)
- **Dimensions**: 0.115×0.10×0.08 m

---

**Status**: ✅ All information available, ready to create Scout model!

