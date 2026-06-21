# Creating Scout 3D Model

**Last Updated**: 2025-12-06  
**Status**: Ready to create

---

## 📐 Scout Specifications (from Pilot Labs)

**Dimensions:**
- Length: 11.5 cm (0.115 m) - Front to back
- Width: 10 cm (0.10 m) - Side to side  
- Height: 8 cm (0.08 m) - Base to top (without LiDAR)

**Features:**
- 4× Mecanum wheels (omnidirectional)
- Camera on front (2MP, 1080p, 120° FOV)
- Flat top surface for LiDAR mounting
- Low profile design

**Wheel Specifications:**
- 4 wheels positioned at corners
- Wheel diameter: ~4 cm (0.04 m)
- Wheel width: ~1 cm (0.01 m)

**Camera:**
- Front-facing
- Position: Front center, slightly above body
- Size: ~1 cm cube

---

## 🛠️ Creation Methods

### Method 1: Using robot_model_create Tool (Automated)

```python
# Via robotics-mcp server
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

**What this does:**
1. Uses `blender-mcp` to create base body (cube: 0.115×0.10×0.08 m)
2. Adds 4 mecanum wheels (cylinders at corners)
3. Adds camera (small cube on front)
4. Uses `gimp-mcp` to create realistic textures
5. Applies textures to model
6. Exports to FBX format

### Method 2: Manual Creation via Blender MCP

```python
# Step 1: Create body
blender_mesh(
    operation="create_cube",
    name="Scout_Body",
    location=[0, 0, 0.04],  # Center at half height
    scale=[0.115, 0.10, 0.08]  # Length × Width × Height
)

# Step 2: Create 4 wheels
wheel_positions = [
    [-0.045, 0.045, 0.02],   # Front-left
    [0.045, 0.045, 0.02],    # Front-right
    [-0.045, -0.045, 0.02],  # Back-left
    [0.045, -0.045, 0.02]    # Back-right
]

for i, pos in enumerate(wheel_positions):
    blender_mesh(
        operation="create_cylinder",
        name=f"Scout_Wheel_{i+1}",
        location=pos,
        scale=[0.02, 0.02, 0.005]  # Radius × Radius × Width
    )

# Step 3: Create camera
blender_mesh(
    operation="create_cube",
    name="Scout_Camera",
    location=[0.0575, 0, 0.04],  # Front center, at body height
    scale=[0.01, 0.01, 0.01]
)

# Step 4: Create textures (via gimp-mcp)
gimp_create_image(
    width=1024,
    height=1024,
    name="scout_texture"
)

# Apply realistic metal/plastic texture
gimp_filter(
    filter_name="noise",
    image_name="scout_texture",
    amount=0.1
)

# Export texture
gimp_export(
    image_name="scout_texture",
    filepath="D:/Textures/scout_texture.png",
    format="png"
)

# Step 5: Apply material in Blender
blender_material(
    operation="create_material",
    name="Scout_Material",
    texture_path="D:/Textures/scout_texture.png"
)

# Step 6: Export to FBX
blender_export(
    filepath="D:/Models/scout_model.fbx",
    file_format="fbx",
    use_selection=False,
    apply_modifiers=True
)
```

---

## 📦 Output Files

**Model:**
- `D:/Models/scout_model.fbx` - Main 3D model

**Textures:**
- `D:/Textures/scout_texture.png` - Base texture (1024×1024)

---

## ✅ Verification Checklist

- [ ] Body dimensions correct (11.5×10×8 cm)
- [ ] 4 wheels positioned correctly at corners
- [ ] Camera on front center
- [ ] Textures applied
- [ ] Model exports to FBX successfully
- [ ] Model can be imported into Unity
- [ ] Model scales correctly (1 unit = 1 meter)

---

## 🎮 Next Steps After Creation

1. **Import to Unity:**
   ```python
   await robot_model_import(
       robot_type="scout",
       model_path="D:/Models/scout_model.fbx",
       format="fbx",
       platform="unity",
       project_path="D:/Projects/UnityRobots"
   )
   ```

2. **Create Vbot:**
   ```python
   await vbot_crud(
       operation="create",
       robot_type="scout",
       platform="unity",
       model_path="Assets/Models/scout_model.fbx",
       position={"x": 0.0, "y": 0.0, "z": 0.0},
       scale=1.0
   )
   ```

---

**Ready to create!** Use `robot_model_create` tool or manual Blender MCP workflow above.

