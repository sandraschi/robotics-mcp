# VRM vs Robot Models for Vbots

**Last Updated**: 2025-12-02  
**Status**: Analysis and Recommendations

---

## 🎯 Key Question: Should Robots Use VRM Format?

### Short Answer: **No, robots should NOT use VRM format**

**Why:**
- **VRM is humanoid-only**: VRM format requires a standardized humanoid bone structure (head, spine, arms, legs, hands, feet)
- **Scout is not humanoid**: Scout is a wheeled robot with mecanum wheels, no humanoid bones
- **VRM specification**: Based on glTF 2.0 but requires VRM humanoid bone mapping

### When VRM Makes Sense:
- ✅ **Humanoid robots** (Unitree G1, Atlas, ASIMO-style)
- ✅ **Anthropomorphic robots** (Robbie from Forbidden Planet - humanoid shape)
- ✅ **VTuber avatars** (VRoid Studio output)
- ✅ **VRChat human avatars**

### When VRM Does NOT Make Sense:
- ❌ **Wheeled robots** (Scout, Scout E)
- ❌ **Quadrupeds** (Unitree Go2 - has legs but not humanoid structure)
- ❌ **Non-humanoid robots** (drones, arms, custom robots)

---

## 📦 Recommended Formats for Robot Vbots

### For Unity3D/VRChat:
1. **FBX** (`.fbx`) - **BEST CHOICE**
   - Industry standard
   - Supports animations, materials, textures
   - Unity native support
   - VRChat compatible

2. **GLB** (`.glb`) - **MODERN ALTERNATIVE**
   - Based on glTF 2.0
   - Single file (includes textures)
   - Web-friendly
   - Unity supports via glTF importer

3. **OBJ** (`.obj`) - **SIMPLE FALLBACK**
   - Basic mesh only
   - No animations
   - No materials (separate MTL file)
   - Use only if nothing else available

### For Resonite:
- **VRM** - ✅ Works (Resonite imports VRM directly)
- **GLB** - ✅ Works (Resonite imports GLB directly)
- **FBX** - ❌ Not directly supported (need to convert)

---

## 🤖 Creating Scout as Vbot

### Option 1: Use FBX/GLB (Recommended)

**Workflow:**
1. **Obtain/Create Scout Model**:
   - Get from official sources (Pilot Labs, FCC filings)
   - Create in Blender from specs (11.5×10×8 cm)
   - Use Blender MCP to create from primitives

2. **Export to FBX**:
   ```python
   # Using blender-mcp
   blender_export(
       format="fbx",
       filepath="D:/Models/scout_model.fbx",
       use_selection=True
   )
   ```

3. **Import to Unity**:
   - Drag FBX into `Assets/Models/`
   - Unity auto-imports
   - Create prefab

4. **Use with vbot_crud**:
   ```python
   vbot_crud(
       operation="create",
       robot_type="scout",
       platform="unity",
       model_path="Assets/Models/scout_model.fbx",
       position={"x": 0.0, "y": 0.0, "z": 0.0}
   )
   ```

### Option 2: Create "Humanoid" Scout VRM (Not Recommended)

**Why this is problematic:**
- Scout has no humanoid bones (no head, arms, legs)
- Would need to "stretch" Scout to fit humanoid rig
- Would look weird and not represent actual Scout
- VRM validator would reject or warn

**If you really want VRM:**
1. Model Scout in Blender
2. Create humanoid armature (head, spine, arms, legs)
3. Map Scout mesh to humanoid bones (stretching required)
4. Export as VRM using Blender VRM add-on
5. **Result**: Weird humanoid-shaped Scout (not recommended)

---

## 🛠️ Current avatar-mcp Tools Analysis

### What avatar-mcp CAN Do:
- ✅ **Load VRM models** (`avatar_load`, `avatar_manager`)
- ✅ **Animate VRM avatars** (bone control, blend shapes)
- ✅ **Export to FBX/UnityPackage** (but NOT to VRM)
- ✅ **Manage VRM metadata**

### What avatar-mcp CANNOT Do:
- ❌ **Create VRM from scratch**
- ❌ **Convert non-humanoid models to VRM**
- ❌ **Export to VRM format** (only imports VRM)
- ❌ **Handle robot-specific models** (no robot bone structures)

---

## 🔧 Recommended Expansion: Robot Model Tools

### Option A: Expand avatar-mcp

**Add robot-specific tools:**
```python
# New tools needed in avatar-mcp
robot_model_load(
    model_path: str,
    robot_type: str,  # "scout", "go2", "custom"
    format: str  # "fbx", "glb", "obj"
)

robot_model_export(
    robot_id: str,
    format: str,  # "fbx", "glb", "vrm" (if humanoid)
    output_path: str
)

robot_to_vrm_convert(
    robot_model_path: str,
    humanoid_rig: bool = False,  # Create humanoid rig for VRM
    output_path: str
)
```

### Option B: Add to robotics-mcp (Better)

**Create robot-specific model tools:**
```python
# In robotics-mcp
robot_model_import(
    robot_type: str,
    model_path: str,
    format: Literal["fbx", "glb", "obj", "vrm"],
    platform: Literal["unity", "vrchat", "resonite"]
)

robot_model_export(
    robot_id: str,
    format: Literal["fbx", "glb", "obj"],
    output_path: str
)

robot_model_convert(
    source_path: str,
    source_format: str,
    target_format: str,
    target_path: str
)
```

**Why this is better:**
- Keeps robot-specific logic in robotics-mcp
- avatar-mcp stays focused on humanoid avatars
- Clear separation of concerns

---

## 🎨 Creating Scout Model: Recommended Workflow

### Step 1: Get/Create Scout 3D Model

**Using Blender MCP:**
```python
# Create Scout from primitives
blender_mesh(
    operation="create_cube",
    name="Scout_Body",
    location=(0, 0, 0),
    scale=(0.115, 0.10, 0.08)  # 11.5×10×8 cm in meters
)

# Add wheels (cylinders)
blender_mesh(
    operation="create_cylinder",
    name="Scout_Wheel_FL",
    location=(-0.05, 0.05, -0.04),
    scale=(0.02, 0.02, 0.01)  # Wheel dimensions
)

# ... repeat for 4 wheels

# Add camera (small cube)
blender_mesh(
    operation="create_cube",
    name="Scout_Camera",
    location=(0.0575, 0, 0.02),  # Front of robot
    scale=(0.01, 0.01, 0.01)
)
```

### Step 2: Export to FBX

```python
blender_export(
    format="fbx",
    filepath="D:/Models/scout_model.fbx",
    use_selection=False,  # Export all
    apply_modifiers=True
)
```

### Step 3: Use in Unity

```python
# Via vbot_crud
vbot_crud(
    operation="create",
    robot_type="scout",
    platform="unity",
    model_path="D:/Models/scout_model.fbx",
    position={"x": 0.0, "y": 0.0, "z": 0.0},
    scale=1.0
)
```

---

## 🎭 Special Case: Robbie from Forbidden Planet

**Robbie IS humanoid!** ✅

Robbie has:
- Humanoid shape (head, torso, arms, legs)
- Can be rigged with humanoid bones
- **CAN be converted to VRM**

**Workflow for Robbie VRM:**
1. Get Robbie 3D model (FBX/OBJ)
2. Import to Blender
3. Create humanoid armature
4. Rig Robbie mesh to armature
5. Export as VRM using Blender VRM add-on
6. Use with avatar-mcp tools

```python
# Robbie can use VRM!
vbot_crud(
    operation="create",
    robot_type="robbie",
    platform="unity",
    model_path="Assets/Models/robbie.vrm",  # VRM format OK!
    position={"x": 1.0, "y": 0.0, "z": 1.0}
)
```

---

## 📋 Summary & Recommendations

### For Scout (Wheeled Robot):
- ✅ **Use FBX/GLB format** (NOT VRM)
- ✅ **Use vbot_crud with `model_path`**
- ✅ **Create model in Blender** (use Blender MCP)
- ❌ **Don't try to make VRM** (not humanoid)

### For Robbie (Humanoid Robot):
- ✅ **Can use VRM format** (humanoid shape)
- ✅ **Use avatar-mcp tools** for VRM management
- ✅ **Or use FBX** (also works)

### For Unitree Go2 (Quadruped):
- ⚠️ **Technically not humanoid** (4 legs, no arms)
- ⚠️ **VRM might work** (if rigged as humanoid - weird)
- ✅ **Better: Use FBX** (preserves quadruped structure)

### For Unitree G1 (Humanoid):
- ✅ **Perfect for VRM** (humanoid with arms)
- ✅ **Use avatar-mcp VRM tools**
- ✅ **Or use FBX** (also works)

### Tool Expansion Needed:

**avatar-mcp expansion:**
- ❌ **NOT needed** for robot models (avatar-mcp is for humanoid avatars)
- ✅ **Keep as-is** (focused on VRM/humanoid avatars)

**robotics-mcp expansion:**
- ✅ **Add robot model import/export tools** (DONE)
- ✅ **Add robot model creation tool** (DONE - uses blender-mcp)
- ✅ **Support FBX/GLB/OBJ formats** (DONE)
- ✅ **Integrate gimp-mcp for textures** (DONE)
- ✅ **Optional: VRM export for humanoid robots only** (DONE)

**MCP Server Integration:**
- ✅ **blender-mcp**: For creating/editing 3D models, exporting FBX/GLB/OBJ
- ✅ **gimp-mcp**: For creating/editing textures and images
- ✅ **unity3d-mcp**: For importing models into Unity projects
- ✅ **avatar-mcp**: For VRM management (humanoid robots only)

---

## 🚀 Next Steps

1. ✅ **Create Scout FBX model** (using `robot_model_create` - automated with blender-mcp + gimp-mcp)
2. ✅ **Add robot model tools to robotics-mcp**:
   - ✅ `robot_model_import` (FBX/GLB/OBJ/VRM)
   - ✅ `robot_model_export` (FBX/GLB/OBJ)
   - ✅ `robot_model_create` (automated creation via blender-mcp + gimp-mcp)
   - ✅ `robot_model_convert` (format conversion via blender-mcp)
3. ✅ **Use FBX for Scout** (not VRM)
4. ✅ **Use VRM for Robbie** (humanoid, VRM works)
5. ✅ **Keep avatar-mcp focused** on humanoid avatars
6. ✅ **Integrate blender-mcp** for model creation/editing
7. ✅ **Integrate gimp-mcp** for texture creation/editing

---

**Austrian Precision**: Right tool for the right job - VRM for humans, FBX for robots! 🇦🇹🤖

