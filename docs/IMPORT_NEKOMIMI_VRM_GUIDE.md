# Import Nekomimi-chan.vrm into Unity

**High-priority VRM avatar import guide for existing VRoid Studio model**

[![VRM Import](https://img.shields.io/badge/VRM-Import-blue)](README.md)
[![Unity](https://img.shields.io/badge/Unity-URP-green)](README.md)
[![Priority](https://img.shields.io/badge/Priority-High-red)](README.md)

---

## 🎯 **Mission: Import Nekomimi-chan.vrm**

**Import the existing VRoid Studio VRM model into Unity for virtual robotics testing.**

### **File Location:**
- **Primary:** `avatar-mcp/models/Nekomimi-chan.vrm`
- **Backup:** `avatar-mcp/examples/Nekomimi-chan.vrm`
- **Test:** `unity3d-mcp/tests/fixtures/Nekomimi-chan.vrm`

---

## 🔧 **Step-by-Step Import Process**

### **Phase 1: Verify Unity Setup**

#### **1. Check Unity Project Requirements**
```bash
# Ensure your Unity project has:
# - Unity 2021.3+ (LTS recommended)
# - URP (Universal Render Pipeline) installed
# - UniVRM package installed
```

#### **2. Install UniVRM (if needed)**
```csharp
// Use unity3d-mcp tool to install UniVRM
// Command: install_univrm(project_path="path/to/your/unity/project")
```

#### **3. Verify UniVRM Installation**
```csharp
// Check if UniVRM is properly installed
// Look for: VRM menu in Unity toolbar
// Look for: VRMImporter scripts in project
```

### **Phase 2: Import the VRM File**

#### **Option A: Using Unity3D-MCP (Recommended)**

**Automated import with proper Unity integration:**
```python
# Use the unity3d-mcp import tool
from unity3d_mcp import import_vrm_to_unity

# Import Nekomimi-chan.vrm
result = import_vrm_to_unity(
    vrm_path="D:/Dev/repos/avatar-mcp/models/Nekomimi-chan.vrm",
    project_path="D:/Your/Unity/Project/Path",
    optimize_for_vrchat=True,  # Enable if targeting VRChat
    generate_prefab=True       # Create reusable prefab
)

print(f"Import result: {result}")
```

#### **Option B: Manual Unity Import**

**If MCP tools aren't available:**
```
1. Open Unity project
2. Drag Nekomimi-chan.vrm into Assets folder
3. Wait for UniVRM import dialog
4. Configure import settings:
   ├── [✓] Extract Textures
   ├── [✓] Extract Materials
   ├── [✓] Extract Meshes
   ├── [✓] Generate Animation Clips
   ├── [✓] Generate MToon Materials (for VRM)
```

### **Phase 3: Configure Avatar**

#### **1. Set Up Humanoid Avatar**
```
1. Select imported VRM model in Project window
2. Go to Rig tab in Inspector
3. Set Animation Type: Humanoid
4. Click Apply
5. Configure bone mappings (should auto-detect)
```

#### **2. VRM Configuration**
```
1. Select the VRM model
2. Open VRM Meta window (VRM -> VRM Meta)
3. Verify:
   ├── Title: Nekomimi-chan
   ├── Author: VRoid Studio
   ├── Version: 1.0
   ├── Allowed User: Everyone
   ├── License: Redistribution_Prohibited
```

#### **3. Material Setup**
```
1. VRM models use MToon shader by default
2. For URP compatibility:
   ├── Convert materials to URP/Lit
   ├── Or keep MToon for VR applications
   ├── Test rendering in Scene view
```

### **Phase 4: Animation & Rigging**

#### **1. Avatar Mask Setup**
```
1. Create new Avatar Mask (Assets → Create → Avatar Mask)
2. Configure body parts for animation
3. Useful for robot avatar control
```

#### **2. Animator Controller**
```
1. Create Animator Controller
2. Add humanoid animations
3. Set up state machine for robot behaviors
4. Connect to avatar-mcp for programmatic control
```

#### **3. Blend Shapes (Facial Animation)**
```
1. VRM models include blend shapes for facial expressions
2. Available shapes: Joy, Angry, Sorrow, Fun
3. Perfect for robot "personality" expressions
```

### **Phase 5: Integration with Robotics**

#### **1. Robot Avatar Setup**
```csharp
// Attach to VRM GameObject for robot control
using UnityEngine;
using VRM;

public class RobotAvatarController : MonoBehaviour
{
    private VRMBlendShapeProxy blendShapeProxy;
    private Animator animator;

    void Start()
    {
        // Get VRM components
        blendShapeProxy = GetComponent<VRMBlendShapeProxy>();
        animator = GetComponent<Animator>();

        // Set up for robot control
        ConfigureForRobotics();
    }

    void ConfigureForRobotics()
    {
        // Disable physics (robots don't need cloth/hair physics)
        var vrmMeta = GetComponent<VRMMeta>();
        // Configure as mechanical avatar

        // Set up control mappings
        MapInputsToAvatar();
    }

    void MapInputsToAvatar()
    {
        // Map robot sensors to avatar expressions
        // - Battery low → Sad expression
        // - Obstacle detected → Alert expression
        // - Task complete → Happy expression
    }

    // Robot control methods
    public void SetExpression(string emotion)
    {
        switch (emotion) {
            case "happy":
                blendShapeProxy.SetValue("Joy", 1.0f);
                break;
            case "alert":
                blendShapeProxy.SetValue("Fun", 1.0f);
                break;
            case "sad":
                blendShapeProxy.SetValue("Sorrow", 1.0f);
                break;
        }
    }

    public void SetMovement(Vector3 direction, float speed)
    {
        // Control avatar walking/running animation
        animator.SetFloat("Speed", speed);
        // Map direction to avatar rotation
    }
}
```

#### **2. Avatar-MCP Integration**
```python
# Connect to avatar-mcp for advanced control
import asyncio
from avatar_mcp import AvatarController

async def setup_robot_avatar():
    controller = AvatarController()

    # Load Nekomimi-chan VRM
    avatar_id = await controller.load_vrm_avatar(
        vrm_path="avatar-mcp/models/Nekomimi-chan.vrm",
        avatar_name="NekoRobot"
    )

    # Configure for robotics
    await controller.configure_avatar(
        avatar_id=avatar_id,
        mode="robot",  # Disable organic movements
        expressions=["happy", "alert", "sad", "neutral"]
    )

    return avatar_id

# Use in robot control loop
async def robot_control_loop():
    avatar_id = await setup_robot_avatar()

    while True:
        # Robot logic
        if obstacle_detected():
            await avatar_controller.set_expression(avatar_id, "alert")
        elif task_completed():
            await avatar_controller.set_expression(avatar_id, "happy")

        await asyncio.sleep(0.1)
```

### **Phase 6: Testing & Validation**

#### **1. Basic Functionality Test**
```
1. Add VRM to scene
2. Play mode test:
   ├── Model renders correctly
   ├── Animations play
   ├── Materials look good
   ├── No console errors
```

#### **2. Robot Integration Test**
```
1. Attach RobotAvatarController script
2. Test expression changes
3. Test movement animations
4. Verify avatar-mcp connection
```

#### **3. Performance Check**
```
1. Monitor frame rate
2. Check draw calls
3. Optimize materials if needed
4. Test on target hardware
```

---

## 🐱 **Nekomimi-chan VRM Specifications**

### **Model Details:**
- **Source:** VRoid Studio export
- **Format:** VRM 1.0 (glTF 2.0 based)
- **Rig:** Full humanoid bone structure
- **Materials:** MToon shader (VRM standard)
- **Blend Shapes:** Joy, Angry, Sorrow, Fun
- **Textures:** Embedded in VRM file
- **File Size:** ~50-100MB (with textures)

### **Unity Compatibility:**
- **Unity Version:** 2021.3+ recommended
- **Render Pipeline:** URP/Lit or MToon
- **Platform:** Windows/Mac/Linux
- **VR Support:** Compatible with VRChat, Resonite
- **Animation:** Humanoid animations supported

### **Robot Use Cases:**
- **Avatar representation** of robot state
- **Visual feedback** for robot actions
- **Human-robot interaction** interface
- **Social robotics** demonstrations
- **VR testing** of robot behaviors

---

## 🔄 **Quick Import Checklist**

### **Pre-Import:**
- [ ] Unity project created with URP
- [ ] UniVRM package installed
- [ ] VRM file located (`avatar-mcp/models/Nekomimi-chan.vrm`)

### **Import Steps:**
- [ ] Drag VRM into Unity Assets
- [ ] Configure import settings
- [ ] Set up humanoid rig
- [ ] Verify VRM meta data
- [ ] Test basic animation

### **Robot Integration:**
- [ ] Attach RobotAvatarController
- [ ] Configure expressions
- [ ] Set up avatar-mcp connection
- [ ] Test sensor-to-expression mapping

### **Validation:**
- [ ] Model renders correctly
- [ ] Animations work
- [ ] Materials compatible
- [ ] Performance acceptable

---

## 📚 **Resources & Troubleshooting**

### **Official Documentation:**
- **UniVRM GitHub:** https://github.com/vrm-c/UniVRM
- **VRM Specification:** https://vrm.dev/
- **Unity VRM Guide:** https://docs.vrm.dev/

### **Common Issues:**

#### **"Shader not found" errors:**
```
Solution: Convert MToon materials to URP/Lit
Or: Install MToon shader package
```

#### **Rigging failures:**
```
Solution: Manually map humanoid bones
Check: Bone naming conventions match Unity humanoid
```

#### **Import hangs:**
```
Solution: Restart Unity
Check: Sufficient RAM (VRM files can be large)
```

#### **Avatar-mcp connection fails:**
```
Solution: Verify avatar-mcp server running
Check: Correct avatar_id from load_vrm_avatar()
```

### **Performance Tips:**
- **LOD system** for distance culling
- **Texture compression** for mobile platforms
- **Animation culling** when avatar not visible
- **Shader variants** optimization

---

## 🎯 **Success Criteria**

### **Basic Import:**
- ✅ VRM loads into Unity without errors
- ✅ Model renders in Scene view
- ✅ Basic humanoid animations work

### **Robot Integration:**
- ✅ Avatar expressions respond to robot state
- ✅ Avatar movement matches robot navigation
- ✅ Avatar-mcp controls work reliably

### **Performance:**
- ✅ 60+ FPS in target environment
- ✅ No memory leaks during extended use
- ✅ Smooth animation transitions

---

**This guide provides everything needed to successfully import Nekomimi-chan.vrm and integrate it with your virtual robotics system!** 🐱🤖
