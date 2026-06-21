# World Labs to Unity Integration Fix

**Resolving format incompatibilities between World Labs Marble exports and Unity splat plugins**

[![World Labs](https://img.shields.io/badge/World_Labs-Marble-blue)](README.md)
[![Unity](https://img.shields.io/badge/Unity-Gaussian_Splatting-green)](README.md)
[![Status](https://img.shields.io/badge/Status-Active_Fix-red)](README.md)

---

## 🎯 **The Core Problem**

World Labs Marble exports `.spz` files (Adobe's compressed Gaussian splat format), but Unity's Gaussian Splatting plugin expects `.ply` or `.splat` files. This creates a format incompatibility.

### **Current State:**
- ✅ Gaussian Splatting plugin installed in Unity
- ❌ `.spz` files not supported by Unity
- ❌ Re-exporting from Marble may not work perfectly
- ❌ Plugin may have version compatibility issues

---

## 🔧 **Step-by-Step Fix Process**

### **Phase 1: Verify Current Setup**

#### **1. Check Unity Package Versions**
```json
// manifest.json - Check these packages
{
  "dependencies": {
    "com.aras-p.gaussian-splatting": "1.0.0",  // May need update
    "com.unity.render-pipelines.universal": "12.1.7"  // URP version
  }
}
```

#### **2. Test Plugin Functionality**
```csharp
// Test script to verify Gaussian Splatting works
using UnityEngine;
using GaussianSplatting;

public class TestGaussianSplatting : MonoBehaviour
{
    void Start()
    {
        // Create test splat
        var splatRenderer = gameObject.AddComponent<GaussianSplatRenderer>();
        if (splatRenderer == null) {
            Debug.LogError("Gaussian Splatting plugin not working!");
        }
    }
}
```

### **Phase 2: Alternative Export Strategies**

#### **Option A: Re-export from Marble as .ply**
```
1. Open Marble project
2. Export → Gaussian Splats → .ply format
3. Note: May lose some quality vs .spz
4. Import .ply into Unity with Gaussian Splatting plugin
```

#### **Option B: Convert .spz to .ply**
```python
# Python script to convert .spz to .ply
import numpy as np
import open3d as o3d

def spz_to_ply(spz_path, ply_path):
    # Load .spz file (if reader exists)
    # Convert to Open3D point cloud
    # Save as .ply

    # Note: May require custom .spz reader
    pass
```

#### **Option C: Use Mesh Export Instead**
```
1. Export from Marble as .fbx or .glb (mesh format)
2. Better for navigation/collision detection
3. Worse visual quality than splats
4. Use standard Unity mesh import
```

### **Phase 3: Plugin Compatibility Fixes**

#### **Update Gaussian Splatting Plugin**
```bash
# In Unity Package Manager
1. Remove old com.aras-p.gaussian-splatting
2. Add latest version (check GitHub releases)
3. Restart Unity
4. Test with known-good .ply file
```

#### **Shader Compatibility Check**
```glsl
// Check if splat shader compiles
// Look for errors in Unity console:
// - Shader compilation errors
// - Missing shader variants
// - URP compatibility issues
```

### **Phase 4: Import Pipeline**

#### **Create Robust Import Script**
```csharp
using UnityEngine;
using UnityEditor;
using System.IO;

public class MarbleImportPipeline
{
    [MenuItem("Tools/Import Marble Environment")]
    static void ImportMarbleEnvironment()
    {
        string marblePath = EditorUtility.OpenFilePanel(
            "Select Marble Export",
            "",
            "ply,fbx,glb"
        );

        if (string.IsNullOrEmpty(marblePath)) return;

        string extension = Path.GetExtension(marblePath).ToLower();

        switch (extension) {
            case ".ply":
                ImportGaussianSplat(marblePath);
                break;
            case ".fbx":
            case ".glb":
                ImportMesh(marblePath);
                break;
            default:
                Debug.LogError("Unsupported format: " + extension);
                break;
        }
    }

    static void ImportGaussianSplat(string path)
    {
        // Use Gaussian Splatting plugin
        var splatObject = new GameObject("Marble Environment");
        var renderer = splatObject.AddComponent<GaussianSplatRenderer>();
        renderer.LoadSplatFile(path);
    }

    static void ImportMesh(string path)
    {
        // Standard Unity mesh import
        AssetDatabase.ImportAsset(path);
        var prefab = AssetDatabase.LoadAssetAtPath<GameObject>(path);
        PrefabUtility.InstantiatePrefab(prefab);
    }
}
```

---

## 🎨 **Improving Scout Vbot Detail & Color**

### **Current Scout Vbot Issues:**
- ❌ Low polygon count (blocky appearance)
- ❌ Missing texture details
- ❌ Limited color variety
- ❌ No material variations

### **Blender Improvement Pipeline:**

#### **Phase 1: Model Enhancement**
```
1. Import current Scout .fbx/.glb
2. Add subdivision surface modifier
3. Sculpt additional details:
   - Panel lines
   - Cable management
   - Sensor housings
   - Wheel treads
4. Add proper topology for deformation
```

#### **Phase 2: Texture & Material Creation**
```
1. UV unwrap model properly
2. Create PBR textures:
   - Base color (multiple color variants)
   - Metallic map
   - Roughness map
   - Normal map
   - Emission map (for lights)
3. Create material variants:
   - Default gray
   - Camouflage
   - Warning colors
   - Custom team colors
```

#### **Phase 3: Animation & Rigging**
```
1. Create armature for movable parts:
   - Wheels (rotation)
   - Neck/head (pan/tilt)
   - Arms/manipulators
   - Antennas/sensors
2. Add shape keys for deformation
3. Create walk cycle animation
```

### **Unity Integration:**

#### **LOD System for Performance**
```csharp
// Multiple detail levels for Scout vbot
public class ScoutLOD : MonoBehaviour
{
    public GameObject highDetailModel;
    public GameObject mediumDetailModel;
    public GameObject lowDetailModel;

    void Start()
    {
        // Distance-based LOD switching
        float distance = Vector3.Distance(transform.position, Camera.main.transform.position);

        highDetailModel.SetActive(distance < 10f);
        mediumDetailModel.SetActive(distance >= 10f && distance < 50f);
        lowDetailModel.SetActive(distance >= 50f);
    }
}
```

#### **Color Variants System**
```csharp
// Dynamic color changing for Scout vbot
public class ScoutColorManager : MonoBehaviour
{
    public enum ScoutColor { Default, Camo, Warning, Custom }
    public ScoutColor currentColor;

    private Material[] scoutMaterials;

    void Start()
    {
        scoutMaterials = GetComponentsInChildren<Renderer>()
            .Select(r => r.material)
            .ToArray();
    }

    public void SetColor(ScoutColor color)
    {
        currentColor = color;
        ApplyColorScheme();
    }

    void ApplyColorScheme()
    {
        Color primary, secondary, accent;

        switch (currentColor) {
            case ScoutColor.Default:
                primary = Color.gray;
                secondary = Color.black;
                accent = Color.blue;
                break;
            case ScoutColor.Camo:
                // Woodland camo pattern
                break;
            case ScoutColor.Warning:
                primary = Color.yellow;
                secondary = Color.black;
                accent = Color.red;
                break;
        }

        // Apply to materials
        foreach (var mat in scoutMaterials) {
            mat.SetColor("_BaseColor", primary);
            // Additional material properties
        }
    }
}
```

---

## 🔧 **Blender Workflow Enhancements**

### **Robot Model Creation Pipeline**

#### **1. Reference Gathering**
```
- Collect reference photos of real Scout
- Measure dimensions accurately
- Note material properties
- Document sensor placements
```

#### **2. Base Mesh Creation**
```
- Start with primitives (cubes, cylinders)
- Use boolean operations for complex shapes
- Maintain proper scale (real-world units)
- Create modular components
```

#### **3. Topology Optimization**
```
- Clean edge loops for deformation
- Avoid n-gons in important areas
- Use proper subdivision workflow
- Optimize for real-time rendering
```

#### **4. UV Mapping & Texturing**
```
- Plan UV layout for efficient use
- Create tileable textures
- Use PBR workflow
- Include detail maps for close-ups
```

#### **5. Rigging & Animation**
```
- Create control rig for easy posing
- Add constraints for realistic movement
- Bake animations for Unity
- Include blend shapes for damage states
```

### **Environment Creation for Testing**

#### **Living Room Test Environment**
```
1. Model furniture to scale
2. Add realistic lighting
3. Include navigation obstacles
4. Create test scenarios:
   - Object pickup/delivery
   - Person following
   - Collision avoidance
   - Path planning around furniture
```

#### **Export Pipeline for Unity**
```
1. Export as .fbx with proper settings
2. Include animation clips
3. Generate LOD variants
4. Create prefab with components
```

---

## 📊 **Success Metrics**

### **World Labs Import:**
- ✅ Environment loads in Unity without errors
- ✅ Visual quality acceptable for testing
- ✅ Navigation mesh generates properly
- ✅ Performance acceptable (30+ FPS)

### **Scout Vbot Improvements:**
- ✅ Higher polygon count (5x improvement)
- ✅ Multiple color/material variants
- ✅ Proper rigging for animation
- ✅ LOD system for performance
- ✅ Texture quality suitable for close-ups

### **Blender Workflow:**
- ✅ Modular components for easy modification
- ✅ PBR materials with proper Unity export
- ✅ Animation-ready rigs
- ✅ Optimized topology for real-time use

---

## 🔄 **Next Steps**

### **Immediate (This Week):**
1. Test current Marble export → Unity import
2. Identify specific error messages
3. Update Gaussian Splatting plugin if needed
4. Create minimal test scene

### **Short Term (2-4 Weeks):**
1. Successfully import living room environment
2. Improve Scout vbot base model in Blender
3. Add basic color variants
4. Test robot-environment interaction

### **Medium Term (1-2 Months):**
1. Full Scout vbot detail enhancement
2. Multiple environment scenarios
3. Performance optimization
4. Animation system completion

---

## 📚 **Resources**

### **World Labs Marble:**
- Export format documentation
- Community forums for Unity integration
- Alternative export tools

### **Unity Gaussian Splatting:**
- Plugin GitHub repository
- Troubleshooting guides
- Performance optimization tips

### **Blender to Unity:**
- Export best practices
- Material conversion guides
- Animation workflow tutorials

### **Scout Robot References:**
- Official Moorebot documentation
- Community modification guides
- 3D model repositories

---

**This systematic approach will resolve the format incompatibilities and significantly improve your virtual robotics testing environment!** 🚀🤖
