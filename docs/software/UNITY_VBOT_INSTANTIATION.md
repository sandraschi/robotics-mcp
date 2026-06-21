# Instantiating Virtual Robots (Vbots) in Unity3D

## Unity Terminology

### Core Concepts

- **Scene**: A Unity scene is a container for all GameObjects, lights, cameras, and other elements. Think of it as a "level" or "world" in your game.
- **GameObject**: The fundamental object in Unity. Everything in a scene is a GameObject (cameras, lights, 3D models, empty containers, etc.).
- **Transform**: Every GameObject has a Transform component that defines its **position** (x, y, z), **rotation** (x, y, z in degrees or quaternion), and **scale** (x, y, z multipliers).
- **Prefab**: A reusable GameObject template saved as an asset. Prefabs allow you to instantiate (spawn) the same object multiple times.
- **Instantiate**: The Unity function `GameObject.Instantiate()` that creates a copy of a GameObject or Prefab in the scene at runtime or in the editor.
- **Component**: Scripts and built-in behaviors attached to GameObjects (e.g., Rigidbody, Collider, MeshRenderer, custom scripts).
- **Asset**: Any file in the `Assets/` folder (models, textures, scripts, prefabs, etc.).

## Workflow: Instantiating a Vbot in Unity

### Step 1: Import the Robot Model

First, you need a 3D model of your robot (e.g., Scout, Robbie, Go2) as a mesh file:

**Supported formats:**
- `.fbx` (most common, supports animations)
- `.obj` (simple mesh, no animations)
- `.glb` / `.gltf` (modern format, good for web/VR)
- `.vrm` (for VR avatars, requires UniVRM package)

**Manual import:**
1. Drag the model file into Unity's `Assets/` folder
2. Unity automatically imports it
3. Select the imported model in the Project window
4. Configure import settings (scale, materials, animations)

**Via robotics-mcp:**
```python
# Use vbot_crud to create with model_path
vbot_crud(
    operation="create",
    robot_type="custom",
    platform="unity",
    model_path="D:/Models/scout_model.fbx",
    position={"x": 0.0, "y": 0.0, "z": 0.0},
    scale=1.0
)
```

### Step 2: Create a Prefab (Recommended)

**Why use a Prefab?**
- Reusable template
- Easy to instantiate multiple copies
- Changes to prefab affect all instances
- Can be instantiated at runtime

**Manual creation:**
1. Drag the imported model from `Assets/` into the **Scene Hierarchy**
2. Configure the GameObject (add components, set up materials, etc.)
3. Drag the configured GameObject from Hierarchy back into `Assets/Prefabs/` folder
4. This creates a `.prefab` file
5. Delete the GameObject from the scene (the prefab remains)

**Prefab structure for a robot:**
```
RobotPrefab (GameObject)
├── Model (GameObject with MeshRenderer, MeshFilter)
├── Collider (GameObject with BoxCollider/CapsuleCollider)
├── Rigidbody (Component for physics)
└── RobotController (Custom C# script component)
```

### Step 3: Instantiate the Prefab in the Scene

**Unity C# Script Method:**

Create a C# script `VbotSpawner.cs`:

```csharp
using UnityEngine;

public class VbotSpawner : MonoBehaviour
{
    [SerializeField] private GameObject robotPrefab;  // Assign prefab in Inspector
    [SerializeField] private Vector3 spawnPosition = Vector3.zero;
    [SerializeField] private float spawnScale = 1.0f;
    
    public GameObject SpawnRobot(string robotId, Vector3 position, float scale)
    {
        if (robotPrefab == null)
        {
            Debug.LogError("Robot prefab not assigned!");
            return null;
        }
        
        // Instantiate the prefab
        GameObject instance = Instantiate(robotPrefab, position, Quaternion.identity);
        
        // Set scale
        instance.transform.localScale = Vector3.one * scale;
        
        // Set name
        instance.name = robotId;
        
        // Add to scene root (or parent to specific GameObject)
        instance.transform.SetParent(null);
        
        Debug.Log($"Spawned robot: {robotId} at {position} with scale {scale}");
        
        return instance;
    }
    
    // Editor method for testing
    [ContextMenu("Spawn Test Robot")]
    void SpawnTestRobot()
    {
        SpawnRobot("vbot_test_01", spawnPosition, spawnScale);
    }
}
```

**Using Unity Editor API (Editor Script):**

For editor-time instantiation (via `execute_unity_method`):

```csharp
using UnityEngine;
using UnityEditor;

public static class VbotEditorSpawner
{
    public static void SpawnVbot(string robotId, string prefabPath, Vector3 position, float scale)
    {
        // Load prefab from Assets
        GameObject prefab = AssetDatabase.LoadAssetAtPath<GameObject>(prefabPath);
        
        if (prefab == null)
        {
            Debug.LogError($"Prefab not found: {prefabPath}");
            return;
        }
        
        // Instantiate in current scene
        GameObject instance = PrefabUtility.InstantiatePrefab(prefab) as GameObject;
        
        // Set transform
        instance.transform.position = position;
        instance.transform.localScale = Vector3.one * scale;
        instance.name = robotId;
        
        // Register undo
        Undo.RegisterCreatedObjectUndo(instance, $"Spawn {robotId}");
        
        // Select the new object
        Selection.activeGameObject = instance;
        
        Debug.Log($"Spawned vbot: {robotId} at {position}");
    }
}
```

### Step 4: Using robotics-mcp Tools

**Option A: Via vbot_crud (Recommended)**

```python
# Create a Scout vbot
vbot_crud(
    operation="create",
    robot_type="scout",
    platform="unity",
    position={"x": 0.0, "y": 0.0, "z": 0.0},
    scale=1.0
)

# Create Robbie from Forbidden Planet
vbot_crud(
    operation="create",
    robot_type="robbie",
    platform="unity",
    position={"x": 1.0, "y": 0.0, "z": 1.0},
    scale=1.0
)
```

**Option B: Via virtual_robotics**

```python
virtual_robotics(
    robot_type="scout",
    action="spawn_robot",
    platform="unity",
    position={"x": 0.0, "y": 0.0, "z": 0.0},
    scale=1.0
)
```

**Option C: Direct Unity Method Execution**

If you have a custom spawner script, use `execute_unity_method`:

```python
# Via unity3d-mcp (if mounted)
execute_unity_method(
    class_name="VbotEditorSpawner",
    method_name="SpawnVbot",
    parameters={
        "robotId": "vbot_scout_01",
        "prefabPath": "Assets/Prefabs/Scout.prefab",
        "position": {"x": 0.0, "y": 0.0, "z": 0.0},
        "scale": 1.0
    },
    project_path="D:/Projects/MyUnityProject"
)
```

## Complete Example: Setting Up a Vbot Spawner

### 1. Create the Spawner Script

**File:** `Assets/Scripts/VbotSpawner.cs`

```csharp
using UnityEngine;
using System.Collections.Generic;

public class VbotSpawner : MonoBehaviour
{
    [System.Serializable]
    public class RobotPrefabData
    {
        public string robotType;
        public GameObject prefab;
    }
    
    [SerializeField] private List<RobotPrefabData> robotPrefabs = new List<RobotPrefabData>();
    private Dictionary<string, GameObject> prefabLookup = new Dictionary<string, GameObject>();
    private Dictionary<string, GameObject> spawnedRobots = new Dictionary<string, GameObject>();
    
    void Awake()
    {
        // Build lookup dictionary
        foreach (var data in robotPrefabs)
        {
            if (data.prefab != null)
            {
                prefabLookup[data.robotType.ToLower()] = data.prefab;
            }
        }
    }
    
    public GameObject SpawnRobot(string robotId, string robotType, Vector3 position, float scale = 1.0f)
    {
        string typeKey = robotType.ToLower();
        
        if (!prefabLookup.ContainsKey(typeKey))
        {
            Debug.LogError($"Robot type '{robotType}' not found in prefab lookup!");
            return null;
        }
        
        GameObject prefab = prefabLookup[typeKey];
        GameObject instance = Instantiate(prefab, position, Quaternion.identity);
        
        instance.transform.localScale = Vector3.one * scale;
        instance.name = robotId;
        
        spawnedRobots[robotId] = instance;
        
        Debug.Log($"Spawned {robotId} ({robotType}) at {position} with scale {scale}");
        
        return instance;
    }
    
    public bool UpdateRobot(string robotId, Vector3? position = null, float? scale = null)
    {
        if (!spawnedRobots.ContainsKey(robotId))
        {
            Debug.LogError($"Robot {robotId} not found!");
            return false;
        }
        
        GameObject robot = spawnedRobots[robotId];
        
        if (position.HasValue)
        {
            robot.transform.position = position.Value;
        }
        
        if (scale.HasValue)
        {
            robot.transform.localScale = Vector3.one * scale.Value;
        }
        
        return true;
    }
    
    public bool DeleteRobot(string robotId)
    {
        if (!spawnedRobots.ContainsKey(robotId))
        {
            Debug.LogError($"Robot {robotId} not found!");
            return false;
        }
        
        GameObject robot = spawnedRobots[robotId];
        spawnedRobots.Remove(robotId);
        Destroy(robot);
        
        Debug.Log($"Deleted robot: {robotId}");
        
        return true;
    }
    
    public GameObject GetRobot(string robotId)
    {
        return spawnedRobots.ContainsKey(robotId) ? spawnedRobots[robotId] : null;
    }
}
```

### 2. Set Up in Unity Editor

1. **Create empty GameObject:**
   - Right-click in Hierarchy → Create Empty
   - Name it "VbotSpawner"

2. **Add VbotSpawner component:**
   - Select "VbotSpawner" GameObject
   - Add Component → Scripts → VbotSpawner

3. **Assign prefabs:**
   - In Inspector, expand "Robot Prefabs" list
   - Add entries:
     - Element 0: robotType = "scout", prefab = (drag Scout.prefab)
     - Element 1: robotType = "robbie", prefab = (drag Robbie.prefab)
     - Element 2: robotType = "go2", prefab = (drag Go2.prefab)

### 3. Use via robotics-mcp

The `vbot_crud` tool will call this spawner via `execute_unity_method`:

```python
# This will be called internally by vbot_crud
execute_unity_method(
    class_name="VbotSpawner",
    method_name="SpawnRobot",
    parameters={
        "robotId": "vbot_scout_01",
        "robotType": "scout",
        "position": {"x": 0.0, "y": 0.0, "z": 0.0},
        "scale": 1.0
    }
)
```

## Unity Coordinate System

**Important:** Unity uses a **left-handed coordinate system**:
- **X**: Right (positive) / Left (negative)
- **Y**: Up (positive) / Down (negative)
- **Z**: Forward (positive) / Backward (negative)

**Transform values:**
- **Position**: World space coordinates in meters (typically)
- **Rotation**: Euler angles in degrees (0-360) or Quaternion
- **Scale**: Multiplier (1.0 = original size, 2.0 = double size)

## Common Components for Robots

When setting up a robot prefab, consider adding:

1. **Rigidbody**: For physics simulation (gravity, collisions)
2. **Collider**: For collision detection (BoxCollider, CapsuleCollider, MeshCollider)
3. **MeshRenderer**: Renders the 3D model
4. **Animator**: For animations (if model has animations)
5. **Custom Scripts**: RobotController, MovementController, etc.

## Next Steps

1. **Import robot models** into `Assets/Models/`
2. **Create prefabs** in `Assets/Prefabs/`
3. **Set up VbotSpawner** script and GameObject
4. **Use vbot_crud** tool to spawn robots programmatically
5. **Test instantiation** in Play mode or Editor mode

## Troubleshooting

**Prefab not found:**
- Check path is relative to `Assets/` folder
- Ensure prefab exists in Project window
- Verify prefab is not a model file (must be a `.prefab`)

**Robot spawns at wrong position:**
- Check Transform component
- Verify position is in world space, not local space
- Check if parent GameObject has offset

**Scale issues:**
- Unity units are typically meters
- Check model import scale settings
- Verify `localScale` vs `lossyScale`

**No robot appears:**
- Check if camera can see the spawn position
- Verify GameObject is active (checkbox in Inspector)
- Check if prefab has MeshRenderer component

---

**Austrian Precision**: Proper Unity terminology ensures clear communication and correct implementation! 🇦🇹🤖

