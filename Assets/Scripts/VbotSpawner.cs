using UnityEngine;
using System.Collections.Generic;

/// <summary>
/// Spawner for virtual robots (vbots) in Unity scenes.
/// This script handles instantiation, updating, and deletion of robot GameObjects.
/// Used by robotics-mcp vbot_crud tool via execute_unity_method.
/// </summary>
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
    
    private static VbotSpawner instance;
    
    void Awake()
    {
        // Singleton pattern for static method access
        if (instance == null)
        {
            instance = this;
            DontDestroyOnLoad(gameObject);
        }
        else
        {
            Destroy(gameObject);
            return;
        }
        
        // Build lookup dictionary
        foreach (var data in robotPrefabs)
        {
            if (data.prefab != null && !string.IsNullOrEmpty(data.robotType))
            {
                prefabLookup[data.robotType.ToLower()] = data.prefab;
            }
        }
    }
    
    /// <summary>
    /// Spawn a robot in the scene. Can be called statically via execute_unity_method.
    /// </summary>
    /// <param name="robotId">Unique identifier for the robot</param>
    /// <param name="robotType">Type of robot (scout, robbie, go2, etc.)</param>
    /// <param name="position">World position (x, y, z)</param>
    /// <param name="scale">Scale multiplier (1.0 = original size)</param>
    /// <returns>True if spawn succeeded, false otherwise</returns>
    public static bool SpawnRobot(string robotId, string robotType, Vector3 position, float scale = 1.0f)
    {
        if (instance == null)
        {
            Debug.LogError("VbotSpawner instance not found! Create a GameObject with VbotSpawner component in the scene.");
            return false;
        }
        
        return instance.SpawnRobotInstance(robotId, robotType, position, scale);
    }
    
    private bool SpawnRobotInstance(string robotId, string robotType, Vector3 position, float scale)
    {
        string typeKey = robotType.ToLower();
        
        if (!prefabLookup.ContainsKey(typeKey))
        {
            Debug.LogError($"Robot type '{robotType}' not found in prefab lookup! Available types: {string.Join(", ", prefabLookup.Keys)}");
            return false;
        }
        
        if (spawnedRobots.ContainsKey(robotId))
        {
            Debug.LogWarning($"Robot {robotId} already exists! Deleting old instance.");
            DeleteRobotInstance(robotId);
        }
        
        GameObject prefab = prefabLookup[typeKey];
        GameObject instance = Instantiate(prefab, position, Quaternion.identity);
        
        instance.transform.localScale = Vector3.one * scale;
        instance.name = robotId;
        
        spawnedRobots[robotId] = instance;
        
        Debug.Log($"Spawned {robotId} ({robotType}) at {position} with scale {scale}");
        
        return true;
    }
    
    /// <summary>
    /// Update robot position and/or scale. Can be called statically.
    /// </summary>
    public static bool UpdateRobot(string robotId, Vector3? position = null, float? scale = null)
    {
        if (instance == null)
        {
            Debug.LogError("VbotSpawner instance not found!");
            return false;
        }
        
        return instance.UpdateRobotInstance(robotId, position, scale);
    }
    
    private bool UpdateRobotInstance(string robotId, Vector3? position, float? scale)
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
        
        Debug.Log($"Updated {robotId}: position={position}, scale={scale}");
        
        return true;
    }
    
    /// <summary>
    /// Delete a robot from the scene. Can be called statically.
    /// </summary>
    public static bool DeleteRobot(string robotId)
    {
        if (instance == null)
        {
            Debug.LogError("VbotSpawner instance not found!");
            return false;
        }
        
        return instance.DeleteRobotInstance(robotId);
    }
    
    private bool DeleteRobotInstance(string robotId)
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
    
    /// <summary>
    /// Get a spawned robot GameObject. Can be called statically.
    /// </summary>
    public static GameObject GetRobot(string robotId)
    {
        if (instance == null)
        {
            return null;
        }
        
        return instance.spawnedRobots.ContainsKey(robotId) ? instance.spawnedRobots[robotId] : null;
    }
    
    /// <summary>
    /// List all spawned robot IDs. Can be called statically.
    /// </summary>
    public static List<string> ListRobots()
    {
        if (instance == null)
        {
            return new List<string>();
        }
        
        return new List<string>(instance.spawnedRobots.Keys);
    }
}

