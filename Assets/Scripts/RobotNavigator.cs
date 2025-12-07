using UnityEngine;
using UnityEngine.AI;
using System.Collections.Generic;
using System.Linq;

/// <summary>
/// Navigation controller for virtual robots in Unity.
/// Handles path planning, waypoints, obstacle avoidance, and NavMesh navigation.
/// Used by robotics-mcp robot_navigation tool via execute_unity_method.
/// </summary>
public class RobotNavigator : MonoBehaviour
{
    private static Dictionary<string, RobotNavigator> instances = new Dictionary<string, RobotNavigator>();
    private static Dictionary<string, PathData> paths = new Dictionary<string, PathData>();
    
    [Header("Navigation Settings")]
    public float speed = 2f;
    public float stoppingDistance = 0.5f;
    public float avoidanceRadius = 1f;
    
    private NavMeshAgent agent;
    private List<Vector3> waypoints = new List<Vector3>();
    private string currentPathId = "";
    private bool isFollowingPath = false;
    
    [System.Serializable]
    public class PathData
    {
        public string pathId;
        public List<Vector3> waypoints;
        public string status; // "planned", "following", "completed", "failed"
        public float progress; // 0.0 to 1.0
    }
    
    void Awake()
    {
        agent = GetComponent<NavMeshAgent>();
        if (agent == null)
        {
            agent = gameObject.AddComponent<NavMeshAgent>();
        }
        
        agent.speed = speed;
        agent.stoppingDistance = stoppingDistance;
        agent.radius = avoidanceRadius;
    }
    
    void Update()
    {
        if (isFollowingPath && waypoints.Count > 0)
        {
            if (!agent.pathPending && agent.remainingDistance < stoppingDistance)
            {
                waypoints.RemoveAt(0);
                if (waypoints.Count > 0)
                {
                    agent.SetDestination(waypoints[0]);
                }
                else
                {
                    isFollowingPath = false;
                    if (paths.ContainsKey(currentPathId))
                    {
                        paths[currentPathId].status = "completed";
                        paths[currentPathId].progress = 1.0f;
                    }
                }
            }
            
            if (paths.ContainsKey(currentPathId))
            {
                float totalDistance = 0f;
                float remainingDistance = agent.remainingDistance;
                for (int i = 0; i < waypoints.Count - 1; i++)
                {
                    totalDistance += Vector3.Distance(waypoints[i], waypoints[i + 1]);
                }
                paths[currentPathId].progress = totalDistance > 0 ? 1.0f - (remainingDistance / totalDistance) : 1.0f;
            }
        }
    }
    
    /// <summary>
    /// Plan path from start to goal using NavMesh.
    /// </summary>
    public static Dictionary<string, object> PlanPath(string robotId, Dictionary<string, float> startPosition, Dictionary<string, float> goalPosition)
    {
        RobotNavigator instance = GetInstance(robotId);
        if (instance == null) return null;
        
        return instance.PlanPathInstance(startPosition, goalPosition);
    }
    
    private Dictionary<string, object> PlanPathInstance(Dictionary<string, float> startPos, Dictionary<string, float> goalPos)
    {
        Vector3 start = new Vector3(startPos["x"], startPos["y"], startPos["z"]);
        Vector3 goal = new Vector3(goalPos["x"], goalPos["y"], goalPos["z"]);
        
        NavMeshPath path = new NavMeshPath();
        bool pathFound = NavMesh.CalculatePath(start, goal, NavMesh.AllAreas, path);
        
        if (!pathFound)
        {
            return new Dictionary<string, object>
            {
                {"path_id", ""},
                {"status", "failed"},
                {"message", "Path not found - goal may be unreachable"}
            };
        }
        
        string pathId = $"path_{System.Guid.NewGuid().ToString().Substring(0, 8)}";
        List<Vector3> waypointList = path.corners.ToList();
        
        PathData pathData = new PathData
        {
            pathId = pathId,
            waypoints = waypointList,
            status = "planned",
            progress = 0.0f
        };
        
        paths[pathId] = pathData;
        
        return new Dictionary<string, object>
        {
            {"path_id", pathId},
            {"status", "planned"},
            {"waypoints", waypointList.Select(w => new Dictionary<string, float> { {"x", w.x}, {"y", w.y}, {"z", w.z} }).ToList()},
            {"distance", CalculatePathDistance(waypointList)}
        };
    }
    
    /// <summary>
    /// Follow a planned path.
    /// </summary>
    public static bool FollowPath(string robotId, string pathId)
    {
        RobotNavigator instance = GetInstance(robotId);
        if (instance == null) return false;
        
        return instance.FollowPathInstance(pathId);
    }
    
    private bool FollowPathInstance(string pathId)
    {
        if (!paths.ContainsKey(pathId))
        {
            Debug.LogError($"Path '{pathId}' not found.");
            return false;
        }
        
        PathData pathData = paths[pathId];
        waypoints = new List<Vector3>(pathData.waypoints);
        currentPathId = pathId;
        pathData.status = "following";
        isFollowingPath = true;
        
        if (waypoints.Count > 0)
        {
            agent.SetDestination(waypoints[0]);
        }
        
        return true;
    }
    
    /// <summary>
    /// Set a waypoint.
    /// </summary>
    public static bool SetWaypoint(string robotId, Dictionary<string, float> waypoint)
    {
        RobotNavigator instance = GetInstance(robotId);
        if (instance == null) return false;
        
        instance.SetWaypointInstance(waypoint);
        return true;
    }
    
    private void SetWaypointInstance(Dictionary<string, float> waypoint)
    {
        Vector3 wp = new Vector3(waypoint["x"], waypoint["y"], waypoint["z"]);
        waypoints.Add(wp);
        Debug.Log($"{gameObject.name} waypoint added: {wp}");
    }
    
    /// <summary>
    /// Clear all waypoints.
    /// </summary>
    public static bool ClearWaypoints(string robotId)
    {
        RobotNavigator instance = GetInstance(robotId);
        if (instance == null) return false;
        
        instance.ClearWaypointsInstance();
        return true;
    }
    
    private void ClearWaypointsInstance()
    {
        waypoints.Clear();
        isFollowingPath = false;
        agent.ResetPath();
        Debug.Log($"{gameObject.name} waypoints cleared");
    }
    
    /// <summary>
    /// Get path execution status.
    /// </summary>
    public static Dictionary<string, object> GetPathStatus(string robotId, string pathId)
    {
        if (!paths.ContainsKey(pathId))
        {
            return new Dictionary<string, object>
            {
                {"path_id", pathId},
                {"status", "not_found"}
            };
        }
        
        PathData pathData = paths[pathId];
        return new Dictionary<string, object>
        {
            {"path_id", pathId},
            {"status", pathData.status},
            {"progress", pathData.progress},
            {"waypoints_remaining", pathData.waypoints.Count}
        };
    }
    
    /// <summary>
    /// Trigger obstacle avoidance.
    /// </summary>
    public static bool AvoidObstacle(string robotId, Dictionary<string, float> obstaclePosition)
    {
        RobotNavigator instance = GetInstance(robotId);
        if (instance == null) return false;
        
        instance.AvoidObstacleInstance(obstaclePosition);
        return true;
    }
    
    private void AvoidObstacleInstance(Dictionary<string, float> obstaclePos)
    {
        Vector3 obstacle = new Vector3(obstaclePos["x"], obstaclePos["y"], obstaclePos["z"]);
        Vector3 direction = (transform.position - obstacle).normalized;
        Vector3 avoidanceTarget = transform.position + direction * avoidanceRadius * 2f;
        
        NavMeshHit hit;
        if (NavMesh.SamplePosition(avoidanceTarget, out hit, avoidanceRadius * 2f, NavMesh.AllAreas))
        {
            agent.SetDestination(hit.position);
            Debug.Log($"{gameObject.name} avoiding obstacle at {obstacle}");
        }
    }
    
    /// <summary>
    /// Get current path being followed.
    /// </summary>
    public static Dictionary<string, object> GetCurrentPath(string robotId)
    {
        RobotNavigator instance = GetInstance(robotId);
        if (instance == null) return null;
        
        return instance.GetCurrentPathInstance();
    }
    
    private Dictionary<string, object> GetCurrentPathInstance()
    {
        if (string.IsNullOrEmpty(currentPathId) || !paths.ContainsKey(currentPathId))
        {
            return new Dictionary<string, object>
            {
                {"path_id", ""},
                {"status", "no_path"}
            };
        }
        
        PathData pathData = paths[currentPathId];
        return new Dictionary<string, object>
        {
            {"path_id", currentPathId},
            {"status", pathData.status},
            {"progress", pathData.progress},
            {"waypoints", pathData.waypoints.Select(w => new Dictionary<string, float> { {"x", w.x}, {"y", w.y}, {"z", w.z} }).ToList()}
        };
    }
    
    private float CalculatePathDistance(List<Vector3> waypoints)
    {
        float distance = 0f;
        for (int i = 0; i < waypoints.Count - 1; i++)
        {
            distance += Vector3.Distance(waypoints[i], waypoints[i + 1]);
        }
        return distance;
    }
    
    private static RobotNavigator GetInstance(string robotId)
    {
        if (instances.ContainsKey(robotId))
        {
            return instances[robotId];
        }
        
        GameObject robot = GameObject.Find(robotId);
        if (robot == null)
        {
            Debug.LogError($"Robot with ID '{robotId}' not found in scene.");
            return null;
        }
        
        RobotNavigator navigator = robot.GetComponent<RobotNavigator>();
        if (navigator == null)
        {
            navigator = robot.AddComponent<RobotNavigator>();
        }
        
        instances[robotId] = navigator;
        return navigator;
    }
    
    void OnDestroy()
    {
        instances.Remove(gameObject.name);
    }
}

