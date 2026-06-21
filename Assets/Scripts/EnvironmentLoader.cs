using UnityEngine;
using UnityEditor;

/// <summary>
/// Loads the living room environment into the scene.
/// Can be called statically via execute_unity_method.
/// </summary>
public class EnvironmentLoader : MonoBehaviour
{
    /// <summary>
    /// Load the Modern Tropical Luxury Residence environment into the scene.
    /// </summary>
    public static string LoadLivingRoom()
    {
        try
        {
            // Find the collider GLB asset
            string colliderPath = "Assets/WorldLabs/Modern Tropical Luxury Residence_collider/Modern Tropical Luxury Residence_collider.glb";
            GameObject colliderPrefab = AssetDatabase.LoadAssetAtPath<GameObject>(colliderPath);
            
            if (colliderPrefab == null)
            {
                return $"ERROR: Collider GLB not found at {colliderPath}";
            }
            
            // Check if already in scene
            GameObject existing = GameObject.Find("Modern Tropical Luxury Residence_collider");
            if (existing != null)
            {
                return $"SUCCESS: Environment already in scene at position {existing.transform.position}";
            }
            
            // Instantiate in scene
            GameObject instance = PrefabUtility.InstantiatePrefab(colliderPrefab) as GameObject;
            if (instance == null)
            {
                instance = Object.Instantiate(colliderPrefab);
            }
            
            instance.name = "Modern Tropical Luxury Residence_collider";
            instance.transform.position = Vector3.zero;
            instance.transform.rotation = Quaternion.identity;
            
            // Select it in hierarchy
            Selection.activeGameObject = instance;
            SceneView.FrameLastActiveSceneView();
            
            return $"SUCCESS: Loaded living room environment at position {instance.transform.position}. " +
                   $"GameObject: {instance.name}, Children: {instance.transform.childCount}";
        }
        catch (System.Exception e)
        {
            return $"ERROR: Failed to load environment: {e.Message}";
        }
    }
    
    /// <summary>
    /// Get status of environment in scene.
    /// </summary>
    public static string GetEnvironmentStatus()
    {
        try
        {
            GameObject env = GameObject.Find("Modern Tropical Luxury Residence_collider");
            if (env == null)
            {
                return "Environment not in scene. Call LoadLivingRoom() to add it.";
            }
            
            return $"Environment in scene: {env.name}\n" +
                   $"Position: {env.transform.position}\n" +
                   $"Children: {env.transform.childCount}\n" +
                   $"Active: {env.activeSelf}";
        }
        catch (System.Exception e)
        {
            return $"ERROR: {e.Message}";
        }
    }
}
