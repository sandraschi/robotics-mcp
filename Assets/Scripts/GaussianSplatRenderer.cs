using UnityEngine;
using System.IO;

/// <summary>
/// Helper script to render Gaussian Splat PLY files in Unity.
/// Requires: com.aras-p.gaussian-splatting package
/// 
/// Usage:
/// 1. Attach this script to an empty GameObject
/// 2. Assign PLY file path in Inspector
/// 3. Click "Load Splat" button in Inspector
/// </summary>
public class GaussianSplatRenderer : MonoBehaviour
{
    [Header("PLY File Settings")]
    [Tooltip("Path to PLY file (relative to Assets/ or absolute path)")]
    public string plyFilePath = "Assets/WorldLabs/Modern Tropical Luxury Residence/Splats/cafcb2c0-c073-435e-adc5-406e6588e0db_ceramic_500k.ply";
    
    [Header("Transform Settings")]
    public Vector3 position = Vector3.zero;
    public Vector3 rotation = Vector3.zero;
    public Vector3 scale = Vector3.one;

    private GameObject splatObject;

    [ContextMenu("Load Splat")]
    public void LoadSplat()
    {
        if (string.IsNullOrEmpty(plyFilePath))
        {
            Debug.LogError("PLY file path is empty!");
            return;
        }

        // Try to find the PLY file
        string fullPath = plyFilePath;
        if (!Path.IsPathRooted(plyFilePath))
        {
            // Relative path - try Assets/ first
            string assetsPath = Path.Combine(Application.dataPath, plyFilePath.Replace("Assets/", ""));
            if (File.Exists(assetsPath))
            {
                fullPath = assetsPath;
            }
            else
            {
                Debug.LogError($"PLY file not found at: {plyFilePath}");
                Debug.LogError($"Tried: {assetsPath}");
                return;
            }
        }

        if (!File.Exists(fullPath))
        {
            Debug.LogError($"PLY file not found: {fullPath}");
            return;
        }

        Debug.Log($"Loading Gaussian Splat from: {fullPath}");

        // Check if Gaussian Splatting package is available
        // The aras-p/UnityGaussianSplatting package should provide a component
        // We'll try to use reflection to access it, or provide manual instructions

        // For now, create a placeholder GameObject and provide instructions
        if (splatObject != null)
        {
            DestroyImmediate(splatObject);
        }

        splatObject = new GameObject("GaussianSplat_" + Path.GetFileNameWithoutExtension(plyFilePath));
        splatObject.transform.SetParent(transform);
        splatObject.transform.localPosition = position;
        splatObject.transform.localRotation = Quaternion.Euler(rotation);
        splatObject.transform.localScale = scale;

        // Try to add Gaussian Splat component via reflection
        // The package typically provides a "GaussianSplatRenderer" or similar component
        System.Type splatType = System.Type.GetType("GaussianSplatRenderer, Assembly-CSharp");
        if (splatType == null)
        {
            // Try to find it in the package
            splatType = System.Type.GetType("GaussianSplatRenderer");
        }

        if (splatType != null)
        {
            Component splatComponent = splatObject.AddComponent(splatType);
            // Try to set the PLY file path via reflection
            var pathProperty = splatType.GetProperty("SplatPath") ?? splatType.GetProperty("PlyPath") ?? splatType.GetProperty("Path");
            if (pathProperty != null)
            {
                pathProperty.SetValue(splatComponent, fullPath);
                Debug.Log($"Loaded Gaussian Splat component and set path: {fullPath}");
            }
            else
            {
                Debug.LogWarning("Gaussian Splat component found but couldn't set path. Check package documentation.");
            }
        }
        else
        {
            Debug.LogWarning("Gaussian Splatting component not found. The package may not be fully loaded yet.");
            Debug.LogWarning("Please ensure:");
            Debug.LogWarning("1. Unity Package Manager has finished downloading the package");
            Debug.LogWarning("2. The package is: com.aras-p.gaussian-splatting");
            Debug.LogWarning("3. Check Package Manager > My Assets > Gaussian Splatting");
            
            // Add a simple visual indicator
            var renderer = splatObject.AddComponent<MeshRenderer>();
            var filter = splatObject.AddComponent<MeshFilter>();
            // Create a simple cube as placeholder
            filter.mesh = CreatePlaceholderMesh();
            var material = new Material(Shader.Find("Standard"));
            material.color = Color.cyan;
            renderer.material = material;
        }

        Debug.Log($"Gaussian Splat GameObject created: {splatObject.name}");
    }

    private Mesh CreatePlaceholderMesh()
    {
        Mesh mesh = new Mesh();
        mesh.vertices = new Vector3[]
        {
            new Vector3(-1, 0, -1),
            new Vector3(1, 0, -1),
            new Vector3(1, 0, 1),
            new Vector3(-1, 0, 1)
        };
        mesh.triangles = new int[] { 0, 1, 2, 0, 2, 3 };
        mesh.RecalculateNormals();
        return mesh;
    }

    void Start()
    {
        // Auto-load on start if path is set
        if (!string.IsNullOrEmpty(plyFilePath) && Application.isPlaying)
        {
            LoadSplat();
        }
    }
}

