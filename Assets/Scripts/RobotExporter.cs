using UnityEngine;
using System.IO;
using System.Text;
using System.Collections.Generic;

/// <summary>
/// Exporter for robot GameObjects to various 3D formats.
/// Used by robotics-mcp robot_model export operation via execute_unity_method.
/// 
/// Supports:
/// - OBJ: Native Unity export (no packages required)
/// - FBX: Requires Unity FBX Exporter package (com.unity.formats.fbx)
/// - GLB: Requires glTF exporter package or conversion via Blender
/// </summary>
public class RobotExporter : MonoBehaviour
{
    private static RobotExporter instance;
    
    void Awake()
    {
        if (instance == null)
        {
            instance = this;
            DontDestroyOnLoad(gameObject);
        }
        else
        {
            Destroy(gameObject);
        }
    }
    
    /// <summary>
    /// Export a robot GameObject to a 3D file format.
    /// Can be called statically via execute_unity_method.
    /// </summary>
    /// <param name="robotId">Robot identifier (used to find GameObject)</param>
    /// <param name="outputPath">Full path where to save the exported file</param>
    /// <param name="format">Export format: "obj", "fbx", or "glb"</param>
    /// <param name="includeAnimations">Whether to include animations (FBX/GLB only)</param>
    /// <returns>Export result message</returns>
    public static string ExportRobot(string robotId, string outputPath, string format = "obj", bool includeAnimations = true)
    {
        try
        {
            // Find the robot GameObject
            GameObject robotObj = GameObject.Find(robotId);
            if (robotObj == null)
            {
                // Try to find via VbotSpawner
                robotObj = VbotSpawner.GetRobot(robotId);
            }
            
            if (robotObj == null)
            {
                return $"ERROR: Robot '{robotId}' not found in scene";
            }
            
            format = format.ToLower();
            
            switch (format)
            {
                case "obj":
                    return ExportToOBJ(robotObj, outputPath);
                case "fbx":
                    return ExportToFBX(robotObj, outputPath, includeAnimations);
                case "glb":
                case "gltf":
                    return ExportToGLB(robotObj, outputPath, includeAnimations);
                default:
                    return $"ERROR: Unsupported format '{format}'. Supported: obj, fbx, glb";
            }
        }
        catch (System.Exception e)
        {
            return $"ERROR: Export failed: {e.Message}";
        }
    }
    
    /// <summary>
    /// Export GameObject to OBJ format (native Unity, no packages required).
    /// </summary>
    private static string ExportToOBJ(GameObject obj, string outputPath)
    {
        try
        {
            StringBuilder sb = new StringBuilder();
            int vertexOffset = 0;
            
            // Write header
            sb.AppendLine("# OBJ file exported from Unity");
            sb.AppendLine($"# Robot: {obj.name}");
            sb.AppendLine();
            
            // Collect all mesh filters in hierarchy
            MeshFilter[] meshFilters = obj.GetComponentsInChildren<MeshFilter>();
            
            foreach (MeshFilter mf in meshFilters)
            {
                if (mf.sharedMesh == null) continue;
                
                Mesh mesh = mf.sharedMesh;
                Transform transform = mf.transform;
                
                // Write object name
                sb.AppendLine($"o {mf.gameObject.name}");
                
                // Write vertices (apply transform)
                foreach (Vector3 vertex in mesh.vertices)
                {
                    Vector3 worldVertex = transform.TransformPoint(vertex);
                    sb.AppendLine($"v {worldVertex.x} {worldVertex.y} {worldVertex.z}");
                }
                
                // Write normals
                foreach (Vector3 normal in mesh.normals)
                {
                    Vector3 worldNormal = transform.TransformDirection(normal).normalized;
                    sb.AppendLine($"vn {worldNormal.x} {worldNormal.y} {worldNormal.z}");
                }
                
                // Write UVs
                foreach (Vector2 uv in mesh.uv)
                {
                    sb.AppendLine($"vt {uv.x} {uv.y}");
                }
                
                // Write faces (with vertex offset)
                int[] triangles = mesh.triangles;
                for (int i = 0; i < triangles.Length; i += 3)
                {
                    int v1 = triangles[i] + 1 + vertexOffset;
                    int v2 = triangles[i + 1] + 1 + vertexOffset;
                    int v3 = triangles[i + 2] + 1 + vertexOffset;
                    
                    if (mesh.uv.Length > 0 && mesh.normals.Length > 0)
                    {
                        sb.AppendLine($"f {v1}/{v1}/{v1} {v2}/{v2}/{v2} {v3}/{v3}/{v3}");
                    }
                    else if (mesh.normals.Length > 0)
                    {
                        sb.AppendLine($"f {v1}//{v1} {v2}//{v2} {v3}//{v3}");
                    }
                    else
                    {
                        sb.AppendLine($"f {v1} {v2} {v3}");
                    }
                }
                
                vertexOffset += mesh.vertexCount;
                sb.AppendLine();
            }
            
            // Write to file
            string directory = Path.GetDirectoryName(outputPath);
            if (!string.IsNullOrEmpty(directory) && !Directory.Exists(directory))
            {
                Directory.CreateDirectory(directory);
            }
            
            File.WriteAllText(outputPath, sb.ToString());
            
            return $"SUCCESS: Exported {obj.name} to OBJ: {outputPath}";
        }
        catch (System.Exception e)
        {
            return $"ERROR: OBJ export failed: {e.Message}";
        }
    }
    
    /// <summary>
    /// Export GameObject to FBX format (requires Unity FBX Exporter package).
    /// </summary>
    private static string ExportToFBX(GameObject obj, string outputPath, bool includeAnimations)
    {
        try
        {
            // Check if FBX Exporter package is available
            #if UNITY_EDITOR && UNITY_FORMATS_FBX
            // Use Unity FBX Exporter package
            UnityEditor.Formats.Fbx.Exporter.ModelExporter.ExportObject(outputPath, obj);
            return $"SUCCESS: Exported {obj.name} to FBX: {outputPath}";
            #else
            // FBX Exporter package not available - suggest OBJ or Blender conversion
            return $"ERROR: FBX export requires Unity FBX Exporter package (com.unity.formats.fbx). " +
                   $"Please install it via Package Manager, or export as OBJ and convert via Blender.";
            #endif
        }
        catch (System.Exception e)
        {
            return $"ERROR: FBX export failed: {e.Message}";
        }
    }
    
    /// <summary>
    /// Export GameObject to GLB/glTF format (requires glTF exporter or conversion via Blender).
    /// </summary>
    private static string ExportToGLB(GameObject obj, string outputPath, bool includeAnimations)
    {
        try
        {
            // GLB export requires a package or conversion
            // For now, export as OBJ and suggest Blender conversion
            string objPath = outputPath.Replace(".glb", ".obj").Replace(".gltf", ".obj");
            string result = ExportToOBJ(obj, objPath);
            
            if (result.StartsWith("SUCCESS"))
            {
                return $"SUCCESS: Exported {obj.name} to OBJ (GLB not directly supported): {objPath}. " +
                       $"Use Blender to convert OBJ to GLB, or install a glTF exporter package.";
            }
            else
            {
                return result;
            }
        }
        catch (System.Exception e)
        {
            return $"ERROR: GLB export failed: {e.Message}";
        }
    }
}
