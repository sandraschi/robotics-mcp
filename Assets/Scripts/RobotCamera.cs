using UnityEngine;
using System.Collections.Generic;
using System.IO;

/// <summary>
/// Camera controller for virtual robots in Unity.
/// Handles camera feeds, angle control, image capture, and streaming.
/// Used by robotics-mcp robot_camera tool via execute_unity_method.
/// </summary>
public class RobotCamera : MonoBehaviour
{
    private static Dictionary<string, RobotCamera> instances = new Dictionary<string, RobotCamera>();
    
    [Header("Camera Settings")]
    public Camera robotCamera;
    public int captureWidth = 1920;
    public int captureHeight = 1080;
    public float fieldOfView = 60f;
    
    private bool isStreaming = false;
    private string streamUrl = "";
    
    void Awake()
    {
        if (robotCamera == null)
        {
            robotCamera = GetComponentInChildren<Camera>();
            if (robotCamera == null)
            {
                // Create camera if none exists
                GameObject cameraObj = new GameObject("RobotCamera");
                cameraObj.transform.SetParent(transform);
                cameraObj.transform.localPosition = new Vector3(0, 0.1f, 0.1f); // Slightly forward and up
                robotCamera = cameraObj.AddComponent<Camera>();
                robotCamera.fieldOfView = fieldOfView;
            }
        }
    }
    
    /// <summary>
    /// Get camera feed (returns base64 encoded image).
    /// </summary>
    public static Dictionary<string, object> GetCameraFeed(string robotId)
    {
        RobotCamera instance = GetInstance(robotId);
        if (instance == null) return null;
        
        return instance.GetCameraFeedInstance();
    }
    
    private Dictionary<string, object> GetCameraFeedInstance()
    {
        if (robotCamera == null) return null;
        
        RenderTexture rt = new RenderTexture(captureWidth, captureHeight, 24);
        robotCamera.targetTexture = rt;
        Texture2D screenshot = new Texture2D(captureWidth, captureHeight, TextureFormat.RGB24, false);
        robotCamera.Render();
        RenderTexture.active = rt;
        screenshot.ReadPixels(new Rect(0, 0, captureWidth, captureHeight), 0, 0);
        robotCamera.targetTexture = null;
        RenderTexture.active = null;
        Destroy(rt);
        
        byte[] imageBytes = screenshot.EncodeToPNG();
        string base64Image = System.Convert.ToBase64String(imageBytes);
        Destroy(screenshot);
        
        return new Dictionary<string, object>
        {
            {"image_base64", base64Image},
            {"width", captureWidth},
            {"height", captureHeight},
            {"format", "PNG"}
        };
    }
    
    /// <summary>
    /// Set camera angle (pitch and yaw).
    /// </summary>
    public static bool SetCameraAngle(string robotId, float angleX, float angleY)
    {
        RobotCamera instance = GetInstance(robotId);
        if (instance == null) return false;
        
        instance.SetCameraAngleInstance(angleX, angleY);
        return true;
    }
    
    private void SetCameraAngleInstance(float angleX, float angleY)
    {
        if (robotCamera != null)
        {
            Transform camTransform = robotCamera.transform;
            camTransform.localRotation = Quaternion.Euler(angleX, angleY, 0);
        }
    }
    
    /// <summary>
    /// Capture image and save to file.
    /// </summary>
    public static Dictionary<string, object> CaptureImage(string robotId, string outputPath)
    {
        RobotCamera instance = GetInstance(robotId);
        if (instance == null) return null;
        
        return instance.CaptureImageInstance(outputPath);
    }
    
    private Dictionary<string, object> CaptureImageInstance(string outputPath)
    {
        if (robotCamera == null) return null;
        
        RenderTexture rt = new RenderTexture(captureWidth, captureHeight, 24);
        robotCamera.targetTexture = rt;
        Texture2D screenshot = new Texture2D(captureWidth, captureHeight, TextureFormat.RGB24, false);
        robotCamera.Render();
        RenderTexture.active = rt;
        screenshot.ReadPixels(new Rect(0, 0, captureWidth, captureHeight), 0, 0);
        robotCamera.targetTexture = null;
        RenderTexture.active = null;
        Destroy(rt);
        
        byte[] imageBytes = screenshot.EncodeToPNG();
        
        if (!string.IsNullOrEmpty(outputPath))
        {
            string directory = Path.GetDirectoryName(outputPath);
            if (!string.IsNullOrEmpty(directory) && !Directory.Exists(directory))
            {
                Directory.CreateDirectory(directory);
            }
            File.WriteAllBytes(outputPath, imageBytes);
        }
        
        string base64Image = System.Convert.ToBase64String(imageBytes);
        Destroy(screenshot);
        
        return new Dictionary<string, object>
        {
            {"image_base64", base64Image},
            {"output_path", outputPath},
            {"width", captureWidth},
            {"height", captureHeight}
        };
    }
    
    /// <summary>
    /// Start video streaming.
    /// </summary>
    public static bool StartStreaming(string robotId, string streamUrl)
    {
        RobotCamera instance = GetInstance(robotId);
        if (instance == null) return false;
        
        instance.StartStreamingInstance(streamUrl);
        return true;
    }
    
    private void StartStreamingInstance(string url)
    {
        isStreaming = true;
        streamUrl = url;
        Debug.Log($"{gameObject.name} camera streaming started: {url}");
        // TODO: Implement actual streaming (WebRTC, MJPEG, etc.)
    }
    
    /// <summary>
    /// Stop video streaming.
    /// </summary>
    public static bool StopStreaming(string robotId)
    {
        RobotCamera instance = GetInstance(robotId);
        if (instance == null) return false;
        
        instance.StopStreamingInstance();
        return true;
    }
    
    private void StopStreamingInstance()
    {
        isStreaming = false;
        streamUrl = "";
        Debug.Log($"{gameObject.name} camera streaming stopped");
    }
    
    /// <summary>
    /// Get camera status and settings.
    /// </summary>
    public static Dictionary<string, object> GetCameraStatus(string robotId)
    {
        RobotCamera instance = GetInstance(robotId);
        if (instance == null) return null;
        
        return instance.GetCameraStatusInstance();
    }
    
    private Dictionary<string, object> GetCameraStatusInstance()
    {
        return new Dictionary<string, object>
        {
            {"isStreaming", isStreaming},
            {"streamUrl", streamUrl},
            {"fieldOfView", robotCamera != null ? robotCamera.fieldOfView : 0f},
            {"width", captureWidth},
            {"height", captureHeight},
            {"hasCamera", robotCamera != null}
        };
    }
    
    private static RobotCamera GetInstance(string robotId)
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
        
        RobotCamera camera = robot.GetComponent<RobotCamera>();
        if (camera == null)
        {
            camera = robot.AddComponent<RobotCamera>();
        }
        
        instances[robotId] = camera;
        return camera;
    }
    
    void OnDestroy()
    {
        instances.Remove(gameObject.name);
    }
}

