using UnityEngine;
using System.Collections.Generic;

/// <summary>
/// Animation controller for virtual robots in Unity.
/// Handles wheel rotation, movement animations, poses, and custom animations.
/// Used by robotics-mcp robot_animation tool via execute_unity_method.
/// </summary>
public class RobotAnimator : MonoBehaviour
{
    private static Dictionary<string, RobotAnimator> instances = new Dictionary<string, RobotAnimator>();
    
    [Header("Animation Settings")]
    public float wheelRotationSpeed = 100f;
    public float animationSpeed = 1f;
    
    private Animator animator;
    private Dictionary<string, GameObject> wheels = new Dictionary<string, GameObject>();
    private string currentAnimation = "";
    private bool isAnimating = false;
    
    void Awake()
    {
        animator = GetComponent<Animator>();
        if (animator == null)
        {
            animator = gameObject.AddComponent<Animator>();
        }
        
        // Find wheels by name (Scout has 4 mecanum wheels)
        FindWheels();
    }
    
    void FindWheels()
    {
        wheels.Clear();
        Transform[] allChildren = GetComponentsInChildren<Transform>();
        foreach (Transform child in allChildren)
        {
            string name = child.name.ToLower();
            if (name.Contains("wheel"))
            {
                if (name.Contains("front") && name.Contains("left"))
                    wheels["front_left"] = child.gameObject;
                else if (name.Contains("front") && name.Contains("right"))
                    wheels["front_right"] = child.gameObject;
                else if (name.Contains("back") && name.Contains("left"))
                    wheels["back_left"] = child.gameObject;
                else if (name.Contains("back") && name.Contains("right"))
                    wheels["back_right"] = child.gameObject;
            }
        }
    }
    
    /// <summary>
    /// Animate wheels with individual speeds (for mecanum wheels).
    /// </summary>
    public static bool AnimateWheels(string robotId, Dictionary<string, float> wheelSpeeds)
    {
        RobotAnimator instance = GetInstance(robotId);
        if (instance == null) return false;
        
        instance.AnimateWheelsInstance(wheelSpeeds);
        return true;
    }
    
    private void AnimateWheelsInstance(Dictionary<string, float> wheelSpeeds)
    {
        foreach (var kvp in wheelSpeeds)
        {
            if (wheels.ContainsKey(kvp.Key))
            {
                GameObject wheel = wheels[kvp.Key];
                float rotation = kvp.Value * wheelRotationSpeed * Time.deltaTime;
                wheel.transform.Rotate(0, 0, rotation, Space.Self);
            }
        }
    }
    
    /// <summary>
    /// Animate movement (walk, turn, etc.).
    /// </summary>
    public static bool AnimateMovement(string robotId, string animationName, float speed, bool loop)
    {
        RobotAnimator instance = GetInstance(robotId);
        if (instance == null) return false;
        
        instance.AnimateMovementInstance(animationName, speed, loop);
        return true;
    }
    
    private void AnimateMovementInstance(string animationName, float speed, bool loop)
    {
        if (animator != null && animator.runtimeAnimatorController != null)
        {
            animator.speed = speed;
            animator.SetBool("Loop", loop);
            animator.Play(animationName);
            currentAnimation = animationName;
            isAnimating = true;
        }
        else
        {
            Debug.LogWarning($"No Animator Controller found for {gameObject.name}. Movement animation '{animationName}' not played.");
        }
    }
    
    /// <summary>
    /// Set robot pose (sit, stand, crouch, etc.).
    /// </summary>
    public static bool SetPose(string robotId, string pose)
    {
        RobotAnimator instance = GetInstance(robotId);
        if (instance == null) return false;
        
        instance.SetPoseInstance(pose);
        return true;
    }
    
    private void SetPoseInstance(string pose)
    {
        if (animator != null && animator.runtimeAnimatorController != null)
        {
            animator.SetTrigger(pose);
            Debug.Log($"{gameObject.name} pose set to: {pose}");
        }
        else
        {
            Debug.LogWarning($"No Animator Controller found for {gameObject.name}. Pose '{pose}' not set.");
        }
    }
    
    /// <summary>
    /// Play custom animation.
    /// </summary>
    public static bool PlayAnimation(string robotId, string animationName, float speed, bool loop)
    {
        RobotAnimator instance = GetInstance(robotId);
        if (instance == null) return false;
        
        instance.PlayAnimationInstance(animationName, speed, loop);
        return true;
    }
    
    private void PlayAnimationInstance(string animationName, float speed, bool loop)
    {
        if (animator != null && animator.runtimeAnimatorController != null)
        {
            animator.speed = speed;
            animator.SetBool("Loop", loop);
            animator.Play(animationName);
            currentAnimation = animationName;
            isAnimating = true;
        }
        else
        {
            Debug.LogWarning($"No Animator Controller found for {gameObject.name}. Animation '{animationName}' not played.");
        }
    }
    
    /// <summary>
    /// Stop current animation.
    /// </summary>
    public static bool StopAnimation(string robotId)
    {
        RobotAnimator instance = GetInstance(robotId);
        if (instance == null) return false;
        
        instance.StopAnimationInstance();
        return true;
    }
    
    private void StopAnimationInstance()
    {
        if (animator != null)
        {
            animator.speed = 0f;
            isAnimating = false;
            currentAnimation = "";
        }
    }
    
    /// <summary>
    /// Get current animation state.
    /// </summary>
    public static Dictionary<string, object> GetAnimationState(string robotId)
    {
        RobotAnimator instance = GetInstance(robotId);
        if (instance == null) return null;
        
        return instance.GetAnimationStateInstance();
    }
    
    private Dictionary<string, object> GetAnimationStateInstance()
    {
        return new Dictionary<string, object>
        {
            {"currentAnimation", currentAnimation},
            {"isAnimating", isAnimating},
            {"animationSpeed", animator != null ? animator.speed : 0f},
            {"hasAnimator", animator != null && animator.runtimeAnimatorController != null}
        };
    }
    
    private static RobotAnimator GetInstance(string robotId)
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
        
        RobotAnimator animator = robot.GetComponent<RobotAnimator>();
        if (animator == null)
        {
            animator = robot.AddComponent<RobotAnimator>();
        }
        
        instances[robotId] = animator;
        return animator;
    }
    
    void OnDestroy()
    {
        instances.Remove(gameObject.name);
    }
}

