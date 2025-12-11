---
title: "Chapter 2: Unity Visualization & Interaction"
sidebar_position: 3
---

# Chapter 2: Unity Visualization & Interaction

## Introduction

Unity provides a powerful platform for robot visualization and human-robot interaction. This chapter explores how to leverage Unity's capabilities to create immersive visualization environments and intuitive interaction mechanisms for humanoid robots.

## Learning Objectives

By the end of this chapter, you will be able to:
- Set up Unity for robot visualization
- Create realistic robot models in Unity
- Implement human-robot interaction interfaces
- Integrate Unity with ROS 2 for real-time data

## Unity Robotics Setup

Unity provides the Unity Robotics Hub for robot simulation and visualization:

```csharp
using UnityEngine;
using RosMessageTypes.Sensor;
using Unity.Robotics.ROSTCPConnector;

public class RobotVisualizer : MonoBehaviour
{
    // ROS connection
    private ROSConnection ros;
    
    // Robot joint transforms
    public Transform[] jointTransforms;
    
    void Start()
    {
        // Get the ROS connection static instance
        ros = ROSConnection.instance;
        
        // Subscribe to joint state messages
        ros.Subscribe<JointStateMsg>("joint_states", JointStateCallback);
    }
    
    void JointStateCallback(JointStateMsg jointState)
    {
        // Update joint transforms based on ROS message
        for (int i = 0; i < jointState.name.Count; i++)
        {
            // Find corresponding joint transform and update rotation
            // Implementation depends on joint naming convention
        }
    }
}
```

## Robot Model Import and Setup

When importing robot models into Unity:

1. **Import URDF**: Use the Unity URDF Importer package
2. **Configure Colliders**: Add appropriate colliders for physics interactions
3. **Set up Joints**: Configure Unity joints to match robot kinematics
4. **Material Assignment**: Apply appropriate materials for visualization

## Visualization Techniques

### Real-time Robot Control Visualization

```csharp
public class RealTimeRobotController : MonoBehaviour
{
    [SerializeField] private GameObject robotModel;
    [SerializeField] private float updateRate = 60f;
    
    private float updateInterval;
    private float lastUpdateTime;
    
    void Start()
    {
        updateInterval = 1f / updateRate;
        lastUpdateTime = Time.time;
    }
    
    void Update()
    {
        if (Time.time - lastUpdateTime >= updateInterval)
        {
            UpdateRobotVisualization();
            lastUpdateTime = Time.time;
        }
    }
    
    private void UpdateRobotVisualization()
    {
        // Update robot model based on current state
        // This could involve updating joint angles, end-effector positions, etc.
    }
}
```

### Camera Systems for Robot Visualization

Unity supports various camera systems for robot visualization:

```csharp
public class RobotCameraController : MonoBehaviour
{
    public enum CameraMode { Follow, Orbit, TopDown, ThirdPerson };
    
    public CameraMode currentMode;
    public Transform robotTarget;
    public float followDistance = 5f;
    public float followHeight = 3f;
    
    void LateUpdate()
    {
        switch (currentMode)
        {
            case CameraMode.Follow:
                FollowRobot();
                break;
            case CameraMode.Orbit:
                OrbitRobot();
                break;
            // Other camera modes
        }
    }
    
    private void FollowRobot()
    {
        Vector3 targetPosition = robotTarget.position + new Vector3(0, followHeight, -followDistance);
        transform.position = Vector3.Lerp(transform.position, targetPosition, Time.deltaTime * 5f);
        transform.LookAt(robotTarget);
    }
}
```

## Human-Robot Interaction Interfaces

### GUI for Robot Control

```csharp
using UnityEngine;
using UnityEngine.UI;

public class RobotControlGUI : MonoBehaviour
{
    public Slider[] jointSliders;
    public Button[] actionButtons;
    public Text statusText;
    
    void Start()
    {
        // Initialize control elements
        InitializeJointSliders();
        SetupActionButtons();
    }
    
    private void InitializeJointSliders()
    {
        for (int i = 0; i < jointSliders.Length; i++)
        {
            int jointIndex = i; // For closure
            jointSliders[i].onValueChanged.AddListener(delegate { JointSliderChanged(jointIndex); });
        }
    }
    
    private void JointSliderChanged(int jointIndex)
    {
        // Send joint command to robot
        float jointValue = jointSliders[jointIndex].value;
        SendJointCommand(jointIndex, jointValue);
    }
    
    private void SendJointCommand(int jointIndex, float value)
    {
        // Implementation to send command to robot
        // Could be via ROS, TCP/IP, etc.
    }
}
```

## Physics Simulation in Unity

For interaction simulation, Unity's physics engine can be configured:

```csharp
public class RobotPhysicsController : MonoBehaviour
{
    public Rigidbody robotBody;
    public ConfigurableJoint[] joints;
    
    [Header("Physics Parameters")]
    public float gravityScale = 1.0f;
    public float drag = 0.1f;
    public float angularDrag = 0.05f;
    
    void Start()
    {
        robotBody.useGravity = true;
        robotBody.drag = drag;
        robotBody.angularDrag = angularDrag;
        robotBody.mass = 10f; // Adjust based on robot mass
    }
    
    void FixedUpdate()
    {
        // Apply physics-based control
        ApplyJointForces();
    }
    
    private void ApplyJointForces()
    {
        // Apply forces to joints based on control inputs
        // Implementation depends on robot kinematics
    }
}
```

## Integration with ROS 2

Unity can communicate with ROS 2 through TCP/IP:

```csharp
using System.Net.Sockets;
using System.Text;
using Newtonsoft.Json;

public class ROS2Bridge : MonoBehaviour
{
    private TcpClient tcpClient;
    private NetworkStream stream;
    
    [SerializeField] private string ipAddress = "127.0.0.1";
    [SerializeField] private int port = 5005;
    
    void Start()
    {
        ConnectToROS2();
    }
    
    private void ConnectToROS2()
    {
        try
        {
            tcpClient = new TcpClient(ipAddress, port);
            stream = tcpClient.GetStream();
        }
        catch (System.Exception e)
        {
            Debug.LogError("Failed to connect to ROS 2: " + e.Message);
        }
    }
    
    public void SendRobotCommand(string command)
    {
        if (stream != null)
        {
            byte[] data = Encoding.ASCII.GetBytes(command);
            stream.Write(data, 0, data.Length);
        }
    }
    
    void OnDestroy()
    {
        if (tcpClient != null)
            tcpClient.Close();
    }
}
```

## Performance Optimization

For smooth visualization:
- Use appropriate LOD (Level of Detail) systems
- Optimize meshes and textures
- Use occlusion culling for complex scenes
- Implement object pooling for frequently instantiated objects

## Summary

Unity provides powerful capabilities for robot visualization and human-robot interaction. By integrating Unity with ROS 2, developers can create immersive environments for robot simulation, testing, and human-robot interaction design.
