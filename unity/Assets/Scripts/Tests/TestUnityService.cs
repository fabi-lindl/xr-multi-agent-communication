using System;
using System.Collections.Generic;

using UnityEngine;

using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;

using RosMessageTypes.UnityRoboticsDemo;

using UnityAgent.Operation;
using UnityAgent.Interface.Registry;
using UnityAgent.Interface.RosAgent;

public class RosUnityServiceExample : MonoBehaviour
{
    string serviceName = "obj_pose_srv";
    private Agent agent;
    private RosAgentInterfaceNodesRegistryManager registryManager;
    private RosAgentInterfaceServiceNode serviceNode;
    private new GameObject gameObject;
    ObjectPoseServiceResponse objectPoseResponse = new();

    private void Awake()
    {
        agent = Agent.Instance;
        agent.Run();
    }

    void Start()
    {
        gameObject = GameObject.Find("Cube");
        registryManager = RosAgentInterfaceNodesRegistryManager.Instance;
        // The RosAgent must have id 1, thus, it must be connected to the
        // RosRouter as the first client.
        List<ushort> agentIds = new List<ushort> { 1 };
        serviceNode = registryManager.RegisterInterfaceServiceNode<ObjectPoseServiceRequest, ObjectPoseServiceResponse>(
            GetObjectPose, serviceName, agentIds);
    }

    void OnApplicationQuit()
    {
        agent = Agent.Instance;
        agent.Stop();
    }

    private ObjectPoseServiceResponse GetObjectPose(ObjectPoseServiceRequest request)
    {
        Debug.Log("Process unity service call ...");
        objectPoseResponse.object_pose.position = gameObject.transform
                                                  .position.To<FLU>();
        objectPoseResponse.object_pose.orientation = gameObject.transform
                                                     .rotation.To<FLU>();
        return objectPoseResponse;
    }
}