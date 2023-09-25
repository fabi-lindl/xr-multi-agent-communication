using System.Collections.Generic;

using UnityEngine;

using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosColor = RosMessageTypes.UnityRoboticsDemo.UnityColorMsg;

using UnityAgent.Operation;
using UnityAgent.Interface.Registry;
using UnityAgent.Interface.RosAgent;

public class TestRosSubscriber : MonoBehaviour
{
    private string topicName = "color";
    private Agent agent;
    private RosAgentInterfaceNodesRegistryManager registryManager;
    private RosAgentInterfaceSubscriberNode subscriberNode;
    private new GameObject gameObject;

    private void Awake()
    {
        agent = Agent.Instance;
        agent.Run();
    }

    private void Start()
    {
        gameObject = GameObject.Find("Cube");
        registryManager = RosAgentInterfaceNodesRegistryManager.Instance;
        // The RosAgent must have id 1, thus, it must be connected to the
        // RosRouter as the first client.
        List<ushort> agentIds = new List<ushort> { 1 };
        subscriberNode = registryManager.RegisterInterfaceSubscriberNode<RosColor>(
            ChangeCubeColor, topicName, agentIds);
    }

    void ChangeCubeColor(RosColor colorMessage, ushort agentId)
    {
        Debug.Log($"Received message for agent of id {agentId}.");
        gameObject.GetComponent<Renderer>().material.color =
            new Color32((byte)colorMessage.r,
                        (byte)colorMessage.g,
                        (byte)colorMessage.b,
                        (byte)colorMessage.a);
    }
}