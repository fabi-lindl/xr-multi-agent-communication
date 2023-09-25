using System.Collections.Generic;

using UnityEngine;
using UnityEngine.UI;

using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Sensor;

using UnityAgent.Operation;
using UnityAgent.Interface.Registry;
using UnityAgent.Interface.RosAgent;

public class TestVideostreamSubscriber : MonoBehaviour
{
    private string topicName = "video_frames";
    private Agent agent;
    private RosAgentInterfaceNodesRegistryManager registryManager;
    private RosAgentInterfaceSubscriberNode subscriberNode;
    private new GameObject gameObject;
    private bool debayer = true;

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
        subscriberNode = registryManager.RegisterInterfaceSubscriberNode<ImageMsg>(
            DisplayVideoFrame, topicName, agentIds);
    }

    void DisplayVideoFrame(ImageMsg message, ushort agentId)
    {
        Debug.Log($"Received video frame message for agent of id {agentId}.");
        if (message == null) return;
        Texture2D texture = message.ToTexture2D(debayer);
        gameObject.GetComponent<Renderer>().material.mainTexture = texture;
    }
}