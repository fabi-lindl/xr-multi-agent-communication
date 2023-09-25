using System.Collections.Generic;

using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.UnityRoboticsDemo;

using UnityAgent.Operation;
using UnityAgent.Interface.Registry;
using UnityAgent.Interface.RosAgent;

public class TestRosPublisher : MonoBehaviour
{
    private string topicName = "pos_rot";
    private Agent agent;
    private RosAgentInterfaceNodesRegistryManager registryManager;
    private RosAgentInterfacePublisherNode publisherNode;
    public GameObject gameCube;
    private float publishMessagePeriod = 2f;
    private float timeElapsed = 0;

    private void Awake()
    {
        agent = Agent.Instance;
        agent.Run();
    }

    void Start()
    {
        gameCube = GameObject.Find("Cube");
        registryManager = RosAgentInterfaceNodesRegistryManager.Instance;
        // The RosAgent must have id 1, thus, it must be connected to the
        // RosRouter as the first client.
        List<ushort> agentIds = new List<ushort> { 1 };
        publisherNode = registryManager.RegisterInterfacePublisherNode<PosRotMsg>(
            topicName, agentIds);
    }

    private void Update()
    {
        timeElapsed += Time.deltaTime;
        if (timeElapsed > publishMessagePeriod)
        {
            gameCube.transform.rotation = Random.rotation;
            PosRotMsg cubePosMsg = new PosRotMsg(
                gameCube.transform.position.x,
                gameCube.transform.position.y,
                gameCube.transform.position.z,
                gameCube.transform.rotation.x,
                gameCube.transform.rotation.y,
                gameCube.transform.rotation.z,
                gameCube.transform.rotation.w
            );
            publisherNode.Publish(cubePosMsg);
            timeElapsed = 0;
        }
    }
}