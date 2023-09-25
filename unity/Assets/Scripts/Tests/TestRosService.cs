using System;
using System.Collections.Generic;

using RosMessageTypes.UnityRoboticsDemo;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

using UnityAgent.Operation;
using UnityAgent.Interface.Registry;
using UnityAgent.Interface.RosAgent;

public class RosServiceCallExample : MonoBehaviour
{

    private Agent agent;
    private RosAgentInterfaceNodesRegistryManager registryManager;
    private RosAgentInterfaceClientNode clientNode;
    private GameObject cube;
    private string serviceName = "pos_srv";
    // The RosAgent must have id 1, thus, it must be connected to the
    // RosRouter as the first client.
    List<ushort> rosAgentIds = new List<ushort> { 1 };
    private uint count = 0;

    // Cube movement conditions.
    private float delta = 1.0f;
    private float speed = 2.0f;
    private Vector3 destination;
    float awaitingResponseUntilTimestamp = -1;

    private void Awake()
    {
        agent = Agent.Instance;
        agent.Run();
    }

    private void Start()
    {
        cube = GameObject.Find("Cube");
        registryManager = RosAgentInterfaceNodesRegistryManager.Instance;
        clientNode = registryManager.RegisterInterfaceClientNode<PositionServiceRequest>(
            serviceName, rosAgentIds);
        destination = cube.transform.position;
    }

    private async void Update()
    {
        float step = speed * Time.deltaTime;
        cube.transform.position = Vector3.MoveTowards(cube.transform.position,
                                                      destination, step);
        if (HasDestinationReached() && IsTimeoutOver())
        {
            Debug.Log("Destination reached.");
            PosRotMsg cubePos = new PosRotMsg(
                cube.transform.position.x,
                cube.transform.position.y,
                cube.transform.position.z,
                cube.transform.rotation.x,
                cube.transform.rotation.y,
                cube.transform.rotation.z,
                cube.transform.rotation.w
            );
            PositionServiceRequest request = new(cubePos);
            ProcessRequest(request);
            Debug.Log("RosAgent response received.");
            SetMinWaitTimeBetweenServiceCalls();
        }
    }

    private bool HasDestinationReached()
    {
        return Vector3.Distance(cube.transform.position, destination) < delta;
    }

    private bool IsTimeoutOver()
    {
        return Time.time > awaitingResponseUntilTimestamp;
    }

    private async void ProcessRequest(PositionServiceRequest request)
    {
        if (count % 2 == 0)
        {
            PositionServiceResponse response = await
                clientNode.MakeRequest<PositionServiceResponse>(request, 1);
            if (response != null)
                SetNewDestination(response);
        }
        else
            clientNode.MakeRequest<PositionServiceResponse>(request, 1,
                                                            SetNewDestination);
        count++;
    }

    void SetNewDestination(PositionServiceResponse response)
    {
        Debug.Log("Setting new destination ...");
        awaitingResponseUntilTimestamp = -1;
        destination = new Vector3(response.output.pos_x, response.output.pos_y,
                                  response.output.pos_z);
    }

    private void SetMinWaitTimeBetweenServiceCalls()
    {
        awaitingResponseUntilTimestamp = Time.time + 2.0f; // Seconds
    }
}