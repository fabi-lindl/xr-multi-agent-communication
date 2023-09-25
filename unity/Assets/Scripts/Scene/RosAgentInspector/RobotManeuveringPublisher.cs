using System;
using System.Collections.Generic;

using UnityEngine;
using UnityEngine.UI;
using TMPro;

using RosMessageTypes.UnityRoboticsDemo;

using UnityAgent.Interface.Registry;
using UnityAgent.Interface.RosAgent;

namespace Scene.RosAgentInspectorMenu
{
public class RobotManeuveringPublisher : MonoBehaviour
{
    public GameObject maneuverCube;
    public GameObject rosAgentInspectorMenu;
    public Button maneuverButton;

    private RosAgentInterfacePublisherNode publisherNode;
    private RosAgentInterfaceNodesRegistryManager registryManager;
    private ushort agentId;
    private bool isManeuvering = false;
    private const string topicName = "pos_rot";
    private const float publishMessagePeriod = 2.5f;
    private float elapsedTime = 0;

    public void SetAgentId(ushort agentId)
    {
        this.agentId = agentId;
    }

    private void Start()
    {
        registryManager = RosAgentInterfaceNodesRegistryManager.Instance;
        maneuverButton.onClick.AddListener(HandleManeuvering);
    }

    private void Update()
    {
        UpdateElapsedTime();
        if (IsReadyToPublish())
        {
            PublishManeuverPose();
            ResetElapsedTime();
        }
    }

    private void UpdateElapsedTime()
    {
        elapsedTime += Time.deltaTime;
    }

    private bool IsReadyToPublish()
    {
        return isManeuvering && IsPublishPeriodExceeded();
    }

    private bool IsPublishPeriodExceeded()
    {
        return elapsedTime > publishMessagePeriod;
    }

    private void PublishManeuverPose()
    {
        PosRotMsg poseMessage = new PosRotMsg(
            maneuverCube.transform.position.x,
            maneuverCube.transform.position.y,
            maneuverCube.transform.position.z,
            maneuverCube.transform.rotation.x,
            maneuverCube.transform.rotation.y,
            maneuverCube.transform.rotation.z,
            maneuverCube.transform.rotation.w
        );
        publisherNode.Publish(poseMessage);
    }

    private void ResetElapsedTime()
    {
        elapsedTime = 0;
    }

    private void HandleManeuvering()
    {
        if (isManeuvering) StopManeuvering();
        else StartManeuvering();
    }

    private void StopManeuvering()
    {
        if (!isManeuvering) return;
        isManeuvering = false;
        DetachFromInterfaceNode();
        ShowStoppedButtonText();
        ResetElapsedTime();
        Debug.Log($"[Agent {agentId}] Stopped maneuvering!");
    }

    public void DetachFromInterfaceNode()
    {
        if (publisherNode == null) return;
        publisherNode.DeregisterAgent(agentId);
        publisherNode = null;
    }

    private void StartManeuvering()
    {
        if (isManeuvering) return;
        isManeuvering = true;
        ShowStartedButtonText();
        publisherNode = registryManager.RegisterInterfacePublisherNode<PosRotMsg>(
            topicName, CreateAgentIds());
        Debug.Log($"[Agent {agentId}] Started maneuvering!");
    }

    private List<ushort> CreateAgentIds()
    {
        return new List<ushort> { agentId };
    }

    private void ShowStartedButtonText()
    {
        GetTextComponent().text = "Stop";
    }

    private void ShowStoppedButtonText()
    {
        GetTextComponent().text = "Start";
    }
    private TextMeshProUGUI GetTextComponent()
    {
        return maneuverButton.GetComponentInChildren<TextMeshProUGUI>();
    }
}
}