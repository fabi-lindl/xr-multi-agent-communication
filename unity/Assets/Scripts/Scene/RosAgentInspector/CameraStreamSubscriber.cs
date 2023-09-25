using System;
using System.Collections.Generic;

using UnityEngine;
using UnityEngine.UI;
using TMPro;

using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Sensor;

using UnityAgent.Interface.Registry;
using UnityAgent.Interface.RosAgent;

using Scene.RosAgentInspectorMenu;

namespace Scene.RosAgentInspectorMenu
{
public class CameraStreamSubscriber : MonoBehaviour
{
    public GameObject screenContainer;
    public GameObject screen;
    public Button streamButton;

    private RosAgentInterfaceNodesRegistryManager registryManager;
    private RosAgentInterfaceSubscriberNode subscriberNode;
    private RosAgentInspectorManager inspectorMenuManager;
    private ushort agentId;
    private bool isStreaming = false;
    private bool debayer = true;
    private const string topicName = "video_frames";

    public void SetAgentId(ushort agentId)
    {
        this.agentId = agentId;
    }

    private void Start()
    {
        registryManager = RosAgentInterfaceNodesRegistryManager.Instance;
        inspectorMenuManager = RosAgentInspectorManager.Instance;
        streamButton.onClick.AddListener(HandleStream);
        HideScreen();
    }

    private void HandleStream()
    {
        if (isStreaming) StopStreaming();
        else StartStreaming();
    }

    private void StopStreaming()
    {
        if (!isStreaming) return;
        isStreaming = false;
        DetachFromInterfaceNode();
        ChangeButtonText();
        HideScreen();
        Debug.Log($"[Agent {agentId}] Stopped streaming!");
    }

    private void StartStreaming()
    {
        if (isStreaming) return;
        isStreaming = true;
        RegisterInterfaceNode();
        ChangeButtonText();
        ShowScreen();
        Debug.Log($"[Agent {agentId}] Start streaming!");
    }

    public void DetachFromInterfaceNode()
    {
        if (subscriberNode == null) return;
        subscriberNode.DeregisterAgent(agentId);
        subscriberNode = null;
    }

    private void RegisterInterfaceNode()
    {
        subscriberNode = registryManager.RegisterInterfaceSubscriberNode<ImageMsg>(
            DisplayStream, topicName, CreateAgentIds());
    }

    private void DisplayStream(ImageMsg message, ushort agentId)
    {
        Texture2D texture = message.ToTexture2D(debayer);
        inspectorMenuManager.TexturizeScreen(texture, agentId);
    }

    private List<ushort> CreateAgentIds()
    {
        return new List<ushort> { agentId };
    }

    private void HideScreen()
    {
        screenContainer.SetActive(false);
    }

    private void ShowScreen()
    {
        screenContainer.SetActive(true);
    }

    private void ChangeButtonText()
    {
        if (isStreaming) SetIsStreamingButtonText();
        else SetIsStoppedButtonText();
    }

    private void SetIsStreamingButtonText()
    {
        GetTextComponent().text = "Stop";
    }

    private void SetIsStoppedButtonText()
    {
        GetTextComponent().text = "Start";
    }

    private TextMeshProUGUI GetTextComponent()
    {
        return streamButton.GetComponentInChildren<TextMeshProUGUI>();
    }
}
}