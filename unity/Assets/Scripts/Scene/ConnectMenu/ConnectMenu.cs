using UnityEngine;
using UnityEngine.UI;
using TMPro;

using UnityAgent.Interface.Registry;
using UnityAgent.Operation;

using Scene.ConnectedAgentsMenu;
using Scene.RosAgentInspectorMenu;

namespace Scene.ConnectMenu
{
public class ConnectMenu : MonoBehaviour
{
    public GameObject connectMenu;
    public Button connectButton;
    public TextMeshProUGUI connectingText;

    private Agent agent;
    private ConnectedAgentsMenuManager connectedAgentsMenuManager;
    private RosAgentInspectorManager rosAgentInspectorManager;
    private RosAgentInterfaceNodesRegistry nodesRegistry;
    
    private bool isConnected = false;
    private bool isDisconnecting = false;

    private void Start()
    {
        agent = Agent.Instance;
        connectedAgentsMenuManager = ConnectedAgentsMenuManager.Instance;
        rosAgentInspectorManager = RosAgentInspectorManager.Instance;
        nodesRegistry = RosAgentInterfaceNodesRegistry.Instance;
        connectButton.onClick.AddListener(ConnectToRouter);
        SetMenuDisconnectedState();
    }

    private void ConnectToRouter()
    {
        if (isConnected) Disconnect();
        else Connect();
    }

    private void Connect()
    {
        if (isConnected) return;
        isConnected = true;
        SetMenuConnectingState();
        if (agent.Run())
        {
            SetMenuConnectedState();
            ShowConnectedAgentsMenu();
        }
        else
        {
            SetMenuDisconnectedState();
            isConnected = false;
        }
    }

    private void Disconnect()
    {
        if (isConnected && !isDisconnecting)
        {
            isDisconnecting = true;
            Debug.Log("Disconnecting ...");
            SetSceneDisconnectedState();
            agent.Stop();
            isDisconnecting = false;
            isConnected = false;
            Debug.Log("Disconnected!");
        }
    }

    private void SetMenuConnectingState()
    {
        HideConnectButton();
        ShowConnectingText();
    }

    private void SetMenuConnectedState()
    {
        HideConnectingText();
        ShowConnectedButtonText();
        ShowConnectButton();
    }

    private void SetMenuDisconnectedState()
    {
        ShowDisconnectedButtonText();
        HideConnectingText();
        ShowConnectButton();
    }

    private void SetSceneDisconnectedState()
    {
        SetMenuDisconnectedState();
        CleanupMenus();
        ClearNodesRegistry();
    }

    private void CleanupMenus()
    {
        CleanupRosAgentInspectorMenus();
        CleanupConnectedAgentsMenu();
    }

    private void CleanupRosAgentInspectorMenus()
    {
        rosAgentInspectorManager.StopAllAgentOperations();
    }

    private void CleanupConnectedAgentsMenu()
    {
        connectedAgentsMenuManager.HideMenu();
        connectedAgentsMenuManager.CleanupMenu();
    }

    private void ClearNodesRegistry()
    {
        nodesRegistry.Clear();
    }

    private void ShowConnectedAgentsMenu()
    {
        connectedAgentsMenuManager.ShowMenu();
    }

    private void HideConnectButton()
    {
        connectButton.gameObject.SetActive(false);
    }

    private void ShowConnectButton()
    {
        connectButton.gameObject.SetActive(true);
    }

    private void ShowConnectingText()
    {
        connectingText.gameObject.SetActive(true);
    }

    private void HideConnectingText()
    {
        connectingText.gameObject.SetActive(false);
    }

    private void ShowConnectedButtonText()
    {
        GetTextComponent().text = "Disconnect";
    }

    private void ShowDisconnectedButtonText()
    {
        GetTextComponent().text = "Connect";
    }

    private TextMeshProUGUI GetTextComponent()
    {
        return connectButton.GetComponentInChildren<TextMeshProUGUI>();
    }
}
}