using System.Collections.Generic;

using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.UI;
using TMPro;

using UnityAgent.Operation;
using UnityAgent.SystemMessage;
using UnityAgent.Management.Message;
using UnityAgent.Util.Serialization;

using Scene.ConnectedAgentsMenu;

namespace Scene.ConnectedAgentsMenu
{
public class ConnectedAgentsMenuManager : MonoBehaviour
{
    public GameObject maneuverCube;
    public GameObject connectedAgentsMenu;
    public Button refreshButton;
    
    private GameObject rosAgentsContainer;
    private GameObject rosAgentsTableBody;
    private GameObject rosAgentsTableRowTemplate;

    private GameObject unityAgentsContainer;
    private GameObject unityAgentsTableBody;
    private GameObject unityAgentsTableRowTemplate;

    private const float tableRowXoffset = 0.0028f;
    private const float tableRowYoffset = 0.0024f;
    private const float tableRowTemplateHeight = 0.014f;

    private ConnectedAgentsStorage connectedAgentsStorage;

    private class Nested
    {
        static Nested()
        {
        }

        internal static readonly ConnectedAgentsMenuManager instance =
            GetOrCreateInstance();
        
        private static ConnectedAgentsMenuManager GetOrCreateInstance()
        {
            ConnectedAgentsMenuManager instance =
                FindObjectOfType<ConnectedAgentsMenuManager>();
            if (instance != null) return instance;
            GameObject obj = new("ConnectedAgentsMenuManager");
            ConnectedAgentsMenuManager newInstance =
                obj.AddComponent<ConnectedAgentsMenuManager>();
            return newInstance;
        }
    }

    private ConnectedAgentsMenuManager()
    {
    }

    public static ConnectedAgentsMenuManager Instance => Nested.instance;

    public void HideMenu()
    {
        connectedAgentsMenu.SetActive(false);
    }

    public void ShowMenu()
    {
        connectedAgentsMenu.SetActive(true);
    }

    public void CleanupMenu()
    {
        ClearTables();
        connectedAgentsStorage = null;
    }

    private void Start()
    {
        FindRosAgentGameObjects();
        FindUnityAgentGameObjects();
        HideMenu();
        refreshButton.onClick.AddListener(HandleRefreshButtonClick);
    }

    private void FindRosAgentGameObjects()
    {
        rosAgentsContainer = GameObject.Find("ConnectedRosAgentsContainer");
        rosAgentsTableBody = GameObject.Find("RosAgentsTableBody");
        rosAgentsTableRowTemplate = GameObject.Find("RosAgentsTableRowTemplate");
        rosAgentsTableRowTemplate.SetActive(false);
    }

    private void FindUnityAgentGameObjects()
    {
        unityAgentsContainer = GameObject.Find("ConnectedUnityAgentsContainer");
        unityAgentsTableBody = GameObject.Find("UnityAgentsTableBody");
        unityAgentsTableRowTemplate = GameObject.Find("UnityAgentsTableRowTemplate");
        unityAgentsTableRowTemplate.SetActive(false);
    }

    private void OnEnable()
    {
        if (IsConnected() && !HasData()) HandleRefreshButtonClick();
    }

    private bool IsConnected()
    {
        return Agent.Instance.AgentId > 0;
    }

    private bool HasData()
    {
        return connectedAgentsStorage != null;
    }

    private void HandleRefreshButtonClick()
    {
        RefreshConnectedAgentsTable();
        ResetManeuverCubePose();
    }

    private async void RefreshConnectedAgentsTable()
    {
        RoutingManagementData rmd = new OnlineAgentsRequest();
        IngressSystemMessage ism = await Communicator.SendRequestToRouter(rmd);
        if (ism == null) return;
        connectedAgentsStorage = NetworkDataDeserializer
                                 .BytesToObj<ConnectedAgentsStorage>(ism.Data);
        RenderConnectedAgents();
    }

    private void ResetManeuverCubePose()
    {
        maneuverCube.transform.position = new Vector3(0, 0, 0.5f);
        maneuverCube.transform.rotation = Quaternion.Euler(0, 0, 0);
    }

    private void RenderConnectedAgents()
    {
        RenderRosAgentsTable();
        RenderUnityAgentsTable();
    }

    private void RenderRosAgentsTable()
    {
        ClearTable(rosAgentsTableBody);
        PopulateTable(connectedAgentsStorage.GetRosAgentIds,
                      rosAgentsTableRowTemplate,
                      rosAgentsTableBody);
    }

    private void RenderUnityAgentsTable()
    {
        ClearTable(unityAgentsTableBody);
        PopulateTable(connectedAgentsStorage.GetUnityAgentIds,
                      unityAgentsTableRowTemplate,
                      unityAgentsTableBody);
    }

    private void ClearTable(GameObject tableBody)
    {
        int count = 0;
        foreach (Transform child in tableBody.transform)
            if (count++ > 0) Destroy(child.gameObject);
    }

    private void ClearTables()
    {
        ClearTable(rosAgentsTableBody);
        ClearTable(unityAgentsTableBody);
    }

    private void PopulateTable(List<ushort> agentIds, GameObject template,
                               GameObject parent)
    {
        int count = 0;
        foreach (ushort agentId in agentIds)
            CreateTableRow(template, parent, agentId, count++);
    }

    private void CreateTableRow(GameObject template, GameObject parent,
                                ushort agentId, int numEntry)
    {
        GameObject tableRow = Instantiate(template, parent.transform);
        FillTableRowItems(tableRow, agentId);
        PositionTableRow(tableRow, numEntry);
        tableRow.SetActive(true);
    }

    private void FillTableRowItems(GameObject tableRow, ushort agentId)
    {
        Transform tableRowTransform = tableRow.transform;
        FillTableRowTitle(tableRowTransform, agentId);
        FillTableRowButton(tableRowTransform, agentId);
    }

    private void FillTableRowTitle(Transform tableRow, ushort agentId)
    {
        Transform title = tableRow.Find("Title");
        TextMeshProUGUI textComponent = title.GetComponent<TextMeshProUGUI>();
        textComponent.text = agentId.ToString();
    }

    private void FillTableRowButton(Transform tableRow, ushort agentId)
    {
        try
        {
            Transform button = tableRow.Find("Button");
            RosAgentOperationButton script = button.GetComponent<RosAgentOperationButton>();
            script.SetInitState(agentId);
        }
        catch
        {
            // Unity operation button is not implemented for the demo.
            // The component cannot be found. Catch the error.
        }
    }

    private void PositionTableRow(GameObject tableRow, int numEntry)
    {
        RectTransform trRectTransform = tableRow.GetComponent<RectTransform>();
        float yOffset = -((tableRowTemplateHeight + tableRowYoffset) * numEntry);
        Vector2 position = new(tableRowXoffset, yOffset);
        trRectTransform.anchoredPosition = position;
    }
}
}