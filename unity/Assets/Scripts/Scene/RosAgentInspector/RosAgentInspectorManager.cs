using System.Collections.Generic;

using UnityEngine;
using UnityEngine.UI;
using TMPro;

namespace Scene.RosAgentInspectorMenu
{
public class RosAgentInspectorManager : MonoBehaviour
{
    private GameObject menuContainer;
    private GameObject menuTemplate;
    private LinkedList<RosAgentInspectorMenu> menus = new();
    private const int maxInspectorMenus = 7;
    
    private class Nested
    {
        static Nested()
        {
        }

        internal static readonly RosAgentInspectorManager instance =
            GetOrCreateInstance();

        private static RosAgentInspectorManager GetOrCreateInstance()
        {
            RosAgentInspectorManager instance =
                FindObjectOfType<RosAgentInspectorManager>();
            if (instance != null)
                return instance;
            GameObject obj = new("RosAgentInspectorManager");
            RosAgentInspectorManager newInstance =
                obj.AddComponent<RosAgentInspectorManager>();
            return newInstance;
        }
    }

    private RosAgentInspectorManager()
    {
    }

    public static RosAgentInspectorManager Instance => Nested.instance;

    private void Start()
    {
        menuContainer = GameObject.Find("RosAgentInspectorMenusContainer");
        menuTemplate = GameObject.Find("RosAgentInspectorMenuTemplate");
        menuTemplate.SetActive(false);
    }

    public bool IsOperatingAgent(ushort agentId)
    {
        return FindNode(agentId) != null;
    }

    public void StartAgentOperation(ushort agentId)
    {
        if (IsFull()) return;
        RosAgentInspectorMenu newMenu = CreateMenu(agentId);
        ComputeNewMenuPosition(newMenu);
        menus.AddLast(newMenu);
        newMenu.Show();
    }

    private bool IsFull()
    {
        return !(menus.Count < maxInspectorMenus);
    }

    private RosAgentInspectorMenu CreateMenu(ushort agentId)
    {
        GameObject menuObj = Instantiate(menuTemplate, menuContainer.transform);
        CreateMenuTitle(agentId, menuObj);
        ConfigureComponents(agentId, menuObj);
        RosAgentInspectorMenu menu = new(agentId, menuObj);
        return menu;
    }

    private void CreateMenuTitle(ushort agentId, GameObject menu)
    {
        string treeLocation = "OverviewContainer/Title";
        Transform title = menu.transform.Find(treeLocation);
        TextMeshProUGUI textMeshPro = title.GetComponentInChildren<TextMeshProUGUI>();
        textMeshPro.text = $"RosAgent {agentId}";
    }

    private void ConfigureComponents(ushort agentId, GameObject menu)
    {
        ConfigureCameraStreamSubscriber(agentId, menu.transform);
        ConfigureRobotManeuveringPublisher(agentId, menu.transform);
    }

    public void ConfigureCameraStreamSubscriber(ushort agentId, Transform menu)
    {
        CameraStreamSubscriber script = menu.GetComponent<CameraStreamSubscriber>();
        script.SetAgentId(agentId);
    }

    public void ConfigureRobotManeuveringPublisher(ushort agentId, Transform menu)
    {
        RobotManeuveringPublisher script = menu.GetComponent<RobotManeuveringPublisher>();
        script.SetAgentId(agentId);
    }

    private void ComputeNewMenuPosition(RosAgentInspectorMenu menu)
    {
        UpdateMenuPosition(menu, menus.Count);
    }

    public void StopAgentOperation(ushort agentId)
    {
        LinkedListNode<RosAgentInspectorMenu> node = FindNode(agentId);
        if (node == null) return;
        GameObject menu = node.Value.GetGameObject();
        DetachFromManeuverNode(menu);
        DetachFromVideoStreamNode(menu);
        Destroy(menu);
        menus.Remove(node);
        UpdateMenuPositions();
    }

    private void DetachFromManeuverNode(GameObject menu)
    {
        RobotManeuveringPublisher script = menu.GetComponent<RobotManeuveringPublisher>();
        script.DetachFromInterfaceNode();
    }

    private void DetachFromVideoStreamNode(GameObject menu)
    {
        CameraStreamSubscriber script = menu.GetComponent<CameraStreamSubscriber>();
        script.DetachFromInterfaceNode();
    }

    private void UpdateMenuPositions()
    {
        int count = 0;
        foreach (RosAgentInspectorMenu menu in menus)
            UpdateMenuPosition(menu, count++);
    }

    private void UpdateMenuPosition(RosAgentInspectorMenu menu, int positionId)
    {
        GameObject menuGameObj = menu.GetGameObject();
        Vector3 position = RosAgentInspectorMenuPositioner.GetPosition(positionId);
        Quaternion rotation = RosAgentInspectorMenuPositioner.GetRotation(positionId);
        menuGameObj.transform.position = position;
        menuGameObj.transform.rotation = rotation;
    }
    
    public void StopAllAgentOperations()
    {
        DestroyAllMenuGameObjects();
        menus.Clear();
    }

    private void DestroyAllMenuGameObjects()
    {
        foreach (RosAgentInspectorMenu menu in menus)
            Destroy(menu.GetGameObject());
    }

    public void TexturizeScreen(Texture2D screenTexture, ushort agentId)
    {
        
        LinkedListNode<RosAgentInspectorMenu> node = FindNode(agentId);
        if (node == null) return;
        GameObject inspectorMenu = node.Value.GetGameObject();
        Transform screen = inspectorMenu.transform.Find("CameraStreamContainer/Screen");
        screen.GetComponent<RawImage>().texture = screenTexture;
    }

    private LinkedListNode<RosAgentInspectorMenu> FindNode(ushort agentId)
    {
        LinkedListNode<RosAgentInspectorMenu> currentNode = menus.First;
        while (currentNode != null)
        {
            RosAgentInspectorMenu menu = currentNode.Value;
            if (menu.GetAgentId() == agentId)
                return currentNode;
            currentNode = currentNode.Next;
        }
        return currentNode;
    }
}
}