using UnityEngine;

namespace Scene.RosAgentInspectorMenu
{
public class RosAgentInspectorMenu
{
    private GameObject menu;
    private ushort agentId;

    public RosAgentInspectorMenu(ushort agentId, GameObject menu)
    {
        this.agentId = agentId;
        this.menu = menu;
    }

    public ushort GetAgentId()
    {
        return agentId;
    }

    public GameObject GetGameObject()
    {
        return menu;
    }

    public void Show()
    {
        menu.SetActive(true);
    }

    public void Hide()
    {
        menu.SetActive(false);
    }
}
}