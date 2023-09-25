using UnityEngine;
using UnityEngine.UI;
using TMPro;

using Scene.RosAgentInspectorMenu;

namespace Scene.ConnectedAgentsMenu
{
public class RosAgentOperationButton : MonoBehaviour
{
    public Button operationButton;
    private RosAgentInspectorManager inspectorManager;
    private ushort agentId;
    private bool isOperating = false;
    
    public void SetInitState(ushort agentId)
    {
        this.agentId = agentId;
        if (RosAgentInspectorManager.Instance.IsOperatingAgent(agentId))
        {
            isOperating = true;
            GetTextComponent().text = "Stop";
        }
        else
        {
            isOperating = false;
            GetTextComponent().text = "Start";
        }
    }

    private void Start()
    {
        inspectorManager = RosAgentInspectorManager.Instance;
        operationButton.onClick.AddListener(HandleOperation);
    }

    private void HandleOperation()
    {
        if (isOperating) StopOperation();
        else StartOperation();
    }

    private void StopOperation()
    {
        isOperating = false;
        inspectorManager.StopAgentOperation(agentId);
        GetTextComponent().text = "Start";
    }

    private void StartOperation()
    {
        isOperating = true;
        inspectorManager.StartAgentOperation(agentId);
        GetTextComponent().text = "Stop";
    }

    private TextMeshProUGUI GetTextComponent()
    {
        return this.GetComponentInChildren<TextMeshProUGUI>();
    }
}
}