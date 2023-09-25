using UnityAgent.Operation;

namespace UnityAgent.Management.Message
{
public class OnlineAgentsRequest : RoutingManagementData
{
    public string messageManager = "OnlineAgentsGetter";
    public byte agentType = Agent.Instance.AgentType;
    public ushort agentId = Agent.Instance.AgentId;

    public OnlineAgentsRequest()
    {
    }

    public override string MessageManager => messageManager;
}
}