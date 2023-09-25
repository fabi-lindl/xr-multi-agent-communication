using UnityAgent.Operation;

namespace UnityAgent.Management.Message
{
public class EgressAgentConfiguration : RoutingManagementData
{
    public byte agentType = Agent.Instance.AgentType;
    public ushort agentId;
    public byte groupId;
    public uint requestId;
    public string messageManager = "AgentclientConfigurer";

    public override string MessageManager => messageManager;

    public EgressAgentConfiguration(ushort agentId, byte groupId,
                                    uint requestId)
    {
        this.agentId = agentId;
        this.groupId = groupId;
        this.requestId = requestId;
    }
}
}