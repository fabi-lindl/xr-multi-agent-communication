namespace UnityAgent.Management.Message
{
public class GoOffline : RoutingManagementData
{
    public string messageManager = "UnityAgentclientRemover";
    public ushort agentId;

    public GoOffline(ushort agentId)
    {
        this.agentId = agentId;
    }

    public override string MessageManager => messageManager;
}
}