namespace UnityAgent.Management.Message
{
public class Heartbeat : RoutingManagementData
{
    public string messageManager = "HeartbeatHandler";
    public override string MessageManager => messageManager;
}
}