using UnityAgent.Config;
using UnityAgent.Management.Message;

namespace UnityAgent.SystemMessage
{
public static class HeartbeatMessage
{
    public static EgressSystemMessage Message = new(CreateRoutingInfo());

    private static RoutingInfo CreateRoutingInfo()
    {
        RoutingData rd = new();
        RoutingManagementData rmd = new Heartbeat();
        RoutingInfo ri = new RoutingInfo(rd, rmd);
        return ri;
    }
}
}