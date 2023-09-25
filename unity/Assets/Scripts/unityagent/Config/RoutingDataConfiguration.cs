namespace UnityAgent.Config
{
public static class RoutingDataConfiguration
{
    // All field sizes are given in bytes.
    private const int agentTypeFieldSize = 1;
    private const int destAgentIdSize = 2;
    private const int destGroupIdSize = 1;
    private const int numDestAgentIds = 10;
    private const int numDestGroupIds = 10;
    private const int managementDataSizeFieldSize = 4;
    private const int totalMessageSizeFieldSize = 4;

    public static int GetMaxNumDestAgentIds()
    {
        return numDestAgentIds;
    }

    public static int GetMaxNumDestGroupIds()
    {
        return numDestGroupIds;
    }

    public static int GetSerializedRoutingDataSize()
    {
        return agentTypeFieldSize +
               (destAgentIdSize * numDestAgentIds) +
               (destGroupIdSize * numDestGroupIds) +
               managementDataSizeFieldSize +
               totalMessageSizeFieldSize;
    }
}
}