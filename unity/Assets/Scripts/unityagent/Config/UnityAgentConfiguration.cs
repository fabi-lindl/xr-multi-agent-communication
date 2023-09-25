namespace UnityAgent.Config
{
public static class UnityAgentConfiguration
{
    private static byte agentType = 2;
    private static ushort agentId;
    private static byte groupId = 0;

    public static byte AgentType => agentType;
    public static ushort AgentId
    {
        get => agentId;
        set => agentId = value;
    }
    public static byte GroupId => groupId;

    public static bool BelongsToGroup()
    {
        return groupId > 0;
    }
}
}