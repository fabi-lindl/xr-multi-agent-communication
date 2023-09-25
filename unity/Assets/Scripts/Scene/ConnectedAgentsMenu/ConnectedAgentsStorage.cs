using System.Collections.Generic;

namespace Scene.ConnectedAgentsMenu
{
public class ConnectedAgentsStorage
{
    public List<ushort> rosAgentIds;
    public List<ushort> unityAgentIds;

    public List<ushort> GetRosAgentIds => rosAgentIds;
    public List<ushort> GetUnityAgentIds => unityAgentIds;
}
}