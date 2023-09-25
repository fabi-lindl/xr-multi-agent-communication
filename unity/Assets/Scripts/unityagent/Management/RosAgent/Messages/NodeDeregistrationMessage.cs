using UnityAgent.Operation;

namespace UnityAgent.Management.RosAgent.Message
{
public class NodeDeregistrationMessage
{
    public byte registrantAgentType = Agent.Instance.AgentType;
    public ushort registrantAgentId = Agent.Instance.AgentId;
    public byte nodeType;
    public string nodeName;

    public NodeDeregistrationMessage(byte nodeType, string nodeName)
    {
        this.nodeType = nodeType;
        this.nodeName = nodeName;
    }
}
}