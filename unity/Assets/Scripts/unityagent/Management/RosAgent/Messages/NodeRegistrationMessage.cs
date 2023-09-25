using UnityEngine;

using UnityAgent.Operation;

namespace UnityAgent.Management.RosAgent.Message
{
public class NodeRegistrationMessage
{
    public byte registrantAgentType = Agent.Instance.AgentType;
    public ushort registrantAgentId = Agent.Instance.AgentId;
    public byte nodeType;
    public string nodeName;
    public string tosName;
    public string dataExchangeType;
    public uint bufferSize;
    public bool dropNew;
    // Only used by RosAgent InterfacePublisherNode.
    public bool isLatched;

    public NodeRegistrationMessage(
        byte nodeType,
        string nodeName,
        string tosName,
        string dataExchangeType,
        uint bufferSize,
        bool dropNew,
        bool isLatched)
    {
        this.registrantAgentId = Agent.Instance.AgentId;
        this.nodeType = nodeType;
        this.nodeName = nodeName;
        this.tosName = tosName;
        this.dataExchangeType = dataExchangeType;
        this.bufferSize = bufferSize;
        this.dropNew = dropNew;
        this.isLatched = isLatched;
    }
}
}