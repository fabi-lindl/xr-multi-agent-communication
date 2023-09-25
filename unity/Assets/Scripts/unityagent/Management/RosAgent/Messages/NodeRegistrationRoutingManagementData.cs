using System.Collections.Generic;

using UnityAgent.Operation;
using UnityAgent.Management.Message;

namespace UnityAgent.Management.RosAgent.Message
{
public class NodeRegistrationRoutingManagementData : RoutingManagementData
{
    public string messageManager;
    public ushort registrantAgentId = Agent.Instance.AgentId;
    public List<ushort> registrarAgentIds;
    public List<byte> registrarGroupIds;
    public byte nodeType;
    public string nodeName;
    public string tosName;
    public string dataExchangeType;

    public NodeRegistrationRoutingManagementData(string messageManager,
                                                 List<ushort> registrarAgentIds,
                                                 List<byte> registrarGroupIds,
                                                 byte nodeType,
                                                 string nodeName,
                                                 string tosName,
                                                 string dataExchangeType)
    {
        this.messageManager = messageManager;
        this.registrarAgentIds = registrarAgentIds;
        this.registrarGroupIds = registrarGroupIds;
        this.nodeType = nodeType;
        this.nodeName = nodeName;
        this.tosName = tosName;
        this.dataExchangeType = dataExchangeType;
    }

    public override string MessageManager => messageManager;
}
}