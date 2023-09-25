using System.Collections.Generic;

using UnityAgent.SystemMessage;
using UnityAgent.Interface;
using UnityAgent.Interface.Registry;

namespace UnityAgent.Operation
{
public static class InterfaceNodeMessageProcessor
{
    private static readonly Dictionary<byte, InterfaceNodesRegistry> registries =
    new()
    {
        { 1, RosAgentInterfaceNodesRegistry.Instance }
    };

    public static void ProcessIsm(IngressSystemMessage ism)
    {
        byte agentType = ism.SenderAgentType;
        byte nodeType = (byte)ism.NodeType;
        string nodeName = ism.NodeName;
        AgentInterfaceNode node = GetNode(agentType, nodeType, nodeName);
        node?.ProcessIsm(ism);
    }

    private static bool HasNode(byte agentType, byte nodeType, string nodeName)
    {
        if (!registries.ContainsKey(agentType)) return false;
        InterfaceNodesRegistry registry = registries[agentType];
        return registry.Has(nodeName);
    }

    private static AgentInterfaceNode GetNode(byte agentType, byte nodeType,
                                              string nodeName)
    {
        if (!registries.ContainsKey(agentType)) return null;
        InterfaceNodesRegistry registry = registries[agentType];
        if (registry.Has(nodeName))
            return registry.Get<AgentInterfaceNode>(nodeType, nodeName);
        return null;
    }
}
}