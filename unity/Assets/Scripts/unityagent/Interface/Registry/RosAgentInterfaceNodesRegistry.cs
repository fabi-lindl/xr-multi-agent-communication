using System;
using System.Collections.Generic;

using UnityAgent.Interface.RosAgent;
using UnityAgent.Operation;

namespace UnityAgent.Interface.Registry
{
public class RosAgentInterfaceNodesRegistry : InterfaceNodesRegistry
{
    private class Nested
    {
        static Nested()
        {
        }

        internal static readonly RosAgentInterfaceNodesRegistry instance = new();
    }

    private Dictionary<byte, Dictionary<string, RosAgentInterfaceNode>> registry =
        new()
    {
        { 1, new Dictionary<string, RosAgentInterfaceNode>() },
        { 2, new Dictionary<string, RosAgentInterfaceNode>() },
        { 3, new Dictionary<string, RosAgentInterfaceNode>() },
        { 4, new Dictionary<string, RosAgentInterfaceNode>() }
    };

    private RosAgentInterfaceNodesRegistry()
    {
    }

    public static RosAgentInterfaceNodesRegistry Instance => Nested.instance;

    public override bool Has<AgentInterfaceNode>(AgentInterfaceNode node)
    {
        var nodeTypeDict = registry[node.NodeType];
        return nodeTypeDict.ContainsKey(node.NodeName);
    }

    public override bool Has(string nodeName)
    {
        foreach (var nodeTypeDict in registry.Values)
            if (nodeTypeDict.ContainsKey(nodeName))
                return true;
        return false;
    }

    public override T Get<T>(byte nodeType, string nodeName)
    {
        return (T)(object)registry[nodeType][nodeName];
    }

    public RosAgentInterfaceNode Get(byte nodeType, string nodeName)
    {
        if (Has(nodeName))
            return registry[nodeType][nodeName];
        return null;
    }

    public override void Add<RosAgentInterfaceNode>(
        List<RosAgentInterfaceNode> nodes)
    {
        foreach (RosAgentInterfaceNode node in nodes)
            Add(node);
    }

    public override bool Add<RosAgentInterfaceNode>(RosAgentInterfaceNode node)
    {
        try
        {
            var nodeTypeDict = registry[node.NodeType];
            nodeTypeDict.Add(node.NodeName, node);
            return true;
        }
        catch
        {
            return false;
        }
    }

    public override void Remove<RosAgentInterfaceNode>(
        List<RosAgentInterfaceNode> nodes)
    {
        foreach (RosAgentInterfaceNode node in nodes)
            Remove(node);
    }

    public override bool Remove<RosAgentInterfaceNode>(
        RosAgentInterfaceNode node)
    {
        var nodeTypeDict = registry[node.NodeType];
        return nodeTypeDict.Remove(node.NodeName);
    }

    public override void Clear()
    {
        foreach (var nodesTypeDict in registry.Values)
            nodesTypeDict.Clear();
    }
}
}