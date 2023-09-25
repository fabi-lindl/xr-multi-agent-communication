using System.Collections.Generic;

using UnityAgent.Interface;
using UnityAgent.Interface.RosAgent;

namespace UnityAgent.Interface.Registry
{
public abstract class InterfaceNodesRegistry
{
    public abstract bool Has<T>(T node) where T : RosAgentInterfaceNode;

    public abstract bool Has(string nodeName);

    public abstract T Get<T>(byte nodeType, string nodeName) where T: AgentInterfaceNode;

    public abstract void Add<T>(List<T> nodes) where T : RosAgentInterfaceNode;

    public abstract bool Add<T>(T node) where T : RosAgentInterfaceNode;

    public abstract void Remove<T>(List<T> nodes)
        where T : RosAgentInterfaceNode;

    public abstract bool Remove<T>(T node) where T : RosAgentInterfaceNode;

    public abstract void Clear();
}
}