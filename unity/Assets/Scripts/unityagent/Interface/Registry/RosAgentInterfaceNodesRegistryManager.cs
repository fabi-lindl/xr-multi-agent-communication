using System;
using System.Collections.Generic;
using System.Threading.Tasks;

using Unity.Robotics.ROSTCPConnector.MessageGeneration;

using UnityAgent.Interface.Registry;
using UnityAgent.Interface.RosAgent;

namespace UnityAgent.Interface.Registry
{
public class RosAgentInterfaceNodesRegistryManager : InterfaceNodesRegistryManager
{
    private class Nested
    {
        static Nested()
        {
        }

        internal static readonly RosAgentInterfaceNodesRegistryManager instance =
            new();
    }

    private readonly RosAgentInterfaceNodesRegistry registry =
        RosAgentInterfaceNodesRegistry.Instance;

    private RosAgentInterfaceNodesRegistryManager()
    {
    }

    public static RosAgentInterfaceNodesRegistryManager Instance =>
                                                        Nested.instance;

    public RosAgentInterfaceSubscriberNode RegisterInterfaceSubscriberNode<T>(
        Action<T, ushort> subscriber,
        string topicName,
        List<ushort> agentIds=null,
        List<byte> groupIds=null,
        uint rosAgentBufSize=10,
        bool dropNew=false)
    where T : Message
    {
        Action<Message, ushort> callback = (Message msg, ushort agentId) => {
            subscriber((T)msg, agentId);
        };
        string dataExchangeType = MessageRegistry.GetRosMessageName<T>();
        RosAgentInterfaceSubscriberNode node = new(callback,
                                                   topicName,
                                                   dataExchangeType,
                                                   rosAgentBufSize,
                                                   dropNew);
        return (RosAgentInterfaceSubscriberNode)Register(node, agentIds, groupIds);
    }

    public RosAgentInterfacePublisherNode RegisterInterfacePublisherNode<T>(
        string topicName,
        List<ushort> agentIds=null,
        List<byte> groupIds=null,
        uint unityAgentBufSize=10,
        uint rosAgentBufSize=10,
        bool dropNew=false,
        bool isLatched=false)
    where T : Message
    {
        string dataExchangeType = MessageRegistry.GetRosMessageName<T>();
        RosAgentInterfacePublisherNode node = new(topicName,
                                                  dataExchangeType,
                                                  unityAgentBufSize,
                                                  rosAgentBufSize,
                                                  dropNew,
                                                  isLatched);
        return (RosAgentInterfacePublisherNode)Register(node, agentIds, groupIds);
    }

    public RosAgentInterfaceServiceNode RegisterInterfaceServiceNode<TRequest, TResponse>(
        Func<TRequest, TResponse> service,
        string serviceName,
        List<ushort> agentIds=null,
        List<byte> groupIds=null,
        uint unityAgentBufSize=10,
        uint rosAgentBufSize=10,
        bool dropNew=true)
    where TRequest : Message where TResponse : Message
    {
        Func<Message, Message> callback = (Message msg) => {
            return service((TRequest)msg);
        };
        string dataExchangeType = MessageRegistry.GetRosMessageName<TRequest>();
        RosAgentInterfaceServiceNode node = new(callback,
                                                serviceName,
                                                dataExchangeType,
                                                unityAgentBufSize,
                                                rosAgentBufSize,
                                                dropNew);
        return (RosAgentInterfaceServiceNode)Register(node, agentIds, groupIds);
    }

    public RosAgentInterfaceServiceNode RegisterAsyncInterfaceServiceNode<TRequest, TResponse>(
        Func<TRequest, TResponse> asyncService,
        string serviceName,
        List<ushort> agentIds=null,
        List<byte> groupIds=null,
        uint unityAgentBufSize=10,
        uint rosAgentBufSize=10,
        bool dropNew=true)
    where TRequest : Message where TResponse : Task<Message>
    {
        Func<Message, Task<Message>> callback = (Message msg) => {
            return asyncService((TRequest)msg);
        };
        string dataExchangeType = MessageRegistry.GetRosMessageName<TRequest>();
        RosAgentInterfaceServiceNode node = new(callback,
                                                serviceName,
                                                dataExchangeType,
                                                unityAgentBufSize,
                                                rosAgentBufSize,
                                                dropNew);
        return (RosAgentInterfaceServiceNode)Register(node, agentIds, groupIds);
    }

    public RosAgentInterfaceClientNode RegisterInterfaceClientNode<TRequest>(
        string tosName,
        List<ushort> agentIds=null,
        List<byte> groupIds=null,
        uint unityAgentBufSize=10,
        uint rosAgentBufSize=10,
        bool dropNew=true)
    where TRequest : Message
    {
        string dataExchangeType = MessageRegistry.GetRosMessageName<TRequest>();
        RosAgentInterfaceClientNode node = new(tosName,
                                               dataExchangeType,
                                               unityAgentBufSize,
                                               rosAgentBufSize,
                                               dropNew);
        return (RosAgentInterfaceClientNode)Register(node, agentIds, groupIds);
    }

    public void DeregisterInterfaceNode(RosAgentInterfaceNode node,
                                        bool sendDeregistrationMessage=true)
    {
        bool isSuccess = registry.Remove(node);
        if (isSuccess && sendDeregistrationMessage)
        {
            node.DeregisterAllAgents();
            node.DeregisterAllGroups();
        }
    }

    private RosAgentInterfaceNode Register(
        RosAgentInterfaceNode node, List<ushort> agentIds, List<byte> groupIds)
    {
        if (node == null) return null;
        if (registry.Has(node))
            return RegisterAgentsOnExistingNode(node, agentIds, groupIds);
        return StoreNewNodeToRegistry(node, agentIds, groupIds);
    }

    private RosAgentInterfaceNode RegisterAgentsOnExistingNode(
        RosAgentInterfaceNode node, List<ushort> agentIds, List<byte> groupIds)
    {
        RosAgentInterfaceNode existingNode =
            registry.Get<RosAgentInterfaceNode>(node.NodeType, node.NodeName);
        if (existingNode.RegisterAgentsAndGroups(agentIds, groupIds))
            return existingNode;
        return null;
    }

    private RosAgentInterfaceNode StoreNewNodeToRegistry(
        RosAgentInterfaceNode node, List<ushort> agentIds, List<byte> groupIds)
    {
        if (!registry.Add(node)) return null;
        if (node.RegisterAgentsAndGroups(agentIds, groupIds)) return node;
        registry.Remove(node);
        return null;
    }
}
}