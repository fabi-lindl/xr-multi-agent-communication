using System;
using System.Collections.Generic;
using System.Linq;

using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace UnityAgent.Interface.RosAgent
{
public abstract class RosAgentInterfaceNode : AgentInterfaceNode
{
    private readonly byte destAgentType = 1; // RosAgent
    private string nodeName;
    protected List<ushort> agentIds = new();
    protected List<byte> groupIds = new();
    protected RosAgentNodeRegistrationMessageSender registrar;
    protected MessageSerializer messageSerializer = new();
    protected MessageDeserializer messageDeserializer = new();
    protected Func<MessageDeserializer, Message> deserializer;

    public RosAgentInterfaceNode(
        byte nodeType,
        string tosName,
        string dataExchangeType,
        uint rosAgentBufSize,
        bool dropNew,
        bool isLatched=false) // Publisher only
    {
        NodeType = nodeType;
        NodeName = tosName;
        TosName = tosName;
        DataExchangeType = dataExchangeType;
        registrar = new(nodeType, NodeName, tosName, dataExchangeType,
                        rosAgentBufSize, dropNew, isLatched);
        deserializer = MessageRegistry.GetDeserializeFunction(
            dataExchangeType, MessageSubtopic.Default);
    }

    public byte DestAgentType => destAgentType;

    public byte NodeType { get; private set; }
    public string NodeName
    {
        get => nodeName;
        private set
        {
            nodeName = TosToNodeName(value);
        }
    }
    public string TosName { get; private set; }
    public string DataExchangeType { get; private set; }
    
    public static string TosToNodeName(string tosName)
    {
        return "nd_" + tosName.Replace("/", "_").Replace("~", "_");
    }

    public bool HasAgent(ushort agentId)
    {
        return agentIds.Contains(agentId);
    }

    public bool HasGroup(byte serviceGroupId)
    {
        return groupIds.Contains(serviceGroupId);
    }

    public virtual bool RegisterAgentsAndGroups(List<ushort> newAgentIds,
                                                List<byte> newGroupIds)
    {
        if (newAgentIds == null)
            return RegisterGroups(newGroupIds);
        else if (newGroupIds == null)
            return RegisterAgents(newAgentIds);
        else
        {
            List<ushort> agentIdsToAdd = AgentDifference(newAgentIds);
            agentIds.AddRange(agentIdsToAdd);
            List<byte> groupIdsToAdd = GroupDifference(newGroupIds);
            groupIds.AddRange(groupIdsToAdd);
            registrar.SendRegistrationMessage(agentIdsToAdd, groupIdsToAdd);
            return true;
        }
    }
    
    public virtual bool RegisterAgent(ushort agentId)
    {
        List<ushort> agentIds = new List<ushort> { agentId };
        return RegisterAgents(agentIds);
    }

    public virtual bool RegisterAgents(List<ushort> newAgentIds)
    {
        if (newAgentIds == null) return false;
        List<ushort> idsToAdd = AgentDifference(newAgentIds);
        agentIds.AddRange(idsToAdd);
        registrar.SendRegistrationMessage(idsToAdd);
        return true;
    }

    public virtual bool RegisterGroup(byte groupId)
    {
        List<byte> groupIds = new List<byte> { groupId };
        return RegisterGroups(groupIds);
    }

    public virtual bool RegisterGroups(List<byte> newGroupIds)
    {
        if (newGroupIds == null)
            return false;
        List<byte> idsToAdd = GroupDifference(newGroupIds);
        groupIds.AddRange(idsToAdd);
        registrar.SendRegistrationMessage(idsToAdd);
        return true;
    }

    public void DeregisterAgent(ushort agentId)
    {
        if (HasAgent(agentId))
        {
            agentIds.Remove(agentId);
            registrar.SendDeregistrationMessage(agentId);
        }
    }

    public void DeregisterAgents(List<ushort> agentIds)
    {
        if (agentIds == null)
            return;
        List<ushort> idsToRemove = AgentIntersection(agentIds);
        foreach (ushort id in idsToRemove)
            agentIds.Remove(id);
        registrar.SendDeregistrationMessage(idsToRemove);
    }

    public void DeregisterAllAgents()
    {
        List<ushort> idsToRemove = agentIds;
        agentIds = new();
        registrar.SendDeregistrationMessage(idsToRemove);
    }

    public void DeregisterGroup(byte groupId)
    {
        if (HasGroup(groupId))
        {
            groupIds.Remove(groupId);
            registrar.SendDeregistrationMessage(groupId);
        }
    }

    public void DeregisterGroups(List<byte> groupIds)
    {
        if (groupIds == null)
            return;
        List<byte> idsToRemove = GroupIntersection(groupIds);
        foreach (byte id in idsToRemove)
            groupIds.Remove(id);
        registrar.SendDeregistrationMessage(idsToRemove);
    }

    public void DeregisterAllGroups()
    {
        List<byte> idsToRemove = groupIds;
        groupIds = new();
        registrar.SendDeregistrationMessage(idsToRemove);
    }

    protected Message DeserializeMessageData(byte[] data)
    {
        messageDeserializer.InitWithBuffer(data);
        Message message = deserializer(messageDeserializer);
        return message;
    }

    protected List<ushort> AgentDifference(List<ushort> agentIds)
    {
        return agentIds.Where(id => !HasAgent(id)).ToList();
    }

    protected List<ushort> AgentIntersection(List<ushort> agentIds)
    {
        return agentIds.Where(id => HasAgent(id)).ToList();
    }

    protected List<byte> GroupDifference(List<byte> groupIds)
    {
        return groupIds.Where(id => !HasAgent(id)).ToList();
    }

    protected List<byte> GroupIntersection(List<byte> groupIds)
    {
        return groupIds.Where(id => HasAgent(id)).ToList();
    }
}
}