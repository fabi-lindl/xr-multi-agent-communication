using System;
using System.Collections.Generic;
using System.Linq;

using Unity.Robotics.ROSTCPConnector.MessageGeneration;

using UnityAgent.SystemMessage;

namespace UnityAgent.Interface.RosAgent
{
public class RosAgentInterfacePublisherNode : RosAgentInterfaceSenderNode
{
    private static byte nodeType = 2;
    private readonly RoutingData routingData;
    private readonly RoutingInfo routingInfo;
    private readonly SystemMessageMetadata metadata;

    public RosAgentInterfacePublisherNode(string tosName,
                                          string dataExchangeType,
                                          uint unityAgentBufSize,
                                          uint rosAgentBufSize,
                                          bool dropNew,
                                          bool isLatched)
    : base(nodeType, tosName, dataExchangeType, unityAgentBufSize,
           rosAgentBufSize, dropNew, isLatched)
    {
        routingData = new(DestAgentType, agentIds, groupIds);
        routingInfo = new(routingData);
        metadata = new(NodeType, NodeName, dataExchangeType);
    }

    public override bool RegisterAgentsAndGroups(List<ushort> newAgentIds,
                                                 List<byte> newGroupIds)
    {
        if (newAgentIds == null)
            return RegisterGroups(newGroupIds);
        else if (newGroupIds == null)
            return RegisterAgents(newAgentIds);
        else
        {
            if (!CanRegisterAgents(newAgentIds) ||
                !CanRegisterGroups(newGroupIds))
                return false;
            List<ushort> agentIdsToAdd = AgentDifference(newAgentIds);
            agentIds.AddRange(agentIdsToAdd);
            List<byte> groupIdsToAdd = GroupDifference(newGroupIds);
            groupIds.AddRange(groupIdsToAdd);
            registrar.SendRegistrationMessage(agentIdsToAdd, groupIdsToAdd);
            return true;
        }
    }

    public override bool RegisterAgent(ushort newAgentId)
    {
        List<ushort> newAgentIds = new List<ushort> { newAgentId };
        return RegisterAgents(newAgentIds);
    }

    public override bool RegisterAgents(List<ushort> newAgentIds)
    {
        if (!CanRegisterAgents(newAgentIds))
            return false;
        List<ushort> idsToAdd = AgentDifference(newAgentIds);
        agentIds.AddRange(idsToAdd);
        registrar.SendRegistrationMessage(idsToAdd);
        return true;
    }

    private bool CanRegisterAgents(List<ushort> newAgentIds)
    {
        List<ushort> idsToAdd = AgentDifference(newAgentIds);
        int currentLength = agentIds.Count;
        int deltaLength = idsToAdd.Count;
        int newLength = currentLength + deltaLength;
        return !RoutingData.ExceedsMaxNumDestAgents(newLength);
    }

    public override bool RegisterGroup(byte newGroupId)
    {
        List<byte> newGroupIds = new List<byte> { newGroupId };
        return RegisterGroups(newGroupIds);
    }

    public override bool RegisterGroups(List<byte> newGroupIds)
    {
        if (!CanRegisterGroups(newGroupIds))
            return false;
        List<byte> idsToAdd = GroupDifference(newGroupIds);
        groupIds.AddRange(idsToAdd);
        registrar.SendRegistrationMessage(idsToAdd);
        return true;
    }

    private bool CanRegisterGroups(List<byte> newGroupIds)
    {
        List<byte> idsToAdd = GroupDifference(newGroupIds);
        int currentLength = groupIds.Count;
        int deltaLength = idsToAdd.Count;
        int newLength = currentLength + deltaLength;
        return !RoutingData.ExceedsMaxNumDestGroups(newLength);
    }

    public void Publish(Message message)
    {
        byte[] data = SerializeMessageData(message);
        EgressSystemMessage esm = new(routingInfo, metadata, data);
        SendEsm(esm);
    }
}
}