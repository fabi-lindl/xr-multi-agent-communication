using System;
using System.Collections.Generic;

using UnityAgent.Buffers;
using UnityAgent.Management.Message;
using UnityAgent.Management.RosAgent.Message;
using UnityAgent.SystemMessage;
using UnityAgent.Util.Serialization;

namespace UnityAgent.Interface.RosAgent
{
public class RosAgentNodeRegistrationMessageSender
{
    private readonly byte nodeType;
    private readonly string nodeName;
    private readonly string tosName;
    private readonly string dataExchangeType;

    private static string routerMessageManager = "RosAgentNodeRegistrationForwarder";
    private static string regMessageManager = "InterfaceNodeAdder";
    private static string deregMessageManager = "InterfaceNodeRemover";
    private readonly SystemMessageMetadata regMetadata = new(regMessageManager);
    private readonly SystemMessageMetadata deregMetadata = new(deregMessageManager);
    
    private readonly byte[] regData;
    private readonly byte[] deregData;

    public RosAgentNodeRegistrationMessageSender(
        byte nodeType,
        string nodeName,
        string tosName,
        string dataExchangeType,
        uint rosAgentBufSize,
        bool dropNew,
        bool isLatched)
    {
        this.nodeType = nodeType;
        this.nodeName = nodeName;
        this.tosName = tosName;
        this.dataExchangeType = dataExchangeType;
        NodeRegistrationMessage regMessage = new(nodeType,
                                                 nodeName,
                                                 tosName,
                                                 dataExchangeType,
                                                 rosAgentBufSize,
                                                 dropNew,
                                                 isLatched);
        regData = NetworkDataSerializer.ObjToBytes(regMessage);
        NodeDeregistrationMessage deregMessage = new(nodeType, nodeName);
        deregData = NetworkDataSerializer.ObjToBytes(deregMessage);
    }

    public void SendRegistrationMessage(ushort agentId)
    {
        List<ushort> ids = new List<ushort> { agentId };
        SendRegistrationMessage(ids);
    }

    public void SendRegistrationMessage(List<ushort> agentIds)
    {
        SendRegistrationMessage(CreateRoutingInfo(agentIds, EmptyGroupList()));
    }

    public void SendRegistrationMessage(byte groupId)
    {
        List<byte> ids = new List<byte> { groupId };
        SendRegistrationMessage(ids);
    }

    public void SendRegistrationMessage(List<byte> groupIds)
    {
        SendRegistrationMessage(CreateRoutingInfo(EmptyAgentList(), groupIds));
    }

    public void SendRegistrationMessage(List<ushort> agentIds,
                                        List<byte> groupIds)
    {
        SendRegistrationMessage(CreateRoutingInfo(agentIds, groupIds));
    }

    private void SendRegistrationMessage(RoutingInfo routingInfo)
    {
        EgressSystemMessage esm = new(routingInfo, regMetadata, regData);
        SendEsm(esm);
    }

    public void SendDeregistrationMessage(ushort agentId)
    {
        List<ushort> ids = new List<ushort> { agentId };
        SendDeregistrationMessage(ids);
    }

    public void SendDeregistrationMessage(List<ushort> agentIds)
    {
        SendDeregistrationMessage(CreateRoutingInfo(agentIds, EmptyGroupList()));
    }

    public void SendDeregistrationMessage(byte groupId)
    {
        List<ushort> ids = new List<ushort> { groupId };
        SendDeregistrationMessage(ids);
    }

    public void SendDeregistrationMessage(List<byte> groupIds)
    {
        SendDeregistrationMessage(CreateRoutingInfo(EmptyAgentList(), groupIds));
    }

    public void SendDeregistrationMessage(List<ushort> agentIds,
                                          List<byte> groupIds)
    {
        SendDeregistrationMessage(CreateRoutingInfo(agentIds, groupIds));
    }

    private void SendDeregistrationMessage(RoutingInfo routingInfo)
    {
        EgressSystemMessage esm = new(routingInfo, deregMetadata, deregData);
        SendEsm(esm);
    }

    private void SendEsm(EgressSystemMessage esm)
    {
        EgressSystemMessageBuffer esmBuffer = new();
        esmBuffer.Enqueue(esm);
        EgressBuffer.Enqueue(esmBuffer);
    }

    private RoutingInfo CreateRoutingInfo(List<ushort> agentIds,
                                          List<byte> groupIds)
    {
        RoutingData routingData = new();
        NodeRegistrationRoutingManagementData managementData = new(
            routerMessageManager,
            agentIds,
            groupIds,
            nodeType,
            nodeName,
            tosName,
            dataExchangeType
        );
        RoutingInfo routingInfo = new(routingData, (RoutingManagementData)managementData);
        return routingInfo;
    }

    private List<ushort> EmptyAgentList()
    {
        return new List<ushort>();
    }

    private List<byte> EmptyGroupList()
    {
        return new List<byte>();
    }
}
}