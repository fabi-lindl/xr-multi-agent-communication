using System.Collections.Generic;
using System.Threading.Tasks;

using UnityAgent.Buffers;
using UnityAgent.Config;
using UnityAgent.Management.Message;
using UnityAgent.SystemMessage;
using UnityAgent.Util.Threading;
using UnityAgent.Util.Networking;

namespace UnityAgent.Operation
{
public static class Communicator
{
    private static readonly object RequestIdLockObject = new();
    private static uint nextRequestId = 0;
    private static readonly Dictionary<uint, PendingRequest> pendingRequests =
        new();

    public static ResponseWaiter GetResponseWaiter(uint requestId)
    {
        PendingRequest pendingRequest = GetPendingRequest(requestId);
        if (PendingRequestExists(pendingRequest))
            return pendingRequest.ResponseWaiter;
        return null;
    }

    private static PendingRequest GetPendingRequest(uint requestId)
    {
        PendingRequest pendingRequest;
        pendingRequests.TryGetValue(requestId, out pendingRequest);
        return pendingRequest;
    }

    private static bool PendingRequestExists(PendingRequest request)
    {
        return request != null;
    }

    public static void SendMessageToRouter(RoutingManagementData rmd)
    {
        byte[] data = new byte[0];
        SendMessageToRouter(rmd, data);
    }

    public static void SendMessageToRouter(RoutingManagementData rmd,
                                           byte[] data)
    {
        EgressSystemMessage esm = CreateRouterEsm(rmd, data);
        SendMessage(esm);
    }

    public static async Task<IngressSystemMessage> SendRequestToRouter(
        RoutingManagementData rmd)
    {
        byte[] data = new byte[0];
        IngressSystemMessage response = await SendRequestToRouter(rmd, data);
        return response;
    }

    public static async Task<IngressSystemMessage> SendRequestToRouter(
        RoutingManagementData rmd, byte[] data)
    {
        EgressSystemMessage esm = CreateRouterEsm(rmd, data);
        IngressSystemMessage response = await ManageRequest(esm);
        return response;
    }

    public static async Task<IngressSystemMessage> SendRequestToAgent(
        byte agentType, ushort agentId, byte[] data, string messageManager)
    {
        RoutingData routingData = new(agentType, agentId);
        EgressSystemMessage esm = CreateEsm(routingData, data, messageManager,
                                            isRequest: true);
        IngressSystemMessage response = await ManageRequest(esm);
        return response;
    }

    public static void SendMessageToAgent(
        byte agentType, ushort agentId, byte[] data, string messageManager)
    {
        RoutingData routingData = new(agentType, agentId);
        EgressSystemMessage esm = CreateEsm(routingData, data, messageManager);
        SendMessage(esm);
    }

    public static void SendMessageToAgents(
        byte agentType, List<ushort> agentIds, byte[] data, string messageManager)
    {
        RoutingData routingData = new(agentType, agentIds);
        EgressSystemMessage esm = CreateEsm(routingData, data, messageManager);
        SendMessage(esm);
    }

    public static void SendMessageToGroup(
        byte agentType, byte groupId, byte[] data, string messageManager)
    {
        RoutingData routingData = new(agentType, groupId);
        EgressSystemMessage esm = CreateEsm(routingData, data, messageManager);
        SendMessage(esm);
    }

    public static void SendMessageToGroups(
        byte agentType, List<byte> groupIds, byte[] data, string messageManager)
    {
        RoutingData routingData = new(agentType, groupIds);
        EgressSystemMessage esm = CreateEsm(routingData, data, messageManager);
        SendMessage(esm);
    }

    private static void SendMessage(EgressSystemMessage esm)
    {
        EgressSystemMessageBuffer esmBuffer = new();
        esmBuffer.Enqueue(esm);
        EgressBuffer.Enqueue(esmBuffer);
    }

    private static async Task<IngressSystemMessage> ManageRequest(
        EgressSystemMessage esm)
    {
        SystemMessageMetadata metadata = esm.Metadata;
        uint requestId = metadata.RequestId;
        ResponseWaiter rw = new();
        CreatePendingRequest(requestId, rw);
        SendMessage(esm);
        await rw.Wait();
        IngressSystemMessage ism = (IngressSystemMessage)rw.Result;
        DeletePendingRequest(requestId);
        return ism;
    }

    private static EgressSystemMessage CreateRouterEsm(
        RoutingManagementData rmd, byte[] data)
    {
        RoutingData routingData = new();
        RoutingInfo routingInfo = new(routingData, rmd);
        SystemMessageMetadata metadata = new(GetRequestId());
        EgressSystemMessage esm = new(routingInfo, metadata, data);
        return esm;
    }

    private static EgressSystemMessage CreateEsm(
        RoutingData routingData, byte[] data, string messageManager="",
        bool isRequest=false)
    {
        RoutingInfo routingInfo = new(routingData);
        SystemMessageMetadata metadata = CreateMetadata(messageManager,
                                                        isRequest);
        EgressSystemMessage esm = new(routingInfo, metadata, data);
        return esm;
    }

    private static SystemMessageMetadata CreateMetadata(
        string messageManager, bool isRequest=false)
    {
        if (isRequest)
            return new SystemMessageMetadata(messageManager, GetRequestId());
        return new SystemMessageMetadata(messageManager);
    }

    private static void CreatePendingRequest(uint requestId, ResponseWaiter rw)
    {
        PendingRequest pendingRequest = new(requestId, rw);
        pendingRequests.Add(requestId, pendingRequest);
    }

    private static void DeletePendingRequest(uint requestId)
    {
        pendingRequests.Remove(requestId);
    }

    private static uint GetRequestId()
    {
        lock(RequestIdLockObject)
        {
            uint requestId = nextRequestId;
            nextRequestId++;
            return requestId;
        }
    }
}
}