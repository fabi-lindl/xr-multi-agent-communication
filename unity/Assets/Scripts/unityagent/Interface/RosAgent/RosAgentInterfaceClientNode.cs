using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;

using Unity.Robotics.ROSTCPConnector.MessageGeneration;

using UnityAgent.SystemMessage;
using UnityAgent.Util.Networking;
using UnityAgent.Util.Threading;
using UnityAgent.Util.Logging;

namespace UnityAgent.Interface.RosAgent
{
public class RosAgentInterfaceClientNode : RosAgentInterfaceSenderNode
{
    private static byte nodeType = 3;
    private readonly Dictionary<uint, PendingRequest> pendingRequests = new();
    private readonly object requestIdLock = new();
    private uint nextRequestId = 0;

    public RosAgentInterfaceClientNode(string tosName,
                                       string dataExchangeType,
                                       uint unityAgentBufSize,
                                       uint rosAgentBufSize,
                                       bool dropNew)
    : base(nodeType, tosName, dataExchangeType, unityAgentBufSize,
           rosAgentBufSize, dropNew)
    {
    }

    public async void MakeRequest<TResponse>(Message message, ushort destAgentId,
                                             Action<TResponse> callback)
    where TResponse : Message
    {
        TResponse response = await MakeRequest<TResponse>(message, destAgentId);
        if (response == null) return;
        try
        {
            callback(response);
        }
        catch
        {
            AgentLogger.LogErrorInResponseCallbackFunction();
        }
    }

    public async Task<TResponse> MakeRequest<TResponse>(Message message,
                                                        ushort destAgentId)
    where TResponse : Message
    {
        PendingRequest pr = CreatePendingRequest();
        EgressSystemMessage esm = CreateEsm(message, destAgentId, pr.RequestId);
        SendEsm(esm);
        TResponse responseMessage = await WaitForResponse<TResponse>(pr);
        RemovePendingRequest(pr.RequestId);
        return responseMessage;
    }

    private PendingRequest CreatePendingRequest()
    {
        uint requestId = RequestId();
        ResponseWaiter rw = new();
        PendingRequest pr = new(requestId, rw);
        pendingRequests.Add(requestId, pr);
        return pr;
    }

    private void RemovePendingRequest(uint requestId)
    {
        pendingRequests.Remove(requestId);
    }

    private uint RequestId()
    {
        lock(requestIdLock)
        {
            uint requestId = nextRequestId;
            nextRequestId++;
            return requestId;
        }
    }

    private EgressSystemMessage CreateEsm(Message message, ushort destAgentId,
                                          uint requestId)
    {
        RoutingData routingData = new(DestAgentType, destAgentId);
        RoutingInfo routingInfo = new(routingData);
        SystemMessageMetadata metadata = new(nodeType, NodeName,
                                             DataExchangeType, requestId);
        byte[] data = SerializeMessageData(message);
        EgressSystemMessage esm = new(routingInfo, metadata, data);
        return esm;
    }

    private async Task<TResponse> WaitForResponse<TResponse>(PendingRequest pr)
    where TResponse : Message
    {
        ResponseWaiter rw = pr.ResponseWaiter;
        await rw.Wait();
        byte[] responseData = (byte[])rw.Result;
        TResponse message = messageDeserializer
                           .DeserializeMessage<TResponse>(responseData);
        pendingRequests.Remove(pr.RequestId);
        return message;
    }

    public override void ProcessIsm(IngressSystemMessage ism)
    {
        PendingRequest request = GetPendingRequest(ism.RequestId);
        if (!PendingRequestExists(request)) return;
        pendingRequests.Remove(ism.RequestId);
        ResponseWaiter rw = request.ResponseWaiter;
        byte[] responseData = ism.Data;
        rw.Resume(responseData);
    }

    private PendingRequest GetPendingRequest(uint requestId)
    {
        PendingRequest request;
        pendingRequests.TryGetValue(requestId, out request);
        return request;
    }

    private bool PendingRequestExists(PendingRequest request)
    {
        return request != null;
    }
}
}