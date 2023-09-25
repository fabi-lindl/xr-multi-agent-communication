using System;
using System.Threading.Tasks;

using Unity.Robotics.ROSTCPConnector.MessageGeneration;

using UnityAgent.SystemMessage;

namespace UnityAgent.Interface.RosAgent
{
public class RosAgentInterfaceServiceNode : RosAgentInterfaceSenderNode
{
    private static byte nodeType = 4;
    private readonly Func<Message, Message> service;
    private readonly Func<Message, Task<Message>> asyncService;

    public RosAgentInterfaceServiceNode(Func<Message, Message> service,
                                        string tosName,
                                        string dataExchangeType,
                                        uint unityAgentBufSize,
                                        uint rosAgentBufSize,
                                        bool dropNew)
    : base(nodeType, tosName, dataExchangeType, unityAgentBufSize,
           rosAgentBufSize, dropNew)
    {
        this.service = service;
    }

    public RosAgentInterfaceServiceNode(Func<Message, Task<Message>> service,
                                        string tosName,
                                        string dataExchangeType,
                                        uint unityAgentBufSize,
                                        uint rosAgentBufSize,
                                        bool dropNew)
    : base(nodeType, tosName, dataExchangeType, unityAgentBufSize,
           rosAgentBufSize, dropNew)
    {
        asyncService = service;
    }

    public async override void ProcessIsm(IngressSystemMessage ism)
    {
        RoutingInfo routingInfo = ResponseRoutingInfo(ism);
        SystemMessageMetadata metadata = ResponseMetadata(ism);
        byte[] data = await ResponseData(ism);
        EgressSystemMessage esm = new(routingInfo, metadata, data);
        SendEsm(esm);
    }

    private RoutingInfo ResponseRoutingInfo(IngressSystemMessage ism)
    {
        byte destAgentType = ism.SenderAgentType;
        ushort destAgentId = ism.SenderAgentId;
        RoutingData routingData = new(destAgentType, destAgentId);
        RoutingInfo routingInfo = new(routingData);
        return routingInfo;
    }

    private SystemMessageMetadata ResponseMetadata(IngressSystemMessage ism)
    {
        uint requestId = (uint)ism.RequestId;
        SystemMessageMetadata metadata = new(nodeType, NodeName,
                                             DataExchangeType, requestId);
        return metadata;
    }

    private async Task<byte[]> ResponseData(IngressSystemMessage ism)
    {
        byte[] requestData = ism.Data;
        Message requestMessage = DeserializeMessageData(requestData);
        Message responseMessage = await ProcessRequestMessage(requestMessage);
        byte[] data = SerializeMessageData(responseMessage);
        return data;
    }

    private async Task<Message> ProcessRequestMessage(Message requestMessage)
    {
        if (service != null)
            return service(requestMessage);
        return await asyncService(requestMessage);
    }
}
}