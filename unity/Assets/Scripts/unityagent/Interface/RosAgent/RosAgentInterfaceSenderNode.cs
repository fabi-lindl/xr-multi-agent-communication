using System.Collections.Generic;

using Unity.Robotics.ROSTCPConnector.MessageGeneration;

using UnityAgent.Buffers;
using UnityAgent.SystemMessage;

namespace UnityAgent.Interface.RosAgent
{
public abstract class RosAgentInterfaceSenderNode : RosAgentInterfaceNode
{
    private EgressSystemMessageBuffer esmBuffer;

    public RosAgentInterfaceSenderNode(byte nodeType,
                                       string tosName,
                                       string dataExchangeType,
                                       uint unityAgentBufSize,
                                       uint rosAgentBufSize,
                                       bool dropNew,
                                       bool isLatched=false) // Publisher only
    : base(nodeType, tosName, dataExchangeType, rosAgentBufSize, dropNew,
           isLatched)
    {
        BufferSize = unityAgentBufSize;
        DropNew = dropNew;
        esmBuffer = new(BufferSize, DropNew);
    }

    public uint BufferSize { get; private set; }
    public bool DropNew { get; private set; }

    protected void SendEsm(EgressSystemMessage esm)
    {
        esmBuffer.Enqueue(esm);
        EgressBuffer.Enqueue(esmBuffer);
    }

    protected byte[] SerializeMessageData(Message message)
    {
        messageSerializer.Clear();
        messageSerializer.SerializeMessage(message);
        byte[] data = messageSerializer.GetBytes();
        return data;
    }
}
}