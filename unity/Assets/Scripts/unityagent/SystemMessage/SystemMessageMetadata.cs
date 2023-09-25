using UnityEngine;

using UnityAgent.Config;
using UnityAgent.Operation;
using UnityAgent.Util.Serialization;

namespace UnityAgent.SystemMessage
{
public class SystemMessageMetadata
{
    public byte senderAgentType;
    public ushort senderAgentId;
    public string messageManager = "";
    public uint requestId;
    public byte nodeType;
    public string nodeName = "";
    public string dataExchangeType = "";

    public SystemMessageMetadata()
    {
        ConfigureMetadataForThisAgent();
    }

    public SystemMessageMetadata(string messageManager)
    {
        this.messageManager = messageManager;
        ConfigureMetadataForThisAgent();
    }

    public SystemMessageMetadata(uint requestId)
    {
        this.requestId = requestId;
        ConfigureMetadataForThisAgent();
    }

    public SystemMessageMetadata(string messageManager, uint requestId)
    {
        this.messageManager = messageManager;
        this.requestId = requestId;
        ConfigureMetadataForThisAgent();
    }

    public SystemMessageMetadata(byte nodeType, string nodeName,
                                 string dataExchangeType)
    {
        this.nodeType = nodeType;
        this.nodeName = nodeName;
        this.dataExchangeType = dataExchangeType;
        ConfigureMetadataForThisAgent();
    }

    public SystemMessageMetadata(byte nodeType, string nodeName,
                                 string dataExchangeType, uint requestId)
    {
        this.nodeType = nodeType;
        this.nodeName = nodeName;
        this.dataExchangeType = dataExchangeType;
        this.requestId = requestId;
        ConfigureMetadataForThisAgent();
    }

    public SystemMessageMetadata(byte senderAgentType, ushort senderAgentId,
                                 string messageManager, uint requestId,
                                 byte nodeType, string nodeName,
                                 string dataExchangeType)
    {
        this.senderAgentType = senderAgentType;
        this.senderAgentId = senderAgentId;
        this.messageManager = messageManager;
        this.requestId = requestId;
        this.nodeType = nodeType;
        this.nodeName = nodeName;
        this.dataExchangeType = dataExchangeType;
    }

    private void ConfigureMetadataForThisAgent()
    {
        senderAgentType = Agent.Instance.AgentType;
        senderAgentId = Agent.Instance.AgentId;
    }

    public byte SenderAgentType => senderAgentType;
    public ushort SenderAgentId => senderAgentId;
    public uint RequestId => requestId;
    public string MessageManager => messageManager;
    public byte NodeType => nodeType;
    public string NodeName => nodeName;
    public string DataExchangeType => dataExchangeType;

    public bool HasMessageManager()
    {
        return messageManager != "";
    }

    public bool HasNodeName()
    {
        return nodeName != "";
    }

    public byte[] Serialize()
    {
        return NetworkDataSerializer.ObjToBytes(this);
    }

    public void Print()
    {
        string message = @$"
        ----------- Metadata ----------
        SenderAgentType  : {senderAgentType}
        SenderAgentId    : {senderAgentId}
        RequestId        : {requestId}
        MessageManager   : {messageManager}
        NodeType         : {nodeType}
        NodeName         : {nodeName}
        DataExchangeType : {dataExchangeType}
        -------------------------------
        ";
        Debug.Log(message);
    }
}
}