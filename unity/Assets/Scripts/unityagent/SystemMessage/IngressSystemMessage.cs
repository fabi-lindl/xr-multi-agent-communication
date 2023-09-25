namespace UnityAgent.SystemMessage
{
public class IngressSystemMessage
{
    private readonly SystemMessageMetadata metadata;
    private readonly byte[] data;

    public IngressSystemMessage(SystemMessageMetadata metadata, byte[] data)
    {
        this.metadata = metadata;
        this.data = data;
    }

    public SystemMessageMetadata Metadata => metadata;
    public byte[] Data => data;
    public byte SenderAgentType => metadata.SenderAgentType;
    public ushort SenderAgentId => metadata.SenderAgentId;
    public byte NodeType => metadata.NodeType;
    public string NodeName => metadata.NodeName;
    public uint RequestId => metadata.RequestId;
    public string MessageManager => metadata.MessageManager;
    
    public bool IsManagementMessage()
    {
        return metadata.HasMessageManager();
    }

    public bool IsInterfaceNodeMessage()
    {
        return metadata.HasNodeName();
    }
}
}