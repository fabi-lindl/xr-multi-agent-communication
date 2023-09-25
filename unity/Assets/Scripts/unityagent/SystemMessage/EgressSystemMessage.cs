using Unity.VisualScripting;
using UnityAgent.Util.Serialization;

namespace UnityAgent.SystemMessage
{
public class EgressSystemMessage
{
    private readonly RoutingInfo routingInfo;
    private readonly SystemMessageMetadata metadata = null;
    private readonly byte[] data = new byte[0];

    public EgressSystemMessage(RoutingInfo routingInfo)
    {
        this.routingInfo = routingInfo;
    }

    public EgressSystemMessage(RoutingInfo routingInfo,
                               SystemMessageMetadata metadata)
    {
        this.routingInfo = routingInfo;
        this.metadata = metadata;
    }

    public EgressSystemMessage(RoutingInfo routingInfo,
                               SystemMessageMetadata metadata,
                               byte[] data)
    {
        this.routingInfo = routingInfo;
        this.metadata = metadata;
        this.data = data;
    }

    public SystemMessageMetadata Metadata => metadata;
    public RoutingInfo RoutingInfo => routingInfo;

    public byte[] Serialize()
    {
        byte[] sMetadata = metadata == null ? new byte[0] : metadata.Serialize();

        int sMetadataSize = sMetadata.Length;
        byte[] metadataSizeFlag = NetworkDataSerializer.IntToBytes(sMetadataSize);
        int metadataSizeFlagSize = metadataSizeFlag.Length;

        int sDataSize = data.Length;
        byte[] dataSizeFlag = NetworkDataSerializer.IntToBytes(sDataSize);
        int dataSizeFlagSize = dataSizeFlag.Length;

        int totalMessageSize = metadataSizeFlagSize + sMetadataSize +
                               dataSizeFlagSize + sDataSize;
        byte[] sRoutingInfo = SerializeRoutingInfo(totalMessageSize);
        int sRoutingInfoSize = sRoutingInfo.Length;

        int systemMessageSize = sRoutingInfoSize + totalMessageSize;
        byte[] systemMessage = new byte[systemMessageSize];
        
        int offset = 0;
        System.Buffer.BlockCopy(sRoutingInfo, 0, systemMessage, 0,
                                sRoutingInfoSize);
        offset += sRoutingInfoSize;
        System.Buffer.BlockCopy(metadataSizeFlag, 0, systemMessage, offset,
                                metadataSizeFlagSize);
        offset +=  metadataSizeFlagSize;
        System.Buffer.BlockCopy(sMetadata, 0, systemMessage, offset,
                                sMetadataSize);
        offset += sMetadataSize;
        System.Buffer.BlockCopy(dataSizeFlag, 0, systemMessage, offset,
                                dataSizeFlagSize);
        offset += dataSizeFlagSize;
        System.Buffer.BlockCopy(data, 0, systemMessage, offset, sDataSize);

        return systemMessage;
    }

    private byte[] SerializeRoutingInfo(int totalMessageSize)
    {
        routingInfo.TotalMessageSize = totalMessageSize;
        byte[] sRoutingInfo = routingInfo.Serialize();
        return sRoutingInfo;
    }
}
}