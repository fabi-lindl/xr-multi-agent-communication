using UnityAgent.Management.Message;

namespace UnityAgent.SystemMessage
{
public class RoutingInfo
{
    private readonly RoutingData data;
    private readonly RoutingManagementData managementData;

    public RoutingInfo(RoutingData data)
    {
        this.data = data;
    }

    public RoutingInfo(RoutingData data, RoutingManagementData managementData)
    {
        this.data = data;
        this.managementData = managementData;
    }

    public RoutingData RoutingData => data;
    public RoutingManagementData RoutingManagementData => managementData;

    public byte AgentType => data.AgentType;
    public int ManagementDataSize => data.ManagementDataSize;
    public int TotalMessageSize
    {
        get => data.TotalMessageSize;
        set => data.TotalMessageSize = value;
    }

    public bool HasManagementData()
    {
        return managementData != null;
    }

    public bool IsBroadcastMessage()
    {
        return data.AgentType == 0;
    }

    public byte[] Serialize()
    {
        byte[] sManagementData = SerializeManagementData();
        byte[] sData = SerializeRoutingData(sManagementData);
        byte[] sRoutingData = CreateSerializedRoutingInfo(sData,
                                                          sManagementData);
        return sRoutingData;
    }

    private byte[] SerializeManagementData()
    {
        if (managementData == null)
            return new byte[0];
        return RoutingManagementData.Serialize(managementData);
    }

    private byte[] SerializeRoutingData(byte[] SerializedManagementData)
    {
        data.ManagementDataSize = SerializedManagementData.Length;
        byte[] sData = data.Serialize();
        return sData;
    }

    private byte[] CreateSerializedRoutingInfo(byte[] routingData,
                                               byte[] managementData)
    {
        int dataSize = routingData.Length;
        int mDataSize = managementData.Length;
        byte[] buffer = new byte[dataSize + mDataSize];
        System.Buffer.BlockCopy(routingData, 0, buffer, 0, dataSize);
        System.Buffer.BlockCopy(managementData, 0, buffer, dataSize, mDataSize);
        return buffer;
    }
}
}