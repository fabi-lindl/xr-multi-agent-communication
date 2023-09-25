using System.Collections.Generic;
using System.Diagnostics;

using UnityAgent.Util.Serialization;
using UnityAgent.Config;

namespace UnityAgent.SystemMessage
{
public class RoutingData
{
    private static int maxNumDestAgents =
        RoutingDataConfiguration.GetMaxNumDestAgentIds();
    private static int maxNumDestGroups =
        RoutingDataConfiguration.GetMaxNumDestGroupIds();
    private static int serializedRoutingDataSize =
        RoutingDataConfiguration.GetSerializedRoutingDataSize();
    
    private byte agentType = 0;
    private readonly List<ushort> destAgentIds = new(maxNumDestAgents);
    private readonly List<byte> destGroupIds = new(maxNumDestGroups);
    private int managementDataSize; // Determined during serialization.
    private int totalMessageSize; // Determined during serialization.

    public RoutingData()
    {
    }

    public RoutingData(byte agentType, ushort destAgentId)
    {
        this.agentType = agentType;
        destAgentIds.Add(destAgentId);
    }

    public RoutingData(byte agentType, List<ushort> destAgentIds)
    {
        Debug.Assert(!ExceedsMaxNumDestAgents(destAgentIds.Count));
        this.agentType = agentType;
        this.destAgentIds = destAgentIds;
    }

    public RoutingData(byte agentType, byte destinationGroupId)
    {
        this.agentType = agentType;
        destGroupIds.Add(destinationGroupId);
    }

    public RoutingData(byte agentType, List<byte> destGroupIds)
    {
        Debug.Assert(!ExceedsMaxNumDestGroups(destGroupIds.Count));
        this.agentType = agentType;
        this.destGroupIds = destGroupIds;
    }

    public RoutingData(byte agentType, List<ushort> destAgentIds,
                       List<byte> destGroupIds)
    {
        Debug.Assert(!ExceedsMaxNumDestAgents(destAgentIds.Count));
        Debug.Assert(!ExceedsMaxNumDestGroups(destGroupIds.Count));
        this.agentType = agentType;
        this.destAgentIds = destAgentIds;
        this.destGroupIds = destGroupIds;
    }

    public static int MaxNumDestAgents => maxNumDestAgents;
    public static int MaxNumDestGroups => maxNumDestGroups;
    public byte AgentType
    {
        get => agentType;
        set => agentType = value;
    }
    public int ManagementDataSize
    {
        get => managementDataSize;
        set => managementDataSize = value;
    }
    public int TotalMessageSize
    {
        get => totalMessageSize;
        set => totalMessageSize = value;
    }

    public static bool ExceedsMaxNumDestAgents(int numAgentIds)
    {
        return numAgentIds > MaxNumDestAgents;
    }

    public static bool ExceedsMaxNumDestGroups(int numGroupIds)
    {
        return numGroupIds > MaxNumDestGroups;
    }

    public bool HasDestinations()
    {
        return !(AreDestAgentIdsEmpty() && AreDestGroupIdsEmpty());
    }

    public bool AddDestAgentIds(List<ushort> newDestAgentIds)
    {
        if (!CanAddDestAgentIds(newDestAgentIds))
            return false;
        foreach (ushort newDestAgentId in newDestAgentIds)
            AddDestAgentId(newDestAgentId);
        return true;
    }

    public bool AddDestAgentId(ushort destAgentId)
    {
        if (CanAddDestAgentId(destAgentId))
            return false;
        destAgentIds.Add(destAgentId);
        return true;
    }

    private bool CanAddDestAgentIds(List<ushort> newDestAgentIds)
    {
        int numToAdd = 0;
        foreach (ushort id in newDestAgentIds)
            if (!HasDestAgentId(id))
                numToAdd++;
        int numAfterAdd = destAgentIds.Count + numToAdd;
        return numAfterAdd <= maxNumDestAgents;
    }

    private bool CanAddDestAgentId(ushort destAgentId)
    {
        return !AreDestAgentIdsFull() && !HasDestAgentId(destAgentId);
    }

    private bool AreDestAgentIdsFull()
    {
        return destAgentIds.Count == maxNumDestAgents;
    }

    private bool AreDestAgentIdsEmpty()
    {
        return destGroupIds.Count == 0;
    }

    public bool HasDestAgentId(ushort destAgentId)
    {
        return destAgentIds.Contains(destAgentId);
    }

    public bool AddDestGroupIds(List<byte> newDestGroupIds)
    {
        if (!CanAddDestGroupIds(newDestGroupIds))
            return false;
        foreach (byte newDestGroupId in newDestGroupIds)
            AddDestGroupId(newDestGroupId);
        return true;
    }

    public bool AddDestGroupId(byte destGroupId)
    {
        if (CanAddDestGroupId(destGroupId))
            return false;
        destGroupIds.Add(destGroupId);
        return true;
    }

    private bool CanAddDestGroupIds(List<byte> newDestGroupIds)
    {
        int numToAdd = 0;
        foreach (byte id in newDestGroupIds)
            if (!HasDestGroupId(id))
                numToAdd++;
        int numAfterAdd = destGroupIds.Count + numToAdd;
        return numAfterAdd <= maxNumDestGroups;
    }

    private bool CanAddDestGroupId(byte destGroupId)
    {
        return !AreDestGroupIdsFull() && !HasDestGroupId(destGroupId);
    }

    private bool AreDestGroupIdsFull()
    {
        return destGroupIds.Count == maxNumDestGroups;
    }

    private bool AreDestGroupIdsEmpty()
    {
        return destGroupIds.Count == 0;
    }

    public bool HasDestGroupId(byte destGroupId)
    {
        return destGroupIds.Contains(destGroupId);
    }

    public void RemoveDestAgentIds(List<ushort> destAgentIds)
    {
        foreach (ushort destAgentId in destAgentIds)
            RemoveDestAgentId(destAgentId);
    }

    public bool RemoveDestAgentId(ushort destAgentId)
    {
        return destAgentIds.Remove(destAgentId);
    }

    public void RemoveDestGroupIds(List<byte> destGroupIds)
    {
        foreach (byte destGroupId in destGroupIds)
            RemoveDestGroupId(destGroupId);
    }

    public bool RemoveDestGroupId(byte destGroupId)
    {
        return destGroupIds.Remove(destGroupId);
    }

    public void ClearDestAgentIds()
    {
        destAgentIds.Clear();
    }

    public void ClearDestGroupIds()
    {
        destGroupIds.Clear();
    }

    public byte[] Serialize()
    {
        byte[] sAgentType = SerializeAgentType();
        byte[] sdestAgentIds = SerializeDestAgentIds();
        byte[] sdestGroupIds = SerializeDestGroupIds();
        byte[] sManagementDataSize = SerializeManagementDataSize();
        byte[] sTotalMessageSize = SerializeTotalMessageSize();
        byte[][] items = new byte[][]
        {
            sAgentType, sdestAgentIds, sdestGroupIds,
            sManagementDataSize, sTotalMessageSize
        };
        byte[] sRoutingData = CreateSerializedRoutingData(items);
        return sRoutingData;
    }

    private byte[] SerializeAgentType()
    {
        return NetworkDataSerializer.ByteToBytes(agentType);
    }

    private byte[] SerializeDestAgentIds()
    {
        return NetworkDataSerializer.UshortsToBytes(destAgentIds.ToArray(),
                                                    maxNumDestAgents);
    }

    private byte[] SerializeDestGroupIds()
    {
        return NetworkDataSerializer.ArrayToBytes(destGroupIds.ToArray(),
                                                  sizeof(byte),
                                                  maxNumDestGroups);
    }

    private byte[] SerializeManagementDataSize()
    {
        return NetworkDataSerializer.IntToBytes(managementDataSize);
    }

    private byte[] SerializeTotalMessageSize()
    {
        return NetworkDataSerializer.IntToBytes(totalMessageSize);
    }

    private byte[] CreateSerializedRoutingData(byte[][] items)
    {
        byte[] sRoutingData = new byte[serializedRoutingDataSize];
        int itemLength;
        int offset = 0;
        foreach (byte[] item in items)
        {
            itemLength = item.Length;
            System.Buffer.BlockCopy(item, 0, sRoutingData, offset, itemLength);
            offset += itemLength;
        }
        return sRoutingData;
    }
}
}