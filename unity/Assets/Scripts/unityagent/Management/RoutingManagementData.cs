using System;
using System.Text;
using UnityEngine;

using UnityAgent.Util.Serialization;

namespace UnityAgent.Management.Message
{
public abstract class RoutingManagementData
{
    public abstract string MessageManager { get; }
    
    public static byte[] Serialize(RoutingManagementData data)
    {
        return NetworkDataSerializer.ObjToBytes(data);
    }
}
}