using System;
using System.Text;
using UnityEngine;

namespace UnityAgent.Util.Serialization
{
public static class NetworkDataDeserializer
{
    public static T BytesToObj<T>(byte[] data)
    {
        string json = Encoding.UTF8.GetString(data);
        return JsonToObj<T>(json);
    }

    public static T JsonToObj<T>(string data)
    {
        return JsonUtility.FromJson<T>(data);
    }

    public static int BytesToInt(byte[] data)
    {
        return BitConverter.ToInt32(data, 0);
    }
}
}