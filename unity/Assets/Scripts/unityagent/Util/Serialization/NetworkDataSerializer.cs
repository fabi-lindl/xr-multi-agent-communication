using System;
using System.Text;
using UnityEngine;

namespace UnityAgent.Util.Serialization
{
public static class NetworkDataSerializer
{
    private static readonly UTF8Encoding utf8 = new();

    public static byte[] ObjToBytes<T>(T obj)
    {
        string jsonObj = JsonUtility.ToJson(obj);
        return StringToBytes(jsonObj);
    }

    public static byte[] StringToBytes(string value)
    {
        return utf8.GetBytes(value);
    }

    public static byte[] ByteToBytes(byte value)
    {
        return new byte[] { value };
    }

    public static byte[] SbyteToBytes(sbyte value)
    {
        return BitConverter.GetBytes(value);
    }

    public static byte[] SbytesToBytes(sbyte[] values, int fixedNumItems=0)
    {
        return ArrayToBytes(values, sizeof(sbyte), fixedNumItems);
    }

    public static byte[] ShortToBytes(short value)
    {
        return BitConverter.GetBytes(value);
    }

    public static byte[] ShortsToBytes(short[] values, int fixedNumItems=0)
    {
        return ArrayToBytes(values, sizeof(short), fixedNumItems);
    }

    public static byte[] UshortToBytes(short value)
    {
        return BitConverter.GetBytes(value);
    }

    public static byte[] UshortsToBytes(ushort[] values, int fixedNumItems=0)
    {
        return ArrayToBytes(values, sizeof(ushort), fixedNumItems);
    }

    public static byte[] IntToBytes(int value)
    {
        return BitConverter.GetBytes(value);
    }

    public static byte[] IntsToBytes(int[] values, int fixedNumItems=0)
    {
        return ArrayToBytes(values, sizeof(int), fixedNumItems);
    }

    public static byte[] UintToBytes(uint value)
    {
        return BitConverter.GetBytes(value);
    }

    public static byte[] UintsToBytes(uint[] values, int fixedNumItems=0)
    {
        return ArrayToBytes(values, sizeof(uint), fixedNumItems);
    }

    public static byte[] LongToBytes(long value)
    {
        return BitConverter.GetBytes(value);
    }

    public static byte[] LongsToBytes(long[] values, int fixedNumItems=0)
    {
        return ArrayToBytes(values, sizeof(long), fixedNumItems);
    }

    public static byte[] UlongToBytes(ulong value)
    {
        return BitConverter.GetBytes(value);
    }

    public static byte[] UlongsToBytes(ulong[] values, int fixedNumItems=0)
    {
        return ArrayToBytes(values, sizeof(ulong), fixedNumItems);
    }

    public static byte[] FloatToBytes(float value)
    {
        return BitConverter.GetBytes(value);
    }

    public static byte[] FloatsToBytes(float[] values, int fixedNumItems=0)
    {
        return ArrayToBytes(values, sizeof(float), fixedNumItems);
    }

    public static byte[] DoubleToBytes(double value)
    {
        return BitConverter.GetBytes(value);
    }

    public static byte[] DoublesToBytes(double[] values, int fixedNumItems=0)
    {
        return ArrayToBytes(values, sizeof(double), fixedNumItems);
    }

    public static byte[] ArrayToBytes<T>(T[] values, int sizeOfValue,
                                          int fixedNumItems)
    {
        int bufferSize = BufferSize(values.Length, sizeOfValue, fixedNumItems);
        byte[] buffer = new byte[bufferSize];
        Buffer.BlockCopy(values, 0, buffer, 0, values.Length * sizeOfValue);
        return buffer;
    }

    private static int BufferSize(int numValues, int sizeOfValue,
                                  int fixedNumItems)
    {
        if (fixedNumItems == 0)
            return numValues * sizeOfValue;
        System.Diagnostics.Debug.Assert(numValues <= fixedNumItems);
        return fixedNumItems * sizeOfValue;
    }
}
}