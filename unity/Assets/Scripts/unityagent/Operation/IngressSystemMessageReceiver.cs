using System;
using System.Net.Sockets;
using System.Threading.Tasks;

using UnityAgent.Buffers;
using UnityAgent.SystemMessage;
using UnityAgent.Util.Serialization;

namespace UnityAgent.Operation
{
public class IngressSystemMessageReceiver : WorkerThread
{
    private NetworkStream stream;
    private const int maxReceiveChunkSize = 4096; // Recommended by docs.
    private const int lengthIdentifierSize = sizeof(int);

    private class Nested
    {
        static Nested()
        {
        }

        internal static readonly IngressSystemMessageReceiver instance = new();
    }

    private IngressSystemMessageReceiver() : base("ISM Reader")
    {
    }

    public static IngressSystemMessageReceiver Instance => Nested.instance;

    public void Run(NetworkStream stream)
    {
        this.stream = stream;
        task = Task.Run(() => Receive());
    }

    private void Receive()
    {
        try
        {
            while (!CancellationToken.IsCancellationRequested)
                ReceiveIsmIntoIngressBuffer();
        }
        catch
        {
            HandleConnectionClosedException();
        }
        CancellationToken.ThrowIfCancellationRequested();
    }

    private void ReceiveIsmIntoIngressBuffer()
    {
        SystemMessageMetadata metadata = ReadMessageMetadata();
        byte[] data = ReadMessageData();
        IngressSystemMessage ism = new(metadata, data);
        IngressBuffer.Enqueue(ism);
    }

    private SystemMessageMetadata ReadMessageMetadata()
    {

        int metadataSize = ReadSizeIdentifier();
        byte[] data = ReadData(metadataSize);
        SystemMessageMetadata metadata =
            NetworkDataDeserializer.BytesToObj<SystemMessageMetadata>(data);
        return metadata;
    }

    private byte[] ReadMessageData()
    {
        int dataSize = ReadSizeIdentifier();
        byte[] data = ReadData(dataSize);
        return data;
    }

    private int ReadSizeIdentifier()
    {
        byte[] data = ReadData(lengthIdentifierSize);
        int size = NetworkDataDeserializer.BytesToInt(data);
        return size;
    }

    private byte[] ReadData(int size)
    {
        byte[] buffer = new byte[size];
        int numMissingBytes, chunkSize, receivedChunkSize;
        int numBytesReceived = 0;
        while (numBytesReceived < size)
        {
            numMissingBytes = size - numBytesReceived;
            chunkSize = Math.Min(numMissingBytes, maxReceiveChunkSize);
            receivedChunkSize = stream.Read(buffer, numBytesReceived, chunkSize);
            numBytesReceived += receivedChunkSize;
        }
        return buffer;
    }
}
}