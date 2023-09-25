using System.Collections.Generic;

namespace UnityAgent.SystemMessage
{
public class EgressSystemMessageBuffer
{
    private uint bufferSize = 10;
    private readonly bool dropNew = false;
    private readonly LinkedList<EgressSystemMessage> buffer = new();

    public EgressSystemMessageBuffer()
    {
    }

    public EgressSystemMessageBuffer(uint bufferSize, bool dropNew)
    {
        BufferSize = bufferSize;
        this.dropNew = dropNew;
    }

    public uint BufferSize
    {
        get => bufferSize;
        set
        {
            if (value == 0)
                bufferSize = 10;
        }
    }
    public bool DropNew => dropNew;


    public void Enqueue(EgressSystemMessage message)
    {
        lock (buffer)
        {
            if (HasFreeSpace())
                buffer.AddLast(message);
            else if (DropsOld())
            {
                buffer.RemoveFirst();
                buffer.AddLast(message);
            }
        }
    }

    public EgressSystemMessage Dequeue()
    {
        lock (buffer)
        {
            EgressSystemMessage esm = buffer.First.Value;
            buffer.RemoveFirst();
            return esm;
        }
    }

    public void Clear()
    {
        buffer.Clear();
    }

    private bool DropsOld()
    {
        return !dropNew;
    }

    private bool HasFreeSpace()
    {
        return buffer.Count < bufferSize;
    }
}
}