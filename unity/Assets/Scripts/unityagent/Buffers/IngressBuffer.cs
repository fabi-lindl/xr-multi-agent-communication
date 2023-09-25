using System.Collections.Concurrent;
using System.Threading;
using UnityAgent.SystemMessage;

namespace UnityAgent.Buffers
{
public static class IngressBuffer
{
    private readonly static int capacity = 10;
    private readonly static BlockingCollection<IngressSystemMessage> buffer =
        new(capacity);
    
    public static int Capacity => capacity;

    public static void Enqueue(IngressSystemMessage ism)
    {
        buffer.Add(ism);
    }

    public static IngressSystemMessage Dequeue()
    {
        return buffer.Take();
    }

    public static IngressSystemMessage Dequeue(CancellationToken token)
    {
        return buffer.Take(token);
    }

    public static IngressSystemMessage TryDequeue()
    {
        if (buffer.TryTake(out IngressSystemMessage ism))
            return ism;
        return null;
    }

    public static IngressSystemMessage TryDequeue(int waitTime)
    {
        if (buffer.TryTake(out IngressSystemMessage ism, waitTime))
            return ism;
        return null;
    }

    public static int Size()
    {
        return buffer.Count;
    }

    public static bool IsEmpty()
    {
        return buffer.Count == 0;
    }
}
}