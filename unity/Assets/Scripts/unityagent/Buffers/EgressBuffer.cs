using System.Collections.Concurrent;

using UnityAgent.SystemMessage;

namespace UnityAgent.Buffers
{
public static class EgressBuffer
{
    private readonly static int heartbeatPeriod = 5000; // Milliseconds.
    private readonly static int capacity = 10;
    private readonly static BlockingCollection<EgressSystemMessageBuffer> buffer =
        new (capacity);

    public static int Capacity => capacity;

    public static void Enqueue(EgressSystemMessageBuffer esmb)
    {
        buffer.Add(esmb);
    }

    public static EgressSystemMessageBuffer Dequeue()
    {
        buffer.TryTake(out EgressSystemMessageBuffer item, heartbeatPeriod);
        return item;
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