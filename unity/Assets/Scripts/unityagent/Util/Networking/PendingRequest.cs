using System;

using UnityAgent.Util.Threading;

namespace UnityAgent.Util.Networking
{
public class PendingRequest
{
    public PendingRequest(uint requestId, ResponseWaiter rw)
    {
        RequestId = requestId;
        ResponseWaiter = rw;
        Timestamp = DateTime.UtcNow;
    }

    public uint RequestId { get; private set; }
    public ResponseWaiter ResponseWaiter { get; private set; }
    public DateTime Timestamp { get; private set; }

    public bool HasExpired(int deltaSeconds)
    {
        DateTime now = DateTime.UtcNow;
        DateTime timestampUpperLim = Timestamp.AddSeconds(deltaSeconds);
        int result = DateTime.Compare(now, timestampUpperLim);
        return result > 0;
    }
}
}