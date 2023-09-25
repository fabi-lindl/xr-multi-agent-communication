using System.Net.Sockets;
using System.Threading.Tasks;

using UnityAgent.Buffers;
using UnityAgent.Management.Message;
using UnityAgent.SystemMessage;

namespace UnityAgent.Operation
{
public class EgressSystemMessageSender : WorkerThread
{
    private NetworkStream stream;

    private class Nested
    {
        static Nested()
        {
        }

        internal static readonly EgressSystemMessageSender instance = new();
    }

    private EgressSystemMessageSender() : base("ESM Sender")
    {
    }

    public static EgressSystemMessageSender Instance => Nested.instance;

    public bool HasSentGoOfflineMessage()
    {
        return HasStopped();
    }

    public bool HasStopped()
    {
        return task.Status == TaskStatus.Canceled ||
               task.Status == TaskStatus.Faulted ||
               task.Status == TaskStatus.RanToCompletion;
    }

    public void Run(NetworkStream stream)
    {
        this.stream = stream;
        task = Task.Run(() => Send());
    }

    private void Send()
    {
        try
        {
            while (!CancellationToken.IsCancellationRequested)
                SendEgressSystemMessage();
            SendGoOfflineMessageToAllNetworkNodes();
        }
        catch
        {
            HandleConnectionClosedException();
        }
        CancellationToken.ThrowIfCancellationRequested();
    }

    private void SendEgressSystemMessage()
    {
        EgressSystemMessage esm = DequeueEsm();
        if (esm.Metadata != null)
        {
            SystemMessageMetadata m = esm.Metadata;
            try
            {
                RoutingInfo ri = esm.RoutingInfo;
                RoutingManagementData rmd = ri.RoutingManagementData;
            }
            catch
            {
            }
        }
        byte[] data = esm.Serialize();
        stream.Write(data, 0, data.Length);
    }

    private EgressSystemMessage DequeueEsm()
    {
        EgressSystemMessageBuffer esmBuffer = EgressBuffer.Dequeue();
        if (IsHeartbeat(esmBuffer))
            return HeartbeatMessage.Message;
        return esmBuffer.Dequeue();
    }

    private bool IsHeartbeat(EgressSystemMessageBuffer buffer)
    {
        return buffer == null;
    }

    private void SendGoOfflineMessageToAllNetworkNodes()
    {
        EgressSystemMessage esm = CreateGoOfflineMessage();
        byte[] data = esm.Serialize();
        stream.Write(data, 0, data.Length);
    }

    private EgressSystemMessage CreateGoOfflineMessage()
    {
        RoutingData routingData = new();
        RoutingManagementData rmd = new GoOffline(Agent.Instance.AgentId);
        RoutingInfo routingInfo = new(routingData, rmd);
        EgressSystemMessage esm = new(routingInfo);
        return esm;
    }
}
}