using UnityAgent.Buffers;
using UnityAgent.SystemMessage;

namespace UnityAgent.Management.Manager
{
public abstract class ManagementMessageManager
{
    public abstract void ProcessIsm(IngressSystemMessage ism);

    protected void SendEsm(EgressSystemMessage esm)
    {
        EgressSystemMessageBuffer esmb = new();
        esmb.Enqueue(esm);
        EgressBuffer.Enqueue(esmb);
    }
}
}