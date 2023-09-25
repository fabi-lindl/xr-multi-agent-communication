using UnityAgent.Buffers;
using UnityAgent.SystemMessage;
using UnityAgent.Util.Threading;

namespace UnityAgent.Operation
{
public class IngressSystemMessageManager
{
    private class Nested
    {
        static Nested()
        {
        }

        internal static readonly IngressSystemMessageManager instance = new();
    }

    private IngressSystemMessageManager()
    {
    }

    public static IngressSystemMessageManager Instance => Nested.instance;

    public System.Collections.IEnumerator ManageReceivedIsms()
    {
        while (true)
        {
            IngressSystemMessage ism = IngressBuffer.TryDequeue();
            if (ism != null)
                ProcessIsm(ism);
            yield return null;
        }
    }

    private void ProcessIsm(IngressSystemMessage ism)
    {
        if (ism.IsInterfaceNodeMessage())
            InterfaceNodeMessageProcessor.ProcessIsm(ism);
        else if (ism.IsManagementMessage())
            ManagementMessageProcessor.ProcessIsm(ism);
        else
            ProcessPlainRequestMessage(ism);
    }

    private void ProcessPlainRequestMessage(IngressSystemMessage ism)
    {
        uint requestId = ism.RequestId;
        ResponseWaiter rw = Communicator.GetResponseWaiter(requestId);
        rw?.Resume(ism);
    }
}
}