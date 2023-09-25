using System;
using System.Threading;
using System.Threading.Tasks;

using UnityAgent.Util.Logging;

namespace UnityAgent.Operation
{
public abstract class WorkerThread
{
    public Task task;
    protected CancellationTokenSource tokenSource = new();

    protected WorkerThread(string workerName)
    {
        WorkerName = workerName;
    }

    public string WorkerName { get; private set; }
    public CancellationToken CancellationToken => tokenSource.Token;

    public async void Stop()
    {
        tokenSource.Cancel();
        try
        {
            await task;
        }
        catch (OperationCanceledException)
        {
            AgentLogger.LogStoppedWorkerThread(WorkerName);
        }
        tokenSource = new();
    }

    protected void HandleConnectionClosedException()
    {
        AgentLogger.LogWorkerThreadConnectionClosed(WorkerName);
    }
}
}