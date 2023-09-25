using System.Threading;
using System.Threading.Tasks;

using UnityAgent.SystemMessage;

namespace UnityAgent.Util.Threading
{
public class ResponseWaiter
{
    private readonly int maxWaitTime = 10000; // Milliseconds
    private CancellationTokenSource tokenSource = new CancellationTokenSource();

    public ResponseWaiter()
    {
    }

    public object Result { get; private set; }

    public async Task<object> Wait(int milliseconds=5000)
    {
        int waitTime = WaitTime(milliseconds);
        try
        {
            await Task.Delay(waitTime, tokenSource.Token);
        }
        catch (TaskCanceledException)
        {
        }
        return Result;
    }

    public void Resume(object result)
    {
        Result = result;
        tokenSource.Cancel();
    }

    private int WaitTime(int milliseconds)
    {
        if (IsInAllowedTimeWindow(milliseconds))
            return milliseconds;
        return maxWaitTime;
    }

    private bool IsInAllowedTimeWindow(int milliseconds)
    {
        return !(milliseconds < 0 || milliseconds > maxWaitTime);
    }
}
}