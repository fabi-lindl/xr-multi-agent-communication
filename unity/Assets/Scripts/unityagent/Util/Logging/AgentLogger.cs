using System;
using UnityEngine;

namespace UnityAgent.Util.Logging
{
public static class AgentLogger
{
    public static void LogConnectingToRouter()
    {
        LogInfo("Connecting to router ...");
    }

    public static void LogConnectingToRouterFailed(string ipAddress, int port,
                                                   Exception e)
    {
        LogInfo(@$"Connecting to router at {ipAddress}: 
                {port} failed with exception {e}!");
    }

    public static void LogNextRetry(int retryPeriod)
    {
        LogInfo($"Next retry in {retryPeriod} s ...");
    }

    public static void LogCouldNotConnectToRouter()
    {
        LogInfo("Connection to router could not be established!");
    }

    public static void LogConnectedToRouter()
    {
        LogInfo("Connection established!");
    }

    public static void LogAgentCouldNotBeConfigured()
    {
        LogInfo("Configuration of unityagent failed!");
    }

    public static void LogConfiguredAgent(ushort agentId)
    {
        LogInfo($"Configure unityagent of id {agentId} ...");
    }

    public static void LogSendAgentConfiguration()
    {
        LogInfo("Sending unityagent config message ...");
    }

    public static void LogAgentIsRunning()
    {
        LogInfo("Agent is running!");
    }

    public static void LogStopAgent()
    {
        LogInfo("Stopped Agent.");
    }

    public static void LogStoppedWorkerThread(string workerName)
    {
        LogInfo($"Stopped worker thread \"{workerName}\".");
    }

    public static void LogWorkerThreadConnectionClosed(string workerName)
    {
        LogInfo($"Connection of worker thread \"{workerName}\" closed.");
    }

    public static void LogErrorInResponseCallbackFunction()
    {
        LogInfo("Error in callback function! Response not processed.");
    }

    public static void LogInfo(string message)
    {
        Debug.Log(message);
    }
}
}