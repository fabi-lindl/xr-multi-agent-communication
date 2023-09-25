using System;
using System.Net.Sockets;
using System.Threading;

using UnityEngine;

using UnityAgent.Buffers;
using UnityAgent.Management.Manager;
using UnityAgent.SystemMessage;
using UnityAgent.Util.Logging;

namespace UnityAgent.Operation
{
public sealed class Agent : MonoBehaviour
{
    private readonly byte agentType = 2;

    private string routerIpAddress = "127.0.0.1";
    private int routerPort = 7000;

    private const int maxNumConnectionAttempts = 3;
    private const int retryPeriod = 5; // Seconds

    private TcpClient client;
    private NetworkStream stream;

    private IngressSystemMessageManager ismManager;
    private IngressSystemMessageReceiver receiver;
    private EgressSystemMessageSender sender;

    private class Nested
    {
        static Nested()
        {
        }

        internal static readonly Agent instance = GetOrCreateInstance();

        private static Agent GetOrCreateInstance()
        {
            Agent instance = FindObjectOfType<Agent>();
            if (instance != null)
                return instance;
            GameObject agentObj = new("Agent");
            Agent newInstance = agentObj.AddComponent<Agent>();
            return newInstance;
        }
    }

    private Agent()
    {
    }

    public static Agent Instance => Nested.instance;

    public byte AgentType => agentType;

    public ushort AgentId { get; set; }

    public byte GroupId { get; private set; }

    private string RouterIpAddress
    {
        set
        {
            if (value != "")
                routerIpAddress = value;
        }
    }

    private int RouterPort
    {
        set
        {
            if (value != 0)
                routerPort = value;
        }
    }

    private void Awake()
    {
        receiver = IngressSystemMessageReceiver.Instance;
        sender = EgressSystemMessageSender.Instance;
    }

    private void OnApplicationQuit()
    {
        Stop();
    }

    public void Stop()
    {
        AgentLogger.LogStopAgent();
        StopWorkers();
        CloseConnection();
        AgentId = 0;
    }

    private void StopWorkers()
    {
        receiver.Stop();
        sender.Stop();
    }

    private void CloseConnection()
    {
        while (!sender.HasSentGoOfflineMessage());
        client.Close();
    }

    public bool Run(byte groupId=0, string ipAddress="", int port=0)
    {
        GroupId = groupId;
        ConfigureRouterSocket(ipAddress, port);
        if (!ConnectToRouter() || !ConfigureAgent()) return false;
        StartEgressSystemMessageSender();
        StartIngressSystemMessageManager();
        AgentLogger.LogAgentIsRunning();
        return true;
    }

    private void ConfigureRouterSocket(string ipAddress, int port)
    {
        RouterIpAddress = ipAddress;
        RouterPort = port;
    }

    private bool ConnectToRouter()
    {
        AgentLogger.LogConnectingToRouter();
        client = new();
        int numMadeAttempts = 0;
        while (numMadeAttempts < maxNumConnectionAttempts)
        {
            try
            {
                TryToConnect();
                return true;
            }
            catch (Exception e)
            {
                HandleConnectionException(++numMadeAttempts, e);
            }
        }
        AgentLogger.LogCouldNotConnectToRouter();
        return false;
    }

    private void TryToConnect()
    {
        client.Connect(routerIpAddress, routerPort);
        stream = client.GetStream();
        AgentLogger.LogConnectedToRouter();
    }

    private void HandleConnectionException(int numMadeAttempts, Exception e)
    {
        AgentLogger.LogConnectingToRouterFailed(routerIpAddress, routerPort, e);
        if (numMadeAttempts < maxNumConnectionAttempts)
        {
            AgentLogger.LogNextRetry(retryPeriod);
            Thread.Sleep(5000);
        }
    }

    private bool ConfigureAgent()
    {
        StartIngressSystemMessageReceiver();
        IngressSystemMessage ism = WaitForConfigMessage();
        ManagementMessageProcessor.ProcessIsm(ism);
        if (IsAgentConfigured()) return true;
        receiver.Stop();
        AgentLogger.LogAgentCouldNotBeConfigured();
        return false;
    }

    private IngressSystemMessage WaitForConfigMessage()
    {
        int maxWaitTime = 5000; // Milliseconds.
        IngressSystemMessage ism = IngressBuffer.TryDequeue(maxWaitTime);
        return ism;
    }

    private bool IsAgentConfigured()
    {
        return AgentId > 0;
    }

    private void StartIngressSystemMessageManager()
    {
        ismManager = IngressSystemMessageManager.Instance;
        StartCoroutine(ismManager.ManageReceivedIsms());
    }

    private void StartIngressSystemMessageReceiver()
    {
        receiver.Run(stream);
    }

    private void StartEgressSystemMessageSender()
    {
        sender.Run(stream);
    }
}
}