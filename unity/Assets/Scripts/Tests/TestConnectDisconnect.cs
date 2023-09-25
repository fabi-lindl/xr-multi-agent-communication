using UnityEngine;

using UnityAgent.Operation;

public class TestConnectDisconnect : MonoBehaviour
{
    private Agent agent;

    void Start()
    {
        agent = Agent.Instance;
        agent.Run(1);
    }

    void Update()
    {
    }

    void OnApplicationQuit()
    {
        agent.Stop();
    }
}