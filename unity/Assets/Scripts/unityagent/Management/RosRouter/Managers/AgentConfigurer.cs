using UnityAgent.Management.Message;
using UnityAgent.Operation;
using UnityAgent.SystemMessage;
using UnityAgent.Util.Logging;
using UnityAgent.Util.Serialization;

using UnityEngine;

namespace UnityAgent.Management.Manager
{
public class AgentConfigurer : ManagementMessageManager
{
    public override void ProcessIsm(IngressSystemMessage ism)
    {
        ConfigureUnityAgent(ism);
        AgentLogger.LogSendAgentConfiguration();
        EgressSystemMessage esm = new(CreateRoutingInfo(ism));
        SendEsm(esm);
    }

    private void ConfigureUnityAgent(IngressSystemMessage ism)
    {
        Agent agent = Agent.Instance;
        IngressAgentConfiguration config = NetworkDataDeserializer
            .BytesToObj<IngressAgentConfiguration>(ism.Data);
        agent.AgentId = config.agentId;
        AgentLogger.LogConfiguredAgent(agent.AgentId);
    }

    private RoutingInfo CreateRoutingInfo(IngressSystemMessage ism)
    {
        RoutingData routingData = new();
        Agent agent = Agent.Instance;
        SystemMessageMetadata metadata = ism.Metadata;
        EgressAgentConfiguration managementData = new(agent.AgentId,
                                                      agent.GroupId,
                                                      ism.RequestId);
        RoutingInfo routingInfo = new(routingData, managementData);
        return routingInfo;
    }
}
}