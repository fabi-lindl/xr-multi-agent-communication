from rosrouter.agentclient.clients.agentclient import Agentclient

from rosutility.config.params.getter import agent_param_getter as AgentParamGetter

class UnityAgentclient(Agentclient):

    AGENT_TYPE = AgentParamGetter.get_unity_agent_type()

    def __init__(self, agent_id, group_id, conn_manager, **kwargs):
        super().__init__(agent_id, group_id, conn_manager,
                         UnityAgentclient.AGENT_TYPE)
