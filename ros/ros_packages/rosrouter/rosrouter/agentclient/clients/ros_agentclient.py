from rosrouter.agentclient.clients.agentclient import Agentclient

from rosutility.config.params.getter import agent_param_getter as AgentParamGetter

class RosAgentclient(Agentclient):

    AGENT_TYPE = AgentParamGetter.get_ros_agent_type()

    def __init__(self, agent_id, group_id, conn_manager,
                 functionality, **kwargs):
        super().__init__(agent_id, group_id, conn_manager,
                         RosAgentclient.AGENT_TYPE)
        self._functionality = functionality

    def get_functionality(self):
        return self._functionality
