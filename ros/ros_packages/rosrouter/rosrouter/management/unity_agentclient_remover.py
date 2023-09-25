from rosrouter.management.management_message_manager import ManagementMessageManager
from rosrouter.agentclient.registry.agentclient_connection_surveillancer import AgentclientConnectionSurveillancer
from rosrouter.agentclient.registry import agentclient_registry as Registry

from rosutility.config.params.getter import agent_param_getter as AgentParamGetter

class UnityAgentclientRemover(ManagementMessageManager):

    def process_ism(self, ism):
        agentclient = self._get_agentclient(ism)
        self._prune_agentclient(agentclient)

    def _get_agentclient(self, ism):
        agentclient_type = AgentParamGetter.get_unity_agent_type()
        management_data = ism.get_management_data()
        agentclient_id = management_data["agent_id"]
        agentclient = Registry.get(agentclient_type, agentclient_id)
        return agentclient

    def _prune_agentclient(self, agentclient):
        surveillancer = AgentclientConnectionSurveillancer()
        surveillancer.prune_agentclient(agentclient)
