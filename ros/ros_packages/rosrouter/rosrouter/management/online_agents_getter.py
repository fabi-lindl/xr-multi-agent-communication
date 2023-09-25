from rosrouter.agentclient.registry import agentclient_registry as Registry
from rosrouter.management.management_message_manager import ManagementMessageManager
from rosrouter.message.egress_system_message import EgressSystemMessage

from rosutility.config.params.getter import agent_param_getter as AgentParamGetter
from rosutility.message.serializer import serializer as Serializer
from rosutility.message.system_message_metadata import SystemMessageMetadata

class OnlineAgentsGetter(ManagementMessageManager):

    def process_ism(self, ism):
        management_data = ism.get_management_data()
        agentclient = self._get_agentclient(management_data)
        esm = self._create_esm(ism)
        agentclient.send(esm.serialize())

    def _get_agentclient(self, management_data):
        agent_type = management_data["agent_type"]
        agent_id = management_data["agent_id"]
        agentclient = Registry.get(agent_type, agent_id)
        return agentclient

    def _create_esm(self, ism):
        metadata = self._create_message_metadata(ism)
        data = self._create_message_data()
        esm = EgressSystemMessage(metadata, data)
        return esm

    def _create_message_metadata(self, ism):
        metadata = self._read_metadata_from_ism(ism)
        request_id = metadata["request_id"]
        metadata = SystemMessageMetadata(request_id=request_id)
        return metadata

    def _create_message_data(self) -> bytes:
        online_agents = self._get_online_agents()
        data = Serializer.dict_to_bytes(online_agents)
        return data

    def _get_online_agents(self):
        onlineAgents = {
            "rosAgentIds": self._get_ros_agentclient_ids(),
            "unityAgentIds": self._get_unity_agentclient_ids()
        }
        return onlineAgents

    def _get_ros_agentclient_ids(self):
        ros_agent_type = AgentParamGetter.get_ros_agent_type()
        ros_agent_ids = Registry.get_agentclient_ids_of_type(ros_agent_type)
        return ros_agent_ids

    def _get_unity_agentclient_ids(self):
        unity_agent_type = AgentParamGetter.get_unity_agent_type()
        unity_agent_ids = Registry.get_agentclient_ids_of_type(unity_agent_type)
        return unity_agent_ids
