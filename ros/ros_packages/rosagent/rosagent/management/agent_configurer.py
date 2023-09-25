from rosagent.config.ros_agent_configuration import RosAgentConfiguration
from rosagent.management.functionality_getter import FunctionalityGetter
from rosagent.management.management_message_manager import ManagementMessageManager
from rosagent.message.egress_system_message import EgressSystemMessage

from rosutility.message.system_message_routing_data import SystemMessageRoutingData
from rosutility.message.system_message_routing_info import SystemMessageRoutingInfo

class AgentConfigurer(ManagementMessageManager):

    def __init__(self):
        super().__init__()
        self._rosagent_configuration = RosAgentConfiguration()

    def process_ism(self, ism):
        self._configure_ros_agent_id(ism)
        esm = self._create_esm(ism)
        self._logger.log_send_config_message()
        self._send_esm(esm)

    def _configure_ros_agent_id(self, ism):
        data = self._deserialize_management_message_data(ism)
        agent_id = data["agent_id"]
        self._logger.log_configure_agent(agent_id)
        self._rosagent_configuration.set_agent_id(agent_id)

    def _create_esm(self, ism):
        routing_info = self._create_routing_info(ism)
        esm = EgressSystemMessage(routing_info)
        return esm

    def _create_routing_info(self, ism):
        agent_type = self._rosagent_configuration.get_agent_type()
        routing_data = SystemMessageRoutingData(agent_type=agent_type)
        management_data = {
            "agent_type": self._rosagent_configuration.get_agent_type(),
            "agent_id": self._rosagent_configuration.get_agent_id(),
            "group_id": self._get_group_id(),
            "request_id": ism.get_request_id(),
            "message_manager": "AgentclientConfigurer",
            "functionality": self._get_functionality()
        }
        routing_info = SystemMessageRoutingInfo(routing_data, management_data)
        return routing_info

    def _get_group_id(self):
        return self._rosagent_configuration.get_group_id()

    def _get_functionality(self):
        functionality_getter = FunctionalityGetter()
        functionality = functionality_getter.get_functionality()
        return functionality
