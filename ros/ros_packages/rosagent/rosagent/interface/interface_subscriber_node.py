from rosagent.config.ros_agent_configuration import RosAgentConfiguration
from rosagent.interface.interface_sender_node import InterfaceSenderNode
from rosagent.message.egress_system_message import EgressSystemMessage
from rosagent.util import data_exchange_type_module_importer as Importer

from rosutility.message.system_message_routing_data import SystemMessageRoutingData
from rosutility.message.system_message_routing_info import SystemMessageRoutingInfo

class InterfaceSubscriberNode(InterfaceSenderNode):

    NODE_TYPE = 1
    _message_manager = "RosAgentSubscriberDataForwarder"

    def __new__(cls, data_exchange_type, *args, **kwargs):
        class_ = Importer.get_message_type_class(data_exchange_type)
        if class_ is None: return None
        return super().__new__(cls)

    def __init__(self, registrant_agent_type, registrant_agent_id,
                 node_name, tos_name, data_exchange_type,
                 buffer_size=10, drop_new=True, **kwargs):
        class_ = Importer.get_message_type_class(data_exchange_type)
        super().__init__(registrant_agent_type, registrant_agent_id,
                         node_name, InterfaceSubscriberNode.NODE_TYPE, tos_name,
                         data_exchange_type, buffer_size, drop_new)
        class_ = Importer.get_message_type_class(data_exchange_type)
        self.class_ = class_
        self._rosagent_config = RosAgentConfiguration()
        self.subscription = self.create_subscription(class_,
                                                     tos_name,
                                                     self._send,
                                                     buffer_size)
        self.subscription # Prevent unused variable warning (according to docs)

    def get_topic_name(self):
        return self.get_tos_name()

    def has_callback_group(self):
        return True

    def _send(self, message):
        esm = self._create_esm(message)
        self._send_esm(esm)

    def _create_esm(self, message):
        routing_info = self._create_routing_info()
        metadata = self._create_esm_metadata(
            message_manager=InterfaceSubscriberNode._message_manager)
        data = self._serialize_ros_message(message)
        esm = EgressSystemMessage(routing_info, metadata, data)
        return esm

    def _create_routing_info(self):
        agent_type = self._rosagent_config.get_agent_type()
        routing_data = SystemMessageRoutingData(agent_type=agent_type)
        management_data = self._create_routing_management_data()
        routing_info = SystemMessageRoutingInfo(routing_data, management_data)
        return routing_info

    def _create_routing_management_data(self):
        management_data = {
            "messageManager": InterfaceSubscriberNode._message_manager,
            "destAgents": self._registrant_agents
        }
        return management_data
