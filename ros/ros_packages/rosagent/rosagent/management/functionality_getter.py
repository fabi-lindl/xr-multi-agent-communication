from rclpy.node import Node

from rosagent.interface import interface_nodes_registry as NodesRegistry
from rosagent.management.management_message_manager import ManagementMessageManager
from rosagent.message.egress_system_message import EgressSystemMessage
from rosagent.config.ros_agent_configuration import RosAgentConfiguration

from rosutility.message.system_message_metadata import SystemMessageMetadata
from rosutility.message.system_message_routing_data import SystemMessageRoutingData
from rosutility.message.system_message_routing_info import SystemMessageRoutingInfo

class FunctionalityGetter(ManagementMessageManager):

    SUBSCRIBERS = "subscribers"
    PUBLISHERS = "publishers"
    SERVICES = "services"

    def __init__(self):
        super().__init__()
        self._node = Node("FunctionalityGetter")
        self._functionality = {
            FunctionalityGetter.SUBSCRIBERS: {},
            FunctionalityGetter.PUBLISHERS: {},
            FunctionalityGetter.SERVICES: {}
        }
        self._rosagent_config = RosAgentConfiguration()

    def process_ism(self, ism):
        esm = self._create_esm()
        self._send_esm(esm)
    
    def _create_esm(self):
        routing_info = self._create_routing_info()
        metadata = self._create_metadata()
        data = bytes()
        esm = EgressSystemMessage(routing_info, metadata, data)
        return esm

    def _create_routing_info(self):
        agent_type = self._rosagent_config.get_agent_type()
        routing_data = SystemMessageRoutingData(agent_type=agent_type)
        data = self.get_functionality()
        routing_info = SystemMessageRoutingInfo(routing_data, data)
        return routing_info

    def _create_metadata(self):
        message_manager = "RosAgentFunctionalityConfigurer"
        agent_type = self._rosagent_config.get_agent_type()
        agent_id = self._rosagent_config.get_agent_id()
        metadata = SystemMessageMetadata(sender_agent_type=agent_type,
                                         sender_agent_id=agent_id,
                                         message_manager=message_manager)
        return metadata

    def get_functionality(self):
        nodes_id = self._discover_non_interface_nodes()
        for node_id in nodes_id:
            self._add_subscriber_functionality(node_id)
            self._add_publisher_functionality(node_id)
            self._add_service_functionality(node_id)
        return self._functionality

    def _discover_non_interface_nodes(self):
        nodes_id = self._node.get_node_names_and_namespaces()
        non_interface_nodes = filter(self._is_non_interface_node, nodes_id)
        return non_interface_nodes
    
    def _is_non_interface_node(self, node_id: tuple):
        node_name = node_id[0]
        is_interface_node = NodesRegistry.has_node_of_name(node_name)
        return not is_interface_node
    
    def _add_subscriber_functionality(self, node_id: tuple):
        topic_ids = self._node.get_subscriber_names_and_types_by_node(*node_id)
        self._add_functionality(topic_ids, FunctionalityGetter.SUBSCRIBERS)

    def _add_publisher_functionality(self, node_id: tuple):
        topic_ids = self._node.get_publisher_names_and_types_by_node(*node_id)
        self._add_functionality(topic_ids, FunctionalityGetter.PUBLISHERS)

    def _add_service_functionality(self, node_id: tuple):
        service_ids = self._node.get_service_names_and_types_by_node(*node_id)
        self._add_functionality(service_ids, FunctionalityGetter.SERVICES)

    def _add_functionality(self, tos_ids: tuple, functionality_type: str):
        for tos_id in tos_ids:
            name = tos_id[0] # topic or service name
            type_ = tos_id[1][0] # message or service type
            self._functionality[functionality_type][name] = type_
