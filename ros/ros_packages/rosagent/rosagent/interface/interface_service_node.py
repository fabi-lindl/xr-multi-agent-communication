from functools import partial

from rclpy.serialization import deserialize_message

from rosagent.config.ros_agent_configuration import RosAgentConfiguration
from rosagent.interface.interface_sender_node import InterfaceSenderNode
from rosagent.message.egress_system_message import EgressSystemMessage
from rosagent.util import data_exchange_type_module_importer as Importer

from rosutility.logging.logger import Logger
from rosutility.message.system_message_routing_data import SystemMessageRoutingData
from rosutility.message.system_message_routing_info import SystemMessageRoutingInfo

class InterfaceServiceNode(InterfaceSenderNode):

    NODE_TYPE = 3

    def __new__(cls, data_exchange_type, *args, **kwargs):
        class_ = Importer.get_service_type_class(data_exchange_type)
        if class_ is None: return None
        return super().__new__(cls)

    def __init__(self, registrant_agent_type, registrant_agent_id,
                 node_name, tos_name, data_exchange_type,
                 buffer_size=10, drop_new=True, **kwargs):
        super().__init__(registrant_agent_type, registrant_agent_id,
                         node_name, InterfaceServiceNode.NODE_TYPE, tos_name,
                         data_exchange_type, buffer_size, drop_new)
        class_ = Importer.get_service_type_class(data_exchange_type)
        self._client = self.create_client(class_, tos_name)
        self._req_message_type = type(class_.Request())
        self._rosagent_config = RosAgentConfiguration()

    def get_service_name(self):
        return self.get_tos_name()

    def has_callback_group(self):
        return True

    def process_ism(self, ism):
        message_bytes = ism.get_data()
        message = deserialize_message(message_bytes, self._req_message_type)
        response_cb = self.create_response_callback(ism)
        future = self._client.call_async(message)
        future.add_done_callback(response_cb)

    def create_response_callback(self, ism):
        metadata = ism.get_metadata()
        callback = partial(self.handle_response, metadata)
        return callback

    def handle_response(self, ism_metadata, future):
        try:
            self.send_response(ism_metadata, future)
        except Exception as e:
            self.log_response_exception(e)

    def send_response(self, ism_metadata, future):
        response = future.result()
        esm = self._create_egress_system_message(ism_metadata, response)
        self._send_esm(esm)

    def _create_egress_system_message(self, ism_metadata, message):
        request_id = ism_metadata.get_request_id()
        routing_info = self._create_routing_info(ism_metadata)
        metadata = self._create_esm_metadata(request_id=request_id)
        data = self._serialize_ros_message(message)
        esm = EgressSystemMessage(routing_info, metadata, data)
        return esm

    def _create_routing_info(self, ism_metadata):
        agent_type = ism_metadata.get_sender_agent_type()
        destination_agents = [ism_metadata.get_sender_agent_id()]
        routing_data = SystemMessageRoutingData(
            agent_type=agent_type, destination_agents=destination_agents)
        routing_info = SystemMessageRoutingInfo(routing_data=routing_data)
        return routing_info

    def log_response_exception(self, exception):
        logger = Logger()
        node_name = self.get_node_name()
        service_name = self.get_service_name()
        error_message = (f"Service call of node {node_name} to service" +
                         f"{service_name} failed: {exception}")
        logger.log_error(error_message)
