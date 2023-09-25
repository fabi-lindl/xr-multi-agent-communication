from rclpy.serialization import serialize_message

from rosagent.buffers import egress_buffer as EgressBuffer
from rosagent.config.ros_agent_configuration import RosAgentConfiguration
from rosagent.interface.interface_node import InterfaceNode
from rosagent.message.egress_system_message_buffer import EgressSystemMessageBuffer

from rosutility.message.system_message_metadata import SystemMessageMetadata

class InterfaceSenderNode(InterfaceNode):

    _sender_agent_type = RosAgentConfiguration().get_agent_type()
    _sender_agent_id = RosAgentConfiguration().get_agent_id()

    def __init__(self, registrant_agent_type, registrant_agent_id,
                 node_name, node_type, tos_name, data_exchange_type,
                 buffer_size=10, drop_new=True):
        super().__init__(registrant_agent_type, registrant_agent_id,
                         node_name, node_type, tos_name, data_exchange_type)
        self._esm_buffer = EgressSystemMessageBuffer(buffer_size, drop_new)

    def _send_esm(self, message):
        success = self._esm_buffer.enqueue(message)
        if success:
            EgressBuffer.enqueue(self._esm_buffer)

    def _create_esm_metadata(self, request_id=0, message_manager=""):
        args = {
            "sender_agent_type": InterfaceSenderNode._sender_agent_type,
            "sender_agent_id": InterfaceSenderNode._sender_agent_id,
            "request_id": request_id,
            "message_manager": message_manager,
            "node_type": self.get_type(),
            "node_name": self.get_name(),
            "data_exchange_type": self.get_data_exchange_type()
        }
        metadata = SystemMessageMetadata(**args)
        return metadata
    
    def _serialize_ros_message(self, message):
        return serialize_message(message)
