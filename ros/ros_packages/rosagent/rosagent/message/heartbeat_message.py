from rosagent.config.ros_agent_configuration import RosAgentConfiguration
from rosagent.message.egress_system_message import EgressSystemMessage

from rosutility.message.system_message_metadata import SystemMessageMetadata
from rosutility.message.system_message_routing_data import SystemMessageRoutingData
from rosutility.message.system_message_routing_info import SystemMessageRoutingInfo

class HeartbeatMessage:

    _instance = None
    _message_manager = "HeartbeatHandler"

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(HeartbeatMessage, cls).__new__(cls)
            cls._instance._initialized = False
        return cls._instance

    def __init__(self):
        if self._initialized:
            return
        self._initialized = True
        self._heartbeat_esm = self._heartbeat_esm()

    def get(self):
        return self._heartbeat_esm

    def _heartbeat_esm(self):
        routing_info = self._create_routing_info()
        metadata = self._create_metadata()
        data = bytes()
        esm = EgressSystemMessage(routing_info, metadata, data)
        return esm
    
    def _create_routing_info(self):
        routing_data = SystemMessageRoutingData(2)
        management_data = {
            "message_manager": HeartbeatMessage._message_manager
        }
        routing_info = SystemMessageRoutingInfo(routing_data, management_data)
        return routing_info
    
    def _create_metadata(self):
        ros_agent_configuration = RosAgentConfiguration()
        agent_type = ros_agent_configuration.get_agent_type()
        agent_id = ros_agent_configuration.get_agent_id()
        metadata = SystemMessageMetadata(
            sender_agent_type=agent_type,
            sender_agent_id=agent_id
        )
        return metadata
