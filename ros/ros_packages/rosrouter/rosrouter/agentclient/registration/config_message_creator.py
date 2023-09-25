from rosrouter.message.egress_system_message import EgressSystemMessage

from rosutility.message.system_message_metadata import SystemMessageMetadata
from rosutility.message.serializer import serializer as Serializer

class ConfigMessageCreator:

    _instance = None
    _message_manager = "AgentConfigurer"

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(ConfigMessageCreator, cls).__new__(cls)
            cls._instance._initialized = False
        return cls._instance

    def __init__(self):
        if self._initialized: return
        self._initialized = True

    def create_config_message(self, config_request) -> bytes:
        metadata = self._create_message_metadata(config_request)
        data = self._create_message_data(config_request)
        esm = EgressSystemMessage(metadata, data)
        esm_bytes = esm.serialize()
        return esm_bytes

    def _create_message_metadata(self, config_request):
        request_id = config_request.get_request_id()
        metadata = SystemMessageMetadata(
            sender_agent_type=None,
            sender_agent_id=None,
            request_id=request_id,
            message_manager=ConfigMessageCreator._message_manager
        )
        return metadata

    def _create_message_data(self, config_request):
        agentclient_id = config_request.get_agentclient_id()
        data = {"agentId": agentclient_id}
        data_bytes = Serializer.dict_to_bytes(data)
        return data_bytes
