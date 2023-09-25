from rosutility.message.serializer import serializer as Serializer
from rosutility.message.serializer import variable_format_serializer as VariableFormatSerializer

class SystemMessageMetadata:

    _message_keys = [
        "senderAgentType",
        "senderAgentId",
        "requestId",
        "messageManager",
        "nodeType",
        "nodeName",
        "dataExchangeType"
    ]

    def __init__(self, sender_agent_type=None, sender_agent_id=None,
                 request_id=0, message_manager="", node_type=0, node_name="",
                 data_exchange_type=""): 
        self._sender_agent_type = sender_agent_type
        self._sender_agent_id = sender_agent_id
        self._request_id = request_id
        self._message_manager = message_manager
        self._node_type = node_type
        self._node_name = node_name
        self._data_exchange_type = data_exchange_type
    
    def init_from_received_message_metadata(self, metadata):
        for key, value in metadata.items():
            attr_name = VariableFormatSerializer.camel_case_str_to_attr_name(key)
            setattr(self, attr_name, value)

    def get_request_id(self):
        return self._request_id

    def get_message_manager(self):
        return self._message_manager

    def get_node_type(self):
        return self._node_type

    def get_node_name(self):
        return self._node_name

    def get_exchange_data_type(self):
        return self._data_exchange_type

    def get_sender_agent_type(self):
        return self._sender_agent_type

    def get_sender_agent_id(self):
        return self._sender_agent_id

    def has_message_manager(self):
        return self._message_manager != ""

    def has_node_name(self):
        return self._node_name != ""

    def serialize(self):
        metadata = {}
        instance_attrs = self.__dict__
        for message_key in SystemMessageMetadata._message_keys:
            attr_name = (VariableFormatSerializer
                         .camel_case_str_to_attr_name(message_key))
            attr_value = instance_attrs[attr_name]
            metadata[message_key] = attr_value
        metadata_bytes = Serializer.dict_to_bytes(metadata)
        return metadata_bytes
