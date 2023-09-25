import struct

from rosutility.message.serializer import struct_format_serializer as StructFormatSerializer
from rosutility.message.system_message_routing_data import SystemMessageRoutingData
from rosutility.config.system_message.getter import routing_data_config_getter as RoutingDataConfig

class RoutingDataDeserializer:

    _instance = None

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(RoutingDataDeserializer, cls).__new__(cls)
            cls._instance._initialized = False
        return cls._instance

    def __init__(self):
        if self._initialized: return
        self._initialized = True
        self._routing_data_size = self._compute_routing_data_size()
    
    def get_routing_data_size(self):
        return self._routing_data_size

    def deserialize(self, rcvd_data: bytes):
        routing_data_config = {}
        num_bytes_read = 0
        for arg_name, chunk_info in RoutingDataConfig.config_data.items():
            bytes_ = self._read_bytes(rcvd_data, chunk_info, num_bytes_read)
            format_ = self._gen_format(**chunk_info)
            data = self._deserialize_data_by_format(bytes_, format_)
            arg = self._gen_arg(data, chunk_info)
            routing_data_config[arg_name] = arg
            num_bytes_read += self._chunk_size(chunk_info)
        routing_data = SystemMessageRoutingData(**routing_data_config)
        return routing_data

    def _read_bytes(self, routing_info, chunk_info, start_pos):
        chunk_size = self._chunk_size(chunk_info)
        end_pos = start_pos + chunk_size
        bytes_ = routing_info[start_pos:end_pos]
        return bytes_

    def _gen_format(self, num_items, item_size):
        format_char = StructFormatSerializer.num_bytes_to_format_char(item_size)
        format_ = f"<{num_items}{format_char}"
        return format_

    def _deserialize_data_by_format(self, value, format_):
        return struct.unpack(format_, value)
    
    def _gen_arg(self, data, chunk_info):
        num_items = chunk_info["num_items"]
        if num_items == 1:
            return data[0]
        return self._remove_pad_items(data)
    
    def _remove_pad_items(self, data):
        return [item for item in data if item != 0]

    def _compute_routing_data_size(self):
        routing_info_size = 0
        for chunk_info in RoutingDataConfig.config_data.values():
            chunk_size = self._chunk_size(chunk_info)
            routing_info_size += chunk_size
        return routing_info_size

    def _chunk_size(self, chunk_info):
        return chunk_info["num_items"] * chunk_info["item_size"]
    