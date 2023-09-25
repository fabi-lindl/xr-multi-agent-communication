from rosutility.message.serializer import serializer as Serializer

class EgressSystemMessage:

    def __init__(self, routing_info, metadata=None, data:bytes=None):
        self._routing_info = routing_info
        self._metadata = metadata
        self._data = bytes() if data is None else data

    def serialize(self):
        s_metadata = (bytes() if self._metadata is None
                      else self._metadata.serialize())

        metadata_length = len(s_metadata)
        s_metadata_length = Serializer.uint_to_bytes(metadata_length)

        data_length = len(self._data)
        s_data_length = Serializer.uint_to_bytes(data_length)
        
        total_message_size = (len(s_metadata_length) + metadata_length +
                              len(s_data_length) + data_length)
        s_routing_info = self._serialize_routing_info(total_message_size)

        return (s_routing_info + s_metadata_length + s_metadata + 
                s_data_length + self._data)

    def _serialize_routing_info(self, total_message_size):
        self._routing_info.set_total_message_size(total_message_size)
        return self._routing_info.serialize()
