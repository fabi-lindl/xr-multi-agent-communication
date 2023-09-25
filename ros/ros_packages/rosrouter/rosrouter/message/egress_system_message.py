from rosutility.message.serializer import serializer as Serializer

class EgressSystemMessage:

    def __init__(self, metadata, data: bytes):
        self._metadata = metadata
        self._data = data

    def serialize(self):
        s_metadata = self._metadata.serialize()
        metadata_length = len(s_metadata)
        data_length = len(self._data)
        s_metadata_length = Serializer.uint_to_bytes(metadata_length)
        s_data_length = Serializer.uint_to_bytes(data_length)
        return s_metadata_length + s_metadata + s_data_length + self._data
    