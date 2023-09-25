from rosutility.message.serializer import deserializer as Deserializer

class ManagementMessageManager:
    
    _size_of_int = 4 # Bytes.

    def _read_metadata_from_ism(self, ism) -> dict:
        message = ism.get_message()
        metadata_length = self._read_length_identifier(message)
        metadata = self._read_metadata(message, metadata_length)
        return metadata

    def _read_length_identifier(self, message):
        bytes_ = message[0:ManagementMessageManager._size_of_int]
        length = Deserializer.bytes_to_uint(bytes_)
        return length

    def _read_metadata(self, message, metadata_length) -> dict:
        start = ManagementMessageManager._size_of_int
        end = start + metadata_length
        bytes_ = message[start:end]
        metadata = Deserializer.bytes_to_dict(bytes_, create_snake_case_keys=True)
        return metadata

    def delete_message_manager_item(self, data):
        del data["message_manager"]

    def delete_request_id_item(self, data):
        del data["request_id"]
