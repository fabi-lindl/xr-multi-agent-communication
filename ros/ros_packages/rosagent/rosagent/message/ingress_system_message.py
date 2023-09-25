class IngressSystemMessage:

    def __init__(self, metadata, data):
        self._metadata = metadata
        self._data = data

    def get_metadata(self):
        return self._metadata

    def get_data(self):
        return self._data

    def get_node_type(self):
        return self._metadata.get_node_type()
    
    def get_node_name(self):
        return self._metadata.get_node_name()

    def get_request_id(self):
        return self._metadata.get_request_id()

    def get_message_manager(self):
        return self._metadata.get_message_manager()

    def is_management_message(self):
        return self._metadata.has_message_manager()

    def is_interface_node_message(self):
        return self._metadata.has_node_name()
