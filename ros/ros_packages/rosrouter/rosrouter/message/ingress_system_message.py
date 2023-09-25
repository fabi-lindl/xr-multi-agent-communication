class IngressSystemMessage:

    def __init__(self, routing_info, message: bytes):
        self._routing_info = routing_info
        self._message = message

    def get_routing_info(self):
        return self._routing_info
    
    def get_message(self):
        return self._message
    
    def get_management_data(self):
        return self._routing_info.get_management_data()

    def get_message_manager(self):
        return self._routing_info.get_message_manager()

    def is_management_message(self):
        return self._routing_info.has_management_data()
    
    def is_broadcast_message(self):
        return self._routing_info.is_broadcast_message()
    