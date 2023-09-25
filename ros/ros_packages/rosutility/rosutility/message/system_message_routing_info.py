from rosutility.message.serializer import serializer as Serializer

class SystemMessageRoutingInfo:

    def __init__(self, routing_data, management_data={}):
        self._routing_data = routing_data
        self._management_data = management_data

    def get_routing_data(self):
        return self._routing_data
    
    def get_management_data(self):
        return self._management_data

    def get_message_manager(self):
        try:
            return self._management_data["message_manager"]
        except:
            return None

    def get_agent_type(self):
        return self._routing_data.get_agent_type()

    def get_destination_agent_ids(self):
        return self._routing_data.get_destination_agent_ids()

    def get_destination_group_ids(self):
        return self._routing_data.get_destination_group_ids()

    def get_management_data_size(self):
        return self._routing_data.get_management_data_size()

    def get_total_message_size(self):
        return self._routing_data.get_total_message_size()

    def set_total_message_size(self, size):
        self._routing_data.set_total_message_size(size)
    
    def has_management_data(self):
        return self._management_data != {}

    def is_broadcast_message(self):
        return self.get_agent_type() == 0

    def serialize(self):
        s_management_data = self._serialize_management_data()
        s_routing_data = self._serialize_routing_data(s_management_data)
        return s_routing_data + s_management_data

    def _serialize_management_data(self):
        if self._management_data == {}:
            return b''
        return Serializer.dict_to_bytes(self._management_data)

    def _serialize_routing_data(self, serialized_management_data):
        size = len(serialized_management_data)
        self._routing_data.set_management_data_size(size)
        s_routing_data = self._routing_data.serialize()
        return s_routing_data
    