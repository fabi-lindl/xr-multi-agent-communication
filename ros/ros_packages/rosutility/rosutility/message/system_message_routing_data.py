from rosutility.message.serializer import serializer as Serializer
from rosutility.config.system_message.getter import routing_data_config_getter as RoutingDataConfig

class SystemMessageRoutingData:

    def __init__(self, agent_type=0, destination_agents=[],
                 destination_groups=[], management_data_size=None,
                 total_message_size=None):
        self._agent_type = agent_type
        self._destination_agents = destination_agents
        self._destination_groups = destination_groups
        self._management_data_size = management_data_size
        self._total_message_size = total_message_size

    def get_agent_type(self):
        return self._agent_type

    def get_destination_agent_ids(self):
        return self._destination_agents

    def get_destination_group_ids(self):
        return self._destination_groups

    def get_management_data_size(self):
        return self._management_data_size

    def set_management_data_size(self, size):
        self._management_data_size = size

    def get_total_message_size(self):
        return self._total_message_size

    def set_total_message_size(self, size):
        self._total_message_size = size

    def serialize(self):
        s_agent_type = self._serialize_agent_type()
        s_agents = self._serialize_destination_agents()
        s_groups = self._serialize_destination_groups()
        s_management_data_size = self._serialize_management_data_size()
        s_total_message_size = self._serialize_total_message_size()
        return (s_agent_type + s_agents + s_groups +
                s_management_data_size + s_total_message_size)

    def _serialize_agent_type(self):
        return Serializer.uint8_to_bytes(self._agent_type)

    def _serialize_destination_agents(self):
        item_size = RoutingDataConfig.get_destination_agent_id_size()
        num_items = RoutingDataConfig.get_num_destination_agents()
        return Serializer.uintx_iterable_to_bytes(self._destination_agents,
                                                  item_size, num_items)

    def _serialize_destination_groups(self):
        item_size = RoutingDataConfig.get_destination_group_id_size()
        num_items = RoutingDataConfig.get_num_destination_groups()
        return Serializer.uintx_iterable_to_bytes(self._destination_groups,
                                                  item_size, num_items)

    def _serialize_management_data_size(self):
        return Serializer.uint_to_bytes(self._management_data_size)

    def _serialize_total_message_size(self):
        return Serializer.uint_to_bytes(self._total_message_size)
