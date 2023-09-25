from rosagent.management.management_message_manager import ManagementMessageManager

from rosutility.logging.logger import Logger

class InterfaceNodeRegistrar(ManagementMessageManager):

    def __init__(self):
        super().__init__()
        self._node_type_key = "node_type"
        self._node_name_key = "node_name"
        self._registrant_agent_type_key = "registrant_agent_type"
        self._registrant_agent_id_key = "registrant_agent_id"
        self._logger = Logger()

    def _get_node_type(self, management_data):
        if self._node_type_key in management_data:
            return management_data[self._node_type_key]
        self._handle_missing_node_type_error(management_data)
        return None

    def _handle_missing_node_type_error(self, management_data):
        node_name_info = self._write_node_name_info(management_data)
        log_message = (node_name_info + f"Key \"{self._node_type_key}\" is "
                       "missing in the management message.")
        self._logger.log_warning(log_message)

    def _get_node_name(self, management_data):
        if self._node_name_key in management_data:
            return management_data[self._node_name_key]
        self._handle_missing_node_name_error(management_data)
        return None

    def _handle_missing_node_name_error(self, management_data):
        node_name_info = self._write_node_name_info(management_data)
        log_message = (node_name_info + f"Key \"{self._node_name_key}\" is "
                       "missing in the management message.")
        self._logger.log_warning(log_message)

    def _write_node_name_info(self, management_data):
        try:
            node_name = management_data[self._node_name_key]
            return f"Node of name \"{node_name}\" could not be registered. "
        except KeyError:
            return "Node could not be registered. "
