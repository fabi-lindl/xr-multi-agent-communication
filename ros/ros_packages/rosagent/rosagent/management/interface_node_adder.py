from rosagent.interface import interface_nodes_registry as NodesRegistry
from rosagent.management.interface_node_registrar import InterfaceNodeRegistrar
from rosagent.operation.rosagent import RosAgent
from rosagent.util import node_type_mapper as NodeTypeMapper
from rosagent.interface.interface_client_node import InterfaceClientNode

class InterfaceNodeAdder(InterfaceNodeRegistrar):

    def process_ism(self, ism):
        management_data = self._deserialize_management_message_data(ism)
        node_type = self._get_node_type(management_data)
        if node_type is None: return
        node_class = NodeTypeMapper.id_to_class(node_type)
        if node_class is None:
            self._handle_node_type_error(management_data)
        else:
            self._add_node(node_class, management_data)

    def _handle_node_type_error(self, management_data):
        node_name_info = self._write_node_name_info(management_data)
        node_type = self._get_node_type(management_data)
        log_message = (node_name_info +
                       f"Node type \"{node_type}\" does not exist!")
        self._logger.log_warning(log_message)

    def _add_node(self, node_class, management_data):
        try:
            self._handle_node_registration(node_class, management_data)
        except Exception as e:
            self._handle_node_construction_error(management_data, e)

    def _handle_node_registration(self, node_class, management_data):
        node = node_class(**management_data)
        if node is None:
            raise ValueError
        if NodesRegistry.has_node(node):
            if self._node_allows_multiple_registrants(node):
                self._add_registrant_agent_id_to_existing_node(management_data)
        else:
            self._add_new_node(node, management_data)

    def _node_allows_multiple_registrants(self, node):
        return node.get_type() != InterfaceClientNode.NODE_TYPE

    def _handle_node_construction_error(self, management_data, exception):
        node_name_info = self._write_node_name_info(management_data)
        log_message = node_name_info + f"Exception is:\n{exception}"
        self._logger.log_warning(log_message)

    def _add_registrant_agent_id_to_existing_node(self, management_data):
        node_type = self._get_node_type(management_data)
        node_name = self._get_node_name(management_data)
        node = NodesRegistry.get_node(node_type, node_name)
        registrant_agent_type = management_data[self._registrant_agent_type_key]
        registrant_agent_id = management_data[self._registrant_agent_id_key]
        node.add_registrant_agent(registrant_agent_type, registrant_agent_id)
        self._log_node_registration_info(node, management_data)

    def _add_new_node(self, node, management_data):
        NodesRegistry.add_node(node)
        RosAgent().add_node_to_executor(node)
        self._log_new_node_created_info(node)
        self._log_node_registration_info(node, management_data)

    def _log_new_node_created_info(self, node):
        node_type = node.get_type_as_str()
        node_name = node.get_name()
        log_message = f"Registered new {node_type} node \"{node_name}\"."
        self._logger.log_info(log_message)

    def _log_node_registration_info(self, node, management_data):
        node_type = node.get_type_as_str()
        node_name = node.get_name()
        registrant_agent_id = management_data[self._registrant_agent_id_key]
        log_message = (f"Registered agent of id {registrant_agent_id} on "
                       f"{node_type} node \"{node_name}\".")
        self._logger.log_info(log_message)
