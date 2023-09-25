from rosagent.interface import interface_nodes_registry as NodesRegistry
from rosagent.management.interface_node_registrar import InterfaceNodeRegistrar
from rosagent.operation.rosagent import RosAgent

class InterfaceNodeRemover(InterfaceNodeRegistrar):

    def process_ism(self, ism):
        management_data = self._deserialize_management_message_data(ism)
        node = self._get_node(management_data)
        if node is None: return
        self._handle_node_deregistration(node, management_data)

    def _get_node(self, management_data):
        node_type = self._get_node_type(management_data)
        if node_type is None: return None
        node_name = self._get_node_name(management_data)
        if node_name is None: return None
        node = NodesRegistry.get_node(node_type, node_name)
        if node is None:
            self._log_node_does_not_exist_error(node_type, node_name)
            return None
        return node

    def _log_node_does_not_exist_error(self, node_type, node_name):
        log_message = (f"Node of type \"{node_type}\" and name \"{node_name}\" "
                       "does not exist in the registry. It could thus not be "
                       "deregistered.")
        self._logger.log_warning(log_message)

    def _handle_node_deregistration(self, node, management_data):
        registrant_agent_type = management_data[self._registrant_agent_type_key]
        registrant_agent_id = management_data[self._registrant_agent_id_key]
        node.remove_registrant_agent(registrant_agent_type, registrant_agent_id)
        if node.has_registrant_agents():
            self._log_deregistered_agent_from_node(node, registrant_agent_id)
        else:
            self._deregister_node(node)

    def _log_deregistered_agent_from_node(self, node, registrar_agent_id):
        log_message = (f"Deregistered agent of id \"{registrar_agent_id}\" "
                       f"from node of type \"{node.get_type()}\" and name "
                       f"\"{node.get_name()}\".")
        self._logger.log_info(log_message)

    def _deregister_node(self, node):
        NodesRegistry.remove_node(node)
        RosAgent().remove_node_from_executor(node)
        self._log_deregistered_node(node)
        node.destroy_node()

    def _log_deregistered_node(self, node):
        log_message = (f"Deregistered {node.get_type_as_str()} node "
                       f"\"{node.get_name()}\".")
        self._logger.log_info(log_message)
