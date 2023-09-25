from rosagent.interface import interface_nodes_registry as Registry
from rosagent.logging.rosagent_logger import RosagentLogger
from rosagent.management.management_message_manager import ManagementMessageManager
from rosagent.operation.rosagent import RosAgent

class PrunedUnityAgentsInterfaceNodesCleaner(ManagementMessageManager):

    def process_ism(self, ism):
        RosagentLogger().log_remove_unity_agents_from_interface_nodes()
        self._remove_registrant_agents_from_interface_nodes(ism)
        self._remove_interface_nodes_wo_registrants()

    def _remove_registrant_agents_from_interface_nodes(self, ism):
        management_data = self._deserialize_management_message_data(ism)
        agents = management_data["agents"]
        Registry.remove_registrant_agents_from_nodes(agents)

    def _remove_interface_nodes_wo_registrants(self):
        nodes_wo_registrants = Registry.get_nodes_wo_registrants()
        Registry.remove_nodes(nodes_wo_registrants)
        RosAgent().remove_nodes_from_executor(nodes_wo_registrants)
        for node in nodes_wo_registrants:
            node.destroy_node()
