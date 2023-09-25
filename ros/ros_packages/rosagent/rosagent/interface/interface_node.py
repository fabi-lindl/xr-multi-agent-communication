from rclpy.node import Node

from rosagent.util import node_type_mapper as NodeTypeMapper

class InterfaceNode(Node):

    def __init__(self, registrant_agent_type, registrant_agent_id,
                 node_name, node_type, tos_name, data_exchange_type):
        super().__init__(node_name)
        self._registrant_agents = {
            registrant_agent_type: [registrant_agent_id]
        }
        self._node_type = node_type
        self._tos_name = tos_name # topic or service name
        self._data_exchange_type = data_exchange_type

    def get_type(self):
        return self._node_type
    
    def get_type_as_str(self):
        return NodeTypeMapper.id_to_str(self._node_type)
    
    def get_tos_name(self):
        return self._tos_name

    def get_data_exchange_type(self):
        return self._data_exchange_type
    
    def add_registrant_agent(self, agent_type: int, agent_id: int):
        if self.has_registrant_agent(agent_type, agent_id): return
        if agent_type in self._registrant_agents:
            self._registrant_agents[agent_type].append(agent_id)
        else:
            self._registrant_agents[agent_type] = [agent_id]

    def has_registrant_agent(self, agent_type: int, agent_id: int):
        if not agent_type in self._registrant_agents:
            return False
        return agent_id in self._registrant_agents[agent_type]

    def has_registrant_agents(self):
        for registrar_agents in self._registrant_agents.values():
            if len(registrar_agents) > 0:
                return True
        return False

    def remove_registrant_agents(self, registrant_agents: list):
        for registrant_agent in registrant_agents:
            self.remove_registrant_agent(**registrant_agent)

    def remove_registrant_agent(self, agent_type, agent_id):
        if self.has_registrant_agent(agent_type, agent_id):
            self._registrant_agents[agent_type].remove(agent_id)
