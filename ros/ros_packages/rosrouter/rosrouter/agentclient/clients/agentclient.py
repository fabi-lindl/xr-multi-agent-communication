from rclpy.node import Node

from rosutility.config.params.getter import agent_param_getter as AgentParamGetter
from rosutility.logging.connection_logger import ConnectionLogger

class Agentclient(Node):

    def __init__(self, agent_id, group_id, conn_manager, agent_type):
        super().__init__(node_name=f"agentclient_{agent_id}")
        self._agent_id = agent_id
        self._group_id = group_id
        self._conn_manager = conn_manager
        self._agent_type = agent_type

    def get_agent_type(self):
        return self._agent_type

    def get_agent_id(self):
        return self._agent_id
    
    def get_group_id(self):
        return self._group_id

    def get_address(self):
        return self._conn_manager.get_address()

    def get_ip_address(self):
        return self._conn_manager.get_ip_address()
    
    def get_port(self):
        return self._conn_manager.get_port()

    def is_ros_agent(self):
        return self._agent_type == AgentParamGetter.get_ros_agent_type()
    
    def is_unity_agent(self):
        return self._agent_type == AgentParamGetter.get_unity_agent_type()

    def belongs_to_group(self):
        return bool(self._group_id)

    def send(self, message: bytes):
        self._conn_manager.send(message)

    def is_connection_alive(self):
        return self._conn_manager.is_connection_alive()
    
    def close_connection(self):
        self._conn_manager.close_connection()
        conn_logger = ConnectionLogger()
        conn_logger.log_pruned_expired_agentclient_connection(
            self._agent_id, self.get_ip_address(), self.get_port())
