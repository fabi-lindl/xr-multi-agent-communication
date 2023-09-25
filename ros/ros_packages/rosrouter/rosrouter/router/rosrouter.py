import socket

from rclpy.node import Node

from rosrouter.agentclient.registry.agent_client_registry_manager import AgentclientRegistryManager

from rosutility.logging.connection_logger import ConnectionLogger

class RosRouter(Node):

    _instance = None

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(RosRouter, cls).__new__(cls)
            cls._instance._initialized = False
        return cls._instance

    def __init__(self):
        if self._initialized: return
        self._initialized = True
        super().__init__(node_name="rosrouter")
        self._declare_ros_parameters()
        self._num_allowed_unaccepted_connections = 10
        self._registry_manager = AgentclientRegistryManager()
        self._logger = ConnectionLogger()

    def run(self):
        serversocket = self._start_server()
        self._run_server(serversocket)

    def _start_server(self):
        serversocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._make_socket_available_in_wait_state(serversocket)
        self._bind_socket_to_address(serversocket)
        return serversocket
    
    def _run_server(self, serversocket):
        self._log_successful_startup()
        while True:
            serversocket.listen(self._num_allowed_unaccepted_connections)
            (conn, address) = serversocket.accept()
            self._logger.log_server_new_connection_accepted(*address)
            self._registry_manager.register_agentclient_for_conn(conn, address)

    def _make_socket_available_in_wait_state(self, serversocket):
        serversocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    def _bind_socket_to_address(self, serversocket):
        host = self._get_host()
        port = self._get_port()
        address = (host, port)
        serversocket.bind(address)

    def _log_successful_startup(self):
        host = self._get_host()
        port = self._get_port()
        self._logger.log_server_is_listening(host, port)

    def _declare_ros_parameters(self):
        self.declare_parameter("ip_address", "127.0.0.1")
        self.declare_parameter("port", 7000)

    def _get_host(self):
        return (self.get_parameter("ip_address")
                .get_parameter_value().string_value)

    def _get_port(self):
        return self.get_parameter("port").get_parameter_value().integer_value
