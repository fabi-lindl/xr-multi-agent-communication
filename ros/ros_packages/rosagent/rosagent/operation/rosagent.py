import time
import socket

from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from rosagent.config.ros_agent_configuration import RosAgentConfiguration
from rosagent.interface import interface_nodes_registry as NodesRegistry
from rosagent.operation.egress_system_message_sender import EgressSystemMessageSender
from rosagent.operation.ingress_system_message_receiver import IngressSystemMessageReceiver
from rosagent.operation.ingress_system_message_manager import IngressSystemMessageManager

from rosutility.logging.connection_logger import ConnectionLogger

class RosAgent(Node):

    _instance = None
    _max_num_connection_attempts = 3
    _retry_period = 5 # seconds

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(RosAgent, cls).__new__(cls)
            cls._instance._initialized = False
        return cls._instance

    def __init__(self):
        if self._initialized:
            return
        self._initialized = True
        super().__init__(node_name="rosagent")
        self._configuration = self._configure_ros_agent()
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._esm_sender = EgressSystemMessageSender(self._socket)
        self._ism_receiver = IngressSystemMessageReceiver(self._socket)
        self._ism_manager = IngressSystemMessageManager()
        self._logger = ConnectionLogger()
        self._executor = MultiThreadedExecutor(1)

    def add_nodes_to_executor(self, nodes):
        for node in nodes:
            self.add_node_to_exector(node)

    def add_node_to_executor(self, node):
        if node.has_callback_group():
            self._executor.add_node(node)

    def remove_nodes_from_executor(self, nodes):
        for node in nodes:
            self.remove_node_from_executor(node)

    def remove_node_from_executor(self, node):
        if node.has_callback_group():
            self._executor.remove_node(node)

    def shutdown(self):
        NodesRegistry.clear()
        self.destroy_node()

    def run(self):
        self._connect()
        self._start_workers()
        self._spin_executor()

    def _connect(self):
        num_trials = 0
        while num_trials < RosAgent._max_num_connection_attempts:
            try:
                self._try_to_connect()
                break
            except Exception as e:
                self._handle_connect_failure(str(e))
                num_trials += 1

    def _try_to_connect(self):
        address = self._configuration.get_router_address()
        self._socket.connect(address)
        self._logger.log_rosagent_connected_to_router(*address)

    def _handle_connect_failure(self, exception: str):
        address = self._configuration.get_router_address()
        self._logger.log_rosagent_connecting_to_router_failed(*address)
        self._logger.log_info(exception)
        self._logger.log_next_retry(RosAgent._retry_period)
        time.sleep(RosAgent._retry_period)

    def _start_workers(self):
        self._logger.log_start_workers()
        self._start_esm_sender()
        self._start_ism_receiver()
        self._start_ism_manager()

    def _start_esm_sender(self):
        self._esm_sender.start()

    def _start_ism_receiver(self):
        self._ism_receiver.start()

    def _start_ism_manager(self):
        self._ism_manager.start()

    def _spin_executor(self):
        self._executor.spin()

    def _configure_ros_agent(self):
        self._declare_ros_parameters()
        router_ip_address = (self.get_parameter("router_ip_address")
                             .get_parameter_value().string_value)
        router_port = (self.get_parameter("router_port")
                       .get_parameter_value().integer_value)
        group_id = (self.get_parameter("group_id")
                    .get_parameter_value().integer_value)
        configuration = RosAgentConfiguration(router_ip_address, router_port,
                                              group_id)
        return configuration

    def _declare_ros_parameters(self):
        self.declare_parameter("router_ip_address", "127.0.0.1")
        self.declare_parameter("router_port", 7000)
        self.declare_parameter("group_id", 0)
