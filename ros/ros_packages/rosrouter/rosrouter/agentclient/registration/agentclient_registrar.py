from rosrouter.agentclient.clients.ros_agentclient import RosAgentclient
from rosrouter.agentclient.clients.unity_agentclient import UnityAgentclient
from rosrouter.agentclient.registration.config_message_creator import ConfigMessageCreator
from rosrouter.agentclient.registration.config_request_creator import ConfigRequestCreator
from rosrouter.agentclient.registration.config_requests_surveillancer import ConfigRequestsSurveillancer
from rosrouter.agentclient.registry import agentclient_registry as Registry
from rosrouter.logging.rosrouter_logger import RosrouterLogger as Logger

class AgentclientRegistrar:

    _instance = None

    _agentclient_classes = [
        RosAgentclient,
        UnityAgentclient
    ]

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(AgentclientRegistrar, cls).__new__(cls)
            cls._instance._initialized = False
        return cls._instance

    def __init__(self):
        if self._initialized: return
        self._initialized = True
        self._pending_requests = {}
        self._surveillancer = ConfigRequestsSurveillancer(self._pending_requests)
        self._surveillancer.start()
        self._config_message_creator = ConfigMessageCreator()
        self._config_request_creator = ConfigRequestCreator()
        self._logger = Logger()

    def register_agentclient(self, socket, address):
        config_request = (self._config_request_creator
                          .create_configuration_request(socket, address))
        self._store_pending_config_requests(config_request)
        config_message = (self._config_message_creator
                          .create_config_message(config_request))
        self._start_conn_manager(config_request)
        self._send_config_message(config_message, config_request)

    def _store_pending_config_requests(self, config_request):
        request_id = config_request.get_request_id()
        self._pending_requests[request_id] = config_request
    
    def _start_conn_manager(self, config_request):
        conn_manager = config_request.get_conn_manager()
        conn_manager.run()

    def _send_config_message(self, config_message, config_request):
        conn_manager = config_request.get_conn_manager()
        conn_manager.send(config_message)
    
    def create_agentclient(self, received_config: dict):
        """
        This method is called by the concerned management class to handle
        the agentclient registration.
        """
        pending_request = self._pop_pending_request(received_config)
        conn_manager = pending_request.get_conn_manager()
        class_ = self._agentclient_class(received_config)
        agentclient = class_(**received_config, conn_manager=conn_manager)
        self._logger.log_agentclient_created(agentclient)
        Registry.add(agentclient)

    def _pop_pending_request(self, received_config: dict):
        request_id = received_config["request_id"]
        pending_request = self._pending_requests[request_id]
        del self._pending_requests[request_id]
        return pending_request

    def _agentclient_class(self, received_config: dict):
        agent_type = received_config["agent_type"]
        for agent_class in AgentclientRegistrar._agentclient_classes:
            if agent_type == agent_class.AGENT_TYPE:
                return agent_class
    