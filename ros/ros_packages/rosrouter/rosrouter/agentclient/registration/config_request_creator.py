from threading import Lock

from rosrouter.agentclient.registration.agentclient_id_pool import AgentclientIdPool
from rosrouter.agentclient.registration.config_request import ConfigRequest
from rosrouter.agentclient.workers.agentclient_connection_manager import AgentclientConnectionManager

class ConfigRequestCreator:

    _instance = None

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(ConfigRequestCreator, cls).__new__(cls)
            cls._instance._initialized = False
        return cls._instance

    def __init__(self):
        if self._initialized: return
        self._initialized = True
        self._next_agentclient_id = 1
        self._next_agentclient_id_lock = Lock()
        self._next_request_id = 1
        self._next_request_id_lock = Lock()
        self._agentclient_id_pool = AgentclientIdPool()

    def create_configuration_request(self, socket, address):
        request_id = self._get_request_id()
        agentclient_id = self._get_agentclient_id()
        conn_manager = AgentclientConnectionManager(socket, address)
        config_request = ConfigRequest(request_id, agentclient_id,
                                       conn_manager)
        return config_request
    
    def _get_request_id(self):
        with self._next_request_id_lock:
            request_id = self._next_request_id
            self._increment_next_request_id()
        return request_id

    def _increment_next_request_id(self):
        self._next_request_id += 1

    def _get_agentclient_id(self):
        request_id = self._agentclient_id_pool.dequeue()
        if request_id is not None:
            return request_id
        with self._next_agentclient_id_lock:
            agentclient_id = self._next_agentclient_id
            self._increment_next_agentclient_id()
        return agentclient_id

    def _increment_next_agentclient_id(self):
        self._next_agentclient_id += 1
        if self._next_agentclient_id == 0:
            self._next_agentclient_id += 1
