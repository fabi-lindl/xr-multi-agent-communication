from rosrouter.agentclient.registration.agentclient_registrar import AgentclientRegistrar
from rosrouter.agentclient.registry import agentclient_registry as Registry
from rosrouter.agentclient.registry.agentclient_connection_surveillancer import AgentclientConnectionSurveillancer

class AgentclientRegistryManager:

    _instance = None

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(AgentclientRegistryManager, cls).__new__(cls)
            cls._instance._initialized = False
        return cls._instance
    
    def __init__(self):
        if self._initialized: return
        self._initialized = True
        self._conn_surveillancer = AgentclientConnectionSurveillancer()
        self._conn_surveillancer.start()
        self._agentclient_registrar = AgentclientRegistrar()

    def register_agentclient_for_conn(self, socket, address):
        self._agentclient_registrar.register_agentclient(socket, address)
    
    def clear_registry(self):
        Registry.clear()
