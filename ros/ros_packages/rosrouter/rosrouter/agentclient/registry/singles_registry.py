from rosrouter.agentclient.registration.agentclient_id_pool import AgentclientIdPool

from rosutility.config.params.getter import agent_param_getter as AgentParamGetter

class SinglesRegistry:

    _instance = None

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(SinglesRegistry, cls).__new__(cls)
            cls._instance._initialized = False
        return cls._instance

    def __init__(self):
        if self._initialized: return
        self._initialized = True
        self._agentclients = self._create_agentclient_storage()
        self._agentclient_id_pool = AgentclientIdPool()

    def get_types(self):
        return self._agentclients.keys()
    
    def get(self, type_, id_):
        try:
            return self._agentclients[type_][id_]
        except:
            return None

    def get_agentclients_of_type(self, type_):
        if self.has_type(type_):
            return list(self._agentclients[type_].values())
        return None

    def get_agentclient_ids_of_type(self, type_):
        if self.has_type(type_):
            return list(self._agentclients[type_].keys())
        return None

    def has(self, agentclient):
        type_ = agentclient.get_agent_type()
        id_ = agentclient.get_agent_id()
        try:
            self._agentclients[type_][id_]
            return True
        except:
            return False

    def has_type(self, type_):
        return type_ in self._agentclients

    def add(self, agentclient):
        type_ = agentclient.get_agent_type()
        id_ = agentclient.get_agent_id()
        if self.has_type(type_):
            self._agentclients[type_][id_] = agentclient

    def remove(self, agentclient):
        type_ = agentclient.get_agent_type()
        id_ = agentclient.get_agent_id()
        try:
            del self._agentclients[type_][id_]
            self._agentclient_id_pool.enqueue(id_)
        except:
            pass
    
    def clear(self):
        self._agentclients = self._create_agentclient_storage()

    def _create_agentclient_storage(self):
        agentclients = {}
        agent_types = AgentParamGetter.get_agent_types()
        for agent_type in agent_types:
            agentclients[agent_type] = {}
        return agentclients
