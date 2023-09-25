from threading import Condition

from rosrouter.agentclient.registry.registry_group import RegistryGroup

class GroupRegistry:

    _instance = None

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(GroupRegistry, cls).__new__(cls)
            cls._instance._initialized = False
        return cls._instance

    def __init__(self):
        if self._initialized: return
        self._initialized = True
        self._groups = {}
        self._cv = Condition()
    
    def get(self, group_id):
        try:
            return self._groups[group_id]
        except KeyError:
            return None

    def has(self, group_id):
        return group_id in self._groups

    def add(self, agentclient):
        if not agentclient.belongs_to_group(): return
        with self._cv:
            group_id = agentclient.get_group_id()
            if self.has(group_id):
                self._add_agentclient_to_group(agentclient)
            else:
                self._create_new_group(group_id, agentclient)

    def _add_agentclient_to_group(self, agentclient):
        group = self.get(agentclient.get_group_id())
        group.add(agentclient)

    def _create_new_group(self, group_id, agentclient):
        group = RegistryGroup(agentclient)
        self._groups[group_id] = group

    def remove(self, agentclient):
        if not agentclient.belongs_to_group(): return
        group_id = agentclient.get_group_id()
        group = self.get(group_id)
        if group is None: return
        group.remove(agentclient)

    def remove_group(self, group_id):
        try:
            del self._groups[group_id]
        except KeyError:
            pass

    def clear(self):
        with self._cv:
            self._groups.clear()
