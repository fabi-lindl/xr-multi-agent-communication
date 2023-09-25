from threading import Condition

class RegistryGroup:

    def __init__(self, agentclient):
        self._cv = Condition()
        self._id = agentclient.get_group_id()
        self._group = {}
        self.add(agentclient)

    def get_id(self):
        return self._id
            
    def get_agentclients_of_type(self, type_):
        if self.has_type(type_):
            return self._group[type_].copy() # Make a copy for thread-safety
        return None

    def has_type(self, type_):
        return type_ in self._group

    def add(self, agentclient):
        with self._cv:
            type_ = agentclient.get_agent_type()
            if self.has_type(type_):
                    self._group[type_].append(agentclient)
            else:
                self._group[type_] = [agentclient]

    def remove(self, agentclient):
        with self._cv:
            type_ = agentclient.get_agent_type()
            if self.has_type(type_):
                category = self.get_agentclients_of_type(type_)
                category.remove(agentclient)

    def send(self, message, type_):
        with self._cv:
            if self._is_sending_to_all(type_):
                self._send_to_all_agentclients(message)
            else:
                self._send_to_agentclients_of_type(message, type_)

    def _send_to_all_agentclients(self, message):
        for category in self._group.values():
            for agentclient in category:
                agentclient.send(message)

    def _send_to_agentclients_of_type(self, message, type_):
        if self.has_type(type_):
            agentclients = self.get_agentclients_of_type(type_)
            for agentclient in agentclients:
                agentclient.send(message)

    def _is_sending_to_all(self, type_):
        return type_ == 0
