from rosutility.logging.logger import Logger
from rosutility.agent import agent_type_mapper as AgentTypeMapper

class RosrouterLogger(Logger):

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(RosrouterLogger, cls).__new__(cls)
            cls._instance._initialized = False
        return cls._instance

    def __init__(self):
        if self._initialized:
            return
        super().__init__("rosrouter_logger")
        self._initialized = True

    def log_agentclient_created(self, agentclient):
        type_, id_ = self._get_agentclient_type_and_id(agentclient, True)
        message = (f"Created {type_} agentclient of id {id_}")
        self.log_info(message)

    def log_agentclient_removed_from_registry(self, agentclient):
        type_, id_ = self._get_agentclient_type_and_id(agentclient, True)
        message = f"Removed {type_} agentclient of id {id_} "
        if agentclient.belongs_to_group():
            group_id = agentclient.get_group_id()
            message += f"(group id: {group_id}) "
        message += "from the registry."
        self.log_warning(message)

    def log_agentclient_added_to_registry(self, agentclient):
        type_, id_ = self._get_agentclient_type_and_id(agentclient, True)
        message = f"Added {type_} agentclient of id {id_} to registry "
        if agentclient.belongs_to_group():
            group_id = agentclient.get_group_id()
            message += f"(belongs to group of id {group_id})."
        else:
            message += f"(does not belong to a group)."
        self.log_info(message)

    def _get_agentclient_type_and_id(self, agentclient, type_as_str=False):
        type_ = agentclient.get_agent_type()
        id_ = agentclient.get_agent_id()
        if type_as_str:
            type_str = AgentTypeMapper.id_to_str(type_)
            return type_str, id_
        return type_, id_
