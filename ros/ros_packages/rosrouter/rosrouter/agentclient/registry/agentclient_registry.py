from rosrouter.agentclient.registry.group_registry import GroupRegistry
from rosrouter.agentclient.registry.singles_registry import SinglesRegistry
from rosrouter.logging.rosrouter_logger import RosrouterLogger as Logger

singles_registry = SinglesRegistry()
group_registry = GroupRegistry()

def get(type_, id_):
    return singles_registry.get(type_, id_)

def get_agentclients_of_type(type_):
    return singles_registry.get_agentclients_of_type(type_)

def get_agentclient_ids_of_type(type_):
    return singles_registry.get_agentclient_ids_of_type(type_)

def get_agentclient_types():
    return singles_registry.get_types()

def get_group(group_id):
    return GroupRegistry.get(group_id)

def has(agentclient):
    return singles_registry.has(agentclient)

def add(agentclient):
    singles_registry.add(agentclient)
    group_registry.add(agentclient)
    Logger().log_agentclient_added_to_registry(agentclient)

def remove(agentclient):
    singles_registry.remove(agentclient)
    group_registry.remove(agentclient)
    Logger().log_agentclient_removed_from_registry(agentclient)

def clear():
    singles_registry.clear()
    group_registry.clear()
