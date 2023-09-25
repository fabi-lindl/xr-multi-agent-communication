from rosrouter.agentclient.registry.singles_registry import SinglesRegistry

registry = SinglesRegistry()

def send_to_agentclient(message: bytes, type_: int, id_: int):
    agentclient = registry.get(int(type_), int(id_))
    if agentclient != None:
        agentclient.send(message)

def send_to_agentclients(message: bytes, type_: int, ids):
    for id_ in ids:
        send_to_agentclient(message, type_, id_)

def send_to_all_agentclients(message):
    types = registry.get_types()
    for type_ in types:
        send_to_all_agentclients_of_type(message, type_)

def send_to_all_agentclients_of_type(message, type_):
    if registry.has_type(type_):
        agentclients = registry.get_agentclients_of_type(type_)
        for agentclient in agentclients:
            agentclient.send(message)
