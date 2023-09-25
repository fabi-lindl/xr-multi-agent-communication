from rosrouter.agentclient.registry.group_registry import GroupRegistry

registry = GroupRegistry()

def send_to_group(message, agentclient_type, group_id):
    if registry.has(group_id):
        group = registry.get(group_id)
        group.send(message, agentclient_type)

def send_to_groups(message, agentclient_type, group_ids):
    for group_id in group_ids:
        send_to_group(message, agentclient_type, group_id)
