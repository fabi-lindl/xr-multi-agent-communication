from rosrouter.agentclient.registry import group_registry_message_sender as GroupSender
from rosrouter.agentclient.registry import singles_registry_message_sender as SingleAgentSender

def send_ism(ism):
    if ism.is_broadcast_message():
        broadcast_ism(ism)
    else:
        send_ism_to_single_agents(ism)
        send_ism_to_agent_groups(ism)

def broadcast_ism(ism):
    message = ism.get_message()
    SingleAgentSender.send_to_all_agentclients(message)

def broadcast_message(message: bytes):
    SingleAgentSender.send_to_all_agentclients(message)

def send_ism_to_single_agents(ism):
    routing_info = ism.get_routing_info()
    agent_type = routing_info.get_agent_type()
    destination_agent_ids = routing_info.get_destination_agent_ids()
    message = ism.get_message()
    SingleAgentSender.send_to_agentclients(message, agent_type,
                                           destination_agent_ids)

def send_ism_to_agent_groups(ism):
    routing_info = ism.get_routing_info()
    agent_type = routing_info.get_agent_type()
    destination_group_ids = routing_info.get_destination_group_ids()
    message = ism.get_message()
    GroupSender.send_to_groups(message, agent_type, destination_group_ids)
