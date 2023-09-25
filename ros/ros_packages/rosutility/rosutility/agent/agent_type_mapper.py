id_to_str_map = {
    1: 'ros', # TODO
    2: 'unity'
}

str_to_id_map = {value: key for key, value in id_to_str_map.items()}

def id_to_str(sender_agent_type: int):
    return id_to_str_map[sender_agent_type]

def str_to_id(sender_agent_type: str):
    return str_to_id_map[sender_agent_type]
