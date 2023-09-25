_interface_nodes = {
    1: {},
    2: {},
    3: {},
    4: {}
}

def get_node(node_type: int, node_name):
    try:
        node = _interface_nodes[node_type][node_name]
        return node
    except:
        return None

def add_node(node):
    node_type = node.get_type()
    node_name = node.get_name()
    _interface_nodes[node_type][node_name] = node

def remove_nodes(nodes):
    for node in nodes:
        remove_node(node)

def remove_node(node):
    if has_node(node):
        node_type = node.get_type()
        node_name = node.get_name()
        del _interface_nodes[node_type][node_name]

def get_nodes_wo_registrants():
    nodes_wo_registrants = []
    for node_items in _interface_nodes.values():
        for node in node_items.values():
            if node.has_registrant_agents(): continue
            nodes_wo_registrants.append(node)
    return nodes_wo_registrants

def remove_registrant_agents_from_nodes(registrant_agents):
    for node_items in _interface_nodes.values():
        for node in node_items.values():
            node.remove_registrant_agents(registrant_agents)

def has_node(node):
    node_type = node.get_type()
    node_name = node.get_name()
    try:
        _interface_nodes[node_type][node_name]
        return True
    except:
        return False

def has_node_of_name(node_name):
    for nodes in _interface_nodes.values():
        for node in nodes:
            if node.get_name() == node_name:
                return True
    return False

def clear():
    for nodes in _interface_nodes.values():
        for node in nodes:
            node.destroy_node()
