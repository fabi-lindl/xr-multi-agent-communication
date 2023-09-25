from rosagent.interface.interface_subscriber_node import InterfaceSubscriberNode
from rosagent.interface.interface_publisher_node import InterfacePublisherNode
from rosagent.interface.interface_client_node import InterfaceClientNode
from rosagent.interface.interface_service_node import InterfaceServiceNode

id_to_class_map = {
    1: InterfaceSubscriberNode,
    2: InterfacePublisherNode,
    3: InterfaceServiceNode,
    4: InterfaceClientNode
}

id_to_str_map = {
    1: 'subscriber',
    2: 'publisher',
    3: 'service',
    4: 'client'
}

str_to_id_map = {value: key for key, value in id_to_str_map.items()}

def id_to_class(node_type: int):
    if node_type in id_to_class_map:
        return id_to_class_map[node_type]
    return None

def id_to_str(node_type: int):
    return id_to_str_map[node_type]

def str_to_id(node_type: str):
    return str_to_id_map[node_type]
