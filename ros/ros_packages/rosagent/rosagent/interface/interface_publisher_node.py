from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from rclpy.serialization import deserialize_message

from rosagent.interface.interface_node import InterfaceNode
from rosagent.util import data_exchange_type_module_importer as Importer

class InterfacePublisherNode(InterfaceNode):

    NODE_TYPE = 2

    def __new__(cls, data_exchange_type, *args, **kwargs):
        class_ = Importer.get_message_type_class(data_exchange_type)
        if class_ is None: return None
        return super().__new__(cls)

    def __init__(self, registrant_agent_type, registrant_agent_id,
                 node_name, tos_name, data_exchange_type,
                 buffer_size=10, is_latched=False, **kwargs):
        super().__init__(registrant_agent_type, registrant_agent_id,
                         node_name, InterfacePublisherNode.NODE_TYPE, tos_name,
                         data_exchange_type)
        class_ = Importer.get_message_type_class(data_exchange_type)
        args = self.create_args(class_, tos_name, buffer_size, is_latched)
        self._publisher = self.create_publisher(*args)
        self._message_type = class_

    def has_callback_group(self):
        return False

    def create_args(self, message_class, topic_name, buffer_size, is_latched):
        args = [message_class, topic_name]
        if is_latched:
            self.add_latcher_arg(args, buffer_size)
        else:
            args.append(buffer_size)
        return args

    def add_latcher_arg(args, depth):
        durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        latched_qos_profile = QoSProfile(depth=depth, durability=durability)
        args.append(latched_qos_profile)

    def get_topic_name(self):
        return self.get_tos_name()

    def process_ism(self, ism):
        message_bytes = ism.get_data()
        message = deserialize_message(message_bytes, self._message_type)
        self._publisher.publish(message)
