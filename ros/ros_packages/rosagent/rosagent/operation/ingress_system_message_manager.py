import importlib
from threading import Thread

from rosagent.buffers import ingress_buffer as IngressBuffer
from rosagent.interface import interface_nodes_registry as InterfaceNodesRegistry

from rosutility.message.serializer import variable_format_serializer as VariableFormatSerializer

class IngressSystemMessageManager(Thread):

    _instance = None

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(IngressSystemMessageManager, cls).__new__(cls)
            cls._instance._initialized = False
        return cls._instance

    def __init__(self):
        if self._initialized:
            return
        self._initialized = True
        Thread.__init__(self)
        self.daemon = True

    def run(self):
        while True:
            ism = self._dequeue_ism()
            self._process_ism(ism)

    def _dequeue_ism(self):
        ism = IngressBuffer.dequeue()
        return ism

    def _process_ism(self, ism):
        if ism.is_interface_node_message():
            self._process_interface_node_message(ism)
        else:
            self._process_management_message(ism)

    def _process_interface_node_message(self, ism):
        node = self._get_node_from_registry(ism)
        if node is not None:
            node.process_ism(ism)

    def _get_node_from_registry(self, ism):
        node_type = ism.get_node_type()
        node_name = ism.get_node_name()
        node = InterfaceNodesRegistry.get_node(node_type, node_name)
        return node

    def _process_management_message(self, ism):
        message_manager = self._get_message_manager(ism)
        message_manager.process_ism(ism)

    def _get_message_manager(self, ism):
        class_name = ism.get_message_manager()
        message_manager_module = self._import_module(class_name)
        message_manager_class = getattr(message_manager_module, class_name)
        message_manager = message_manager_class()
        return message_manager

    def _import_module(self, class_name):
        module_name = VariableFormatSerializer.camel_to_snake_case(class_name)
        module_path = f"rosagent.management.{module_name}"
        module = importlib.import_module(module_path)
        return module
