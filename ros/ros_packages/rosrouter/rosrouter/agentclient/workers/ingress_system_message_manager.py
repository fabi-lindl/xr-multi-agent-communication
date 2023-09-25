import importlib
from threading import Thread

from rosrouter.management.sending import message_distributor as MessageDistributor

from rosutility.message.serializer import variable_format_serializer as VariableFormatSerializer

class IngressSystemMessageManager(Thread):

    def __init__(self, buffer):
        Thread.__init__(self)
        self.daemon = True
        self._buffer = buffer

    def run(self):
        while True:
            ism = self._buffer.dequeue()
            self._process_ism(ism)
    
    def _process_ism(self, ism):
        if ism.is_management_message():
            self._process_management_message(ism)
        else:
            self._forward_ism(ism)
    
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
        module_path = f"rosrouter.management.{module_name}"
        module = importlib.import_module(module_path)
        return module

    def _forward_ism(self, ism):
        MessageDistributor.send_ism(ism)
