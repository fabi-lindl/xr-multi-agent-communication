from datetime import datetime

from rosrouter.agentclient.buffer.system_message_buffer import SystemMessageBuffer
from rosrouter.agentclient.workers.system_message_receiver import SystemMessageReceiver
from rosrouter.agentclient.workers.system_message_sender import SystemMessageSender
from rosrouter.agentclient.workers.ingress_system_message_manager import IngressSystemMessageManager

from rosutility.config.params.getter import network_param_getter as NetworkParamGetter

class AgentclientConnectionManager:

    _expiration_delta_time = NetworkParamGetter.get_connection_pruning_period()

    def __init__(self, socket, address: tuple):
        self._socket = socket
        self._address = address

        self._ingress_buffer = SystemMessageBuffer()
        self._egress_buffer = SystemMessageBuffer()
        
        self._sm_sender = SystemMessageSender(self._socket,
                                              self._egress_buffer)
        self._sm_receiver = SystemMessageReceiver(self._socket,
                                                  self._ingress_buffer)
        self._ism_manager = IngressSystemMessageManager(self._ingress_buffer)

    def get_address(self):
        return self._address

    def get_ip_address(self):
        return self._address[0]
    
    def get_port(self):
        return self._address[1]

    def send(self, message: bytes):
        self._egress_buffer.enqueue(message)

    def run(self):
        self._sm_sender.start()
        self._sm_receiver.start()
        self._ism_manager.start()
    
    def is_connection_alive(self):
        now = datetime.now()
        last_received_timestamp = (self._sm_receiver
                                   .get_last_received_timestamp())
        delta_time = (now - last_received_timestamp).total_seconds()
        return delta_time < AgentclientConnectionManager._expiration_delta_time
    
    def close_connection(self):
        self._socket.close()
