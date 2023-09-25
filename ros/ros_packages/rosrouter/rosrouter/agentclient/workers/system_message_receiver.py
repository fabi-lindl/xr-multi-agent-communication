from datetime import datetime
from threading import Thread

from rosrouter.message.ingress_system_message import IngressSystemMessage

from rosutility.message.serializer import deserializer as Deserializer
from rosutility.message.serializer.routing_data_deserializer import RoutingDataDeserializer
from rosutility.message.system_message_routing_info import SystemMessageRoutingInfo
from rosutility.socket.socket_transceiver import SocketTransceiver

class SystemMessageReceiver(Thread):

    def __init__(self, socket, buffer):
        Thread.__init__(self)
        self.daemon = True
        self._socket = socket
        self._socket_transceiver = SocketTransceiver(self._socket)
        self._buffer = buffer
        self._routing_data_deserializer = RoutingDataDeserializer()
        self._routing_data_size = (self._routing_data_deserializer
                                   .get_routing_data_size())
        self._last_received_timestamp = None

    def get_last_received_timestamp(self):
        return self._last_received_timestamp

    def run(self):
        while True:
            self._receive_sm_into_buffer()

    def _receive_sm_into_buffer(self):
        routing_info = self._read_routing_info()
        message = self._read_message(routing_info)
        ism = IngressSystemMessage(routing_info, message)
        self._buffer.enqueue(ism)
        self._set_last_received_timestamp()
    
    def _read_routing_info(self):
        routing_data = self._read_routing_data()
        management_data = self._read_management_data(routing_data)
        routing_info = SystemMessageRoutingInfo(routing_data, management_data)
        return routing_info

    def _read_routing_data(self):
        bytes_ = self._socket_transceiver.receive(self._routing_data_size)
        routing_data = self._routing_data_deserializer.deserialize(bytes_)
        return routing_data

    def _read_management_data(self, routing_data):
        size = routing_data.get_management_data_size()
        bytes_ = self._socket_transceiver.receive(size)
        management_data = Deserializer.bytes_to_dict(bytes_, True)
        return management_data

    def _read_message(self, routing_info):
        size = routing_info.get_total_message_size()
        message_bytes = self._socket_transceiver.receive(size)
        return message_bytes
    
    def _set_last_received_timestamp(self):
        now = datetime.now()
        self._last_received_timestamp = now
