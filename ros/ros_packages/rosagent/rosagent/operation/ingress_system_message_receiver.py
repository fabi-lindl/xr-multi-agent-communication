from threading import Thread

from rosagent.buffers import ingress_buffer as IngressBuffer
from rosagent.message.ingress_system_message import IngressSystemMessage

from rosutility.message.serializer import deserializer as Deserializer
from rosutility.message.system_message_metadata import SystemMessageMetadata
from rosutility.socket.socket_transceiver import SocketTransceiver

class IngressSystemMessageReceiver(Thread):

    _instance = None

    def __new__(cls, socket):
        if cls._instance is None:
            cls._instance = (super(IngressSystemMessageReceiver, cls)
                             .__new__(cls))
            cls._instance._initialized = False
        return cls._instance

    def __init__(self, socket):
        if self._initialized:
            return
        self._initialized = True
        Thread.__init__(self)
        self.daemon = True
        self._socket = socket
        self._socket_transceiver = SocketTransceiver(self._socket)
        self._int_byte_size = 4
    
    def run(self):
        while True:
            self._receive_ism_into_buffer()

    def _receive_ism_into_buffer(self):
        metadata = self._read_metadata()
        data = self._read_data()
        ism = IngressSystemMessage(metadata, data)
        IngressBuffer.enqueue(ism)

    def _read_metadata(self):
        metadata_length = self._read_length_identifier()
        metadata_bytes = self._socket_transceiver.receive(metadata_length)
        received_metadata = Deserializer.bytes_to_dict(metadata_bytes)
        metadata = SystemMessageMetadata()
        metadata.init_from_received_message_metadata(received_metadata)
        return metadata

    def _read_data(self):
        data_length = self._read_length_identifier()
        data_bytes = self._socket_transceiver.receive(data_length)
        return data_bytes

    def _read_length_identifier(self):
        length_bytes = self._socket_transceiver.receive(self._int_byte_size)
        length = Deserializer.bytes_to_uint(length_bytes)
        return length
