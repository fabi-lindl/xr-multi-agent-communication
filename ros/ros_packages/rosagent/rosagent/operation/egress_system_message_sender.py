from threading import Thread

from rosagent.buffers import egress_buffer as EgressBuffer
from rosagent.message.heartbeat_message import HeartbeatMessage

from rosutility.socket.socket_transceiver import SocketTransceiver

class EgressSystemMessageSender(Thread):

    _instance = None

    def __new__(cls, socket):
        if cls._instance is None:
            cls._instance = super(EgressSystemMessageSender, cls).__new__(cls)
            cls._instance._initialized = False
        return cls._instance

    def __init__(self, socket):
        if self._initialized: return
        self._initialized = True
        Thread.__init__(self)
        self.daemon = True
        self.socket = socket
        self._socket_transceiver = SocketTransceiver(socket)
        self._heartbeat_message = HeartbeatMessage()

    def run(self):
        while True:
            self.send_esm()

    def send_esm(self):
        esm = self.dequeue_esm()
        if esm is None: return
        esm_bytes = esm.serialize()
        self._socket_transceiver.send(esm_bytes)
    
    def dequeue_esm(self):
        esm_buffer = EgressBuffer.dequeue()
        if self.is_heartbeat(esm_buffer):
            esm = self._heartbeat_message.get()
        else:
            esm = esm_buffer.dequeue()
        return esm
    
    def is_heartbeat(self, value):
        return value is None
