from threading import Thread

from rosutility.socket.socket_transceiver import SocketTransceiver

class SystemMessageSender(Thread):

    def __init__(self, socket, buffer):
        Thread.__init__(self)
        self.daemon = True
        self._socket = socket
        self._socket_transceiver = SocketTransceiver(self._socket)
        self._buffer = buffer

    def run(self):
        while True:
            system_message = self._buffer.dequeue()
            self._socket_transceiver.send(system_message)
