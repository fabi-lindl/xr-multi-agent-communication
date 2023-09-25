from rosutility.logging.connection_logger import ConnectionLogger as Logger

class SocketTransceiver:

    _error_message = "Connection closed!"

    def __init__(self, socket):
        self._socket = socket
        self._logger = Logger()

    def send(self, data: bytes):
        data_length = len(data)
        num_bytes_sent = 0
        while num_bytes_sent < data_length:
            sent_chunk_size = self._send_data_chunk(data[num_bytes_sent:])
            assert self._is_connection_alive(sent_chunk_size), SocketTransceiver._error_message
            num_bytes_sent += sent_chunk_size

    def _send_data_chunk(self, data_chunk: bytes):
        try:
            sent_chunk_size = self._socket.send(data_chunk)
            return sent_chunk_size
        except BrokenPipeError:
            return 0

    def receive(self, message_length):
        buffer = bytearray(message_length)
        view = memoryview(buffer)
        num_bytes_received = 0
        while num_bytes_received < message_length:
            num_missing_bytes = message_length - num_bytes_received
            read_view = view[num_bytes_received:]
            recvd_chunk_size = self._receive_chunk(read_view, num_missing_bytes)
            assert self._is_connection_alive(recvd_chunk_size), SocketTransceiver._error_message
            num_bytes_received += recvd_chunk_size
        return bytes(buffer)

    def _receive_chunk(self, read_view, num_missing_bytes):
        max_chunk_size = 4096 # Good match for hardware and network (see docs)
        chunk_size = min(num_missing_bytes, max_chunk_size)
        try:
            recvd_chunk_size = self._socket.recv_into(read_view, chunk_size)
            return recvd_chunk_size
        except ConnectionResetError:
            return 0

    def _is_connection_alive(self, num_bytes):
        if num_bytes > 0:
            return True
        self._logger.log_connection_closed()
        return False
