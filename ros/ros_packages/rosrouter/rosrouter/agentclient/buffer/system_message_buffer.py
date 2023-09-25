import queue

class SystemMessageBuffer:

    def __init__(self, size=0):
        self._size = size
        self._buffer = queue.Queue()

    def enqueue(self, data):
        self._buffer.put(data)

    def dequeue(self):
        return self._buffer.get()

    def get_size(self):
        return self._size()

    def is_empty(self):
        return self._buffer.empty()
