from collections import deque

class EgressSystemMessageBuffer:

    def __init__(self, size=10, drop_new=True):
        self._buffer = deque(maxlen=size)
        self._size = size
        self._drop_new = drop_new
    
    def enqueue(self, message):
        if self.can_enqueue_item():
            self._buffer.append(message)
            return True
        return False
    
    def dequeue(self):
        try:
            return self._buffer.popleft()
        except IndexError:
            return None

    def get_num_items(self):
        return len(self._buffer)

    def get_size(self):
        return self._size

    def is_full(self):
        return not self._has_free_space()

    def can_enqueue_item(self):
        return self._drop_old() or self._has_free_space()

    def _drop_old(self):
        return not self._drop_new

    def _has_free_space(self):
        return len(self._buffer) < self._size
    