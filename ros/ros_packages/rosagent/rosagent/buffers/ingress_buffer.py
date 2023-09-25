import queue

buffer = queue.Queue()

def enqueue(data):
    buffer.put(data)

def dequeue():
    return buffer.get()

def get_size():
    return buffer.qsize()

def is_empty():
    return buffer.empty()
